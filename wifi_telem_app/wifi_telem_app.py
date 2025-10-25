# UDP Telemetry GUI for OpalCar
# Requirements: pip install pyside6 pyqtgraph
import sys
import json
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg

UDP_PORT = 4210
BUFFER_SIZE = 4096

@dataclass
class Telemetry:
    t: int = 0
    state: int = 0  # 0 IDLE, 1 FOLLOWING, 2 BRAKING, 3 STOPPED
    detect: bool = False
    primary: int = -1
    active: List[int] = field(default_factory=list)
    cols: List[int] = field(default_factory=lambda: [0]*10)
    base: List[int] = field(default_factory=lambda: [0]*10)
    speed: float = 0.0
    angle: int = 0
    ambient: bool = False

class UdpListener(QtCore.QObject):
    packetReceived = QtCore.Signal(object, str)  # Telemetry, sender_ip

    def __init__(self, port=UDP_PORT, parent=None):
        super().__init__(parent)
        self.port = port
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('', self.port))
        sock.settimeout(0.2)

        while self._running:
            try:
                data, addr = sock.recvfrom(BUFFER_SIZE)
            except socket.timeout:
                continue
            except Exception:
                time.sleep(0.1)
                continue

            sender_ip = addr[0]
            try:
                payload = data.decode('utf-8', errors='ignore')
                obj = json.loads(payload)
                tel = Telemetry(
                    t=obj.get('t', 0),
                    state=obj.get('state', 0),
                    detect=obj.get('detect', False),
                    primary=obj.get('primary', -1),
                    active=obj.get('active', []),
                    cols=obj.get('cols', [0]*10),
                    base=obj.get('base', [0]*10),
                    speed=obj.get('speed', 0.0),
                    angle=obj.get('angle', 0),
                    ambient=obj.get('ambient', False)
                )
                self.packetReceived.emit(tel, sender_ip)
            except Exception:
                continue

        try:
            sock.close()
        except:
            pass

class HeatmapWidget(pg.GraphicsLayoutWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackground('w')

        self.view = self.addViewBox(row=0, col=0)
        self.view.setAspectLocked(True)
        self.img = pg.ImageItem()
        self.img.setZValue(0)
        self.view.addItem(self.img)

        # 1) ابتدا تصویر اولیه را تنظیم کن (10×9 RGBA)
        h, w = 10, 9
        green = np.zeros((h, w), dtype=np.uint8)
        red   = np.zeros((h, w), dtype=np.uint8)
        blue  = np.zeros((h, w), dtype=np.uint8)
        alpha = np.full((h, w), 255, dtype=np.uint8)
        rgba0 = np.dstack([red, green, blue, alpha])
        self.img.setImage(rgba0, levels=(0,255))

        # 2) سپس مستطیل نمایش را تعیین کن
        self.img.setRect(QtCore.QRectF(0, 0, w, h))

        # Grid overlay
        self.gridLines = []
        pen = pg.mkPen(color=(180,180,180,180), width=1)
        for r in range(11):  # 10 rows -> 11 horizontal lines
            y = r
            line = pg.InfiniteLine(pos=y, angle=0, pen=pen, movable=False)
            line.setZValue(10)
            self.view.addItem(line)
            self.gridLines.append(line)
        for c in range(10):  # 9 cols -> 10 vertical lines
            x = c
            line = pg.InfiniteLine(pos=x, angle=90, pen=pen, movable=False)
            line.setZValue(10)
            self.view.addItem(line)
            self.gridLines.append(line)

        # Text per column (top labels)
        self.colTexts = [pg.TextItem("", color=(20,20,20), anchor=(0.5, -0.1)) for _ in range(9)]
        for i, t in enumerate(self.colTexts):
            t.setZValue(20)
            self.view.addItem(t)

        # Active column highlight bars (top)
        self.activeBars = [pg.InfiniteLine(angle=90, pen=pg.mkPen(color=(0,180,0,200), width=3)) for _ in range(9)]
        for bar in self.activeBars:
            bar.setVisible(False)
            bar.setZValue(15)
            self.view.addItem(bar)

        # State info
        self.infoText = pg.TextItem("", color=(0,0,0), anchor=(0,0))
        self.infoText.setZValue(30)
        self.view.addItem(self.infoText)

        self.view.setRange(xRange=(0, 9), yRange=(0, 10), padding=0.02)

    def updateHeatmap(self, mat: np.ndarray, cols9: np.ndarray, bases9: np.ndarray, tel: Telemetry, gain: float, clamp: float, showNumbers: bool):
        # mat is 10x9 in [0..1] intensity
        h, w = mat.shape

        # Build RGBA image; green channel encodes intensity
        green = (np.clip(mat * gain, 0.0, 1.0) * 255.0).astype(np.uint8)
        red = np.zeros_like(green, dtype=np.uint8)
        blue = np.zeros_like(green, dtype=np.uint8)
        alpha = np.full_like(green, 255, dtype=np.uint8)
        rgba = np.dstack([red, green, blue, alpha])

        self.img.setImage(rgba, levels=(0,255))
        self.img.setRect(QtCore.QRectF(0, 0, w, h))

        # Update column labels
        for i in range(9):
            txt = f"C:{int(round(cols9[i]))}  B:{int(round(bases9[i]))}" if showNumbers else ""
            self.colTexts[i].setText(txt)
            self.colTexts[i].setPos(i+0.5, 10.0)

        # Active highlights based on original 10 columns mapped to 9
        active9 = map_active_to9(tel.active)
        for i in range(9):
            self.activeBars[i].setPos(i)
            self.activeBars[i].setVisible(active9[i])

        # Info text
        stateNames = ["IDLE","FOLLOWING","BRAKING","STOPPED"]
        st = stateNames[tel.state] if 0 <= tel.state < len(stateNames) else str(tel.state)
        info = f"t={tel.t} ms | state={st} | detect={tel.detect} | primary={tel.primary} | speed={tel.speed:.1f} | angle={tel.angle} | ambient={tel.ambient}"
        self.infoText.setText(info)
        self.infoText.setPos(0.1, 9.7)

def resample10_to9(arr10: np.ndarray) -> np.ndarray:
    # Linear resample 10 -> 9 over same physical span
    src_x = np.linspace(0, 9, 10)   # 10 samples
    tgt_x = np.linspace(0, 9, 9)    # 9 samples
    return np.interp(tgt_x, src_x, arr10)

def map_active_to9(active10: List[int]) -> List[bool]:
    # Map 0..9 -> 0..8 using floor(i*9/10)
    flags = [False]*9
    for i in active10:
        if 0 <= i <= 9:
            ti = int(np.floor(i * 9.0 / 10.0))
            if ti < 0: ti = 0
            if ti > 8: ti = 8
            flags[ti] = True
    return flags

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpalCar Telemetry GUI (UDP 4210)")
        self.resize(1000, 600)

        self.tel: Optional[Telemetry] = None
        self.lastSenderIp: Optional[str] = None

        # Central UI
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # Left: Heatmap
        self.heatmap = HeatmapWidget()
        layout.addWidget(self.heatmap, 3)

        # Right: Controls and info
        right = QtWidgets.QVBoxLayout()
        layout.addLayout(right, 2)

        self.lblConn = QtWidgets.QLabel("Status: waiting for packets...")
        right.addWidget(self.lblConn)

        self.gainSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.gainSlider.setRange(1, 50)  # 0.05..2.5 gain (scaled later)
        self.gainSlider.setValue(10)
        right.addWidget(QtWidgets.QLabel("Gain"))
        right.addWidget(self.gainSlider)

        self.clampSpin = QtWidgets.QSpinBox()
        self.clampSpin.setRange(10, 1024)
        self.clampSpin.setValue(200)
        right.addWidget(QtWidgets.QLabel("Clamp (diff cap)"))
        right.addWidget(self.clampSpin)

        self.showNumbersChk = QtWidgets.QCheckBox("Show column numbers (cols/base)")
        self.showNumbersChk.setChecked(True)
        right.addWidget(self.showNumbersChk)

        # Packet info textbox
        self.textLog = QtWidgets.QTextEdit()
        self.textLog.setReadOnly(True)
        right.addWidget(QtWidgets.QLabel("Last packet JSON"))
        right.addWidget(self.textLog, 1)

        # UDP listener
        self.listener = UdpListener(port=UDP_PORT)
        self.listener.packetReceived.connect(self.onPacket)
        self.listener.start()

        # Update timer ~50ms
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.onTimer)
        self.timer.start()

    @QtCore.Slot(object, str)
    def onPacket(self, tel: Telemetry, sender_ip: str):
        self.tel = tel
        self.lastSenderIp = sender_ip
        self.lblConn.setText(f"Receiving from {sender_ip}")
        # Show compact JSON
        try:
            compact = {
                "t": tel.t, "state": tel.state, "detect": tel.detect,
                "primary": tel.primary, "active": tel.active,
                "speed": tel.speed, "angle": tel.angle, "ambient": tel.ambient
            }
            self.textLog.setPlainText(json.dumps(compact, ensure_ascii=False))
        except Exception:
            pass

    @QtCore.Slot()
    def onTimer(self):
        if self.tel is None:
            return
        # Prepare arrays
        cols = np.array(self.tel.cols, dtype=float)
        base = np.array(self.tel.base, dtype=float)
        diff = cols - base
        diff[diff < 0] = 0.0

        clamp = float(self.clampSpin.value())
        gain = float(self.gainSlider.value()) / 20.0  # 0.05..2.5

        # Normalize intensity
        diff_norm = np.clip(diff / max(1.0, clamp), 0.0, 1.0)

        # Resample 10->9
        diff9 = resample10_to9(diff_norm)
        cols9 = resample10_to9(cols)
        bases9 = resample10_to9(base)

        # Form 10x9 matrix by tiling rows
        mat = np.tile(diff9, (10, 1))  # 10 rows, 9 cols

        # Optional: slightly boost active columns
        active9 = map_active_to9(self.tel.active)
        for i, isActive in enumerate(active9):
            if isActive:
                mat[:, i] = np.clip(mat[:, i] * 1.3 + 0.1, 0.0, 1.0)

        self.heatmap.updateHeatmap(mat, cols9, bases9, self.tel, gain=gain, clamp=clamp, showNumbers=self.showNumbersChk.isChecked())

def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
