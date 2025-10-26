# OpalCar — Wi‑Fi Telemetry + Smart Sensing

ESP32-based laser-following car with robust Smart Sensing and lightweight UDP telemetry. The car only sends data via Wi‑Fi; there is no HTML or WebServer on the vehicle.

## Features

- Smart Sensing (robust ambient rejection):
  - Median/MAD calibration over 40 samples
  - Dynamic thresholds: `max(minAbsThreshold, baseline * sensorThresholdRatio + kMAD * MAD)`
  - Local contrast check
  - Cluster width limit (max 3 columns)
  - Ambient-saturation block (up to ~300 ms if ≥6 channels saturated at ~950/1024)
  - Adaptive baseline: β=0.01 normal, β_fast=0.05 during ambient block
- Control:
  - Time-based debouncer (foundMs=40, lostMs=60)
  - Non-blocking electric brake (140 ms), then coasting
  - Movement profiles mapped to detected centroid
  - Speed latch via GPIO34 (buttonStableMs=70 ms)
- Telemetry:
  - Car sends UDP broadcast every ~50 ms on port 4210
  - SoftAP mode by default (SSID: `OpalCar`, password: `opal1234`)
  - Compact JSON payload with sensors, baseline, state, speed, angle, etc.
- Receiver GUI (Python):
  - PySide6 + pyqtgraph visualization
  - 10×9 heatmap (10 columns resampled to 9; rows tiled)
  - Gain and Clamp controls; active columns overlay; primary centroid

## Repository Layout

- Firmware (Arduino/ESP32)
  - `test03_debouncing_betterBrakes.ino`
  - `test04.ino`
  - `test04_faster.ino`
  - `test06.ino`
  - `test07-SmartSensing.ino` ← latest and recommended
- Receiver (Python)
  - `telem_app.py` ← UDP visualization GUI
  - `requirements.txt` ← Python dependencies

## Firmware Setup (ESP32)

1. Open `test07-SmartSensing.ino` in Arduino IDE.
2. Board: ESP32 Dev Module (or your exact ESP32 variant).
3. Libraries:
   - Adafruit ADS1X15
   - ESP32Servo
   - WiFi (comes with ESP32 core)
4. Pins and hardware:
   - 3× ADS1115 (addresses: 0x48, 0x49, 0x4A)
   - L298N driver (ENA=23, IN1=18, IN2=19)
   - Servo on pin 5 (center ~100)
   - Button on GPIO34 (INPUT)
5. Upload to ESP32.
6. Default Wi‑Fi:
   - Mode: SoftAP (ESP32 creates hotspot)
   - SSID: `OpalCar`
   - Password: `opal1234`
   - UDP broadcast every ~50 ms to port `4210`

## Receiver Setup (Python GUI)

1. Connect your laptop to Wi‑Fi SSID `OpalCar` (password `opal1234`).
2. Open firewall for UDP 4210:
   - Windows: Allow Python on UDP 4210 (Windows Security → Firewall → Advanced settings → Inbound Rule).
   - macOS: System Settings → Network → Firewall → allow Python/Terminal.
   - Linux (ufw): `sudo ufw allow 4210/udp`.

3. Create a virtual environment and install dependencies:
```bash
python -m venv .venv
# Windows
.\.venv\Scripts\activate
# macOS/Linux
source .venv/bin/activate

pip install --upgrade pip
pip install -r requirements.txt
# or:
pip install PySide6 pyqtgraph numpy

4. Run the GUI:
bash
python telem_app.py

You should see packets arriving at ~20 FPS when connected to `OpalCar`.

### requirements.txt
text
PySide6
pyqtgraph
numpy

## Telemetry JSON (UDP, sent by car)

- Port: 4210
- Interval: ~50 ms
- Example fields:
json
{
  "t": 123456,
  "state": 1,
  "detect": true,
  "primary": 5,
  "active": [4,5,6],
  "cols": [ ...10 values... ],
  "base": [ ...10 values... ],
  "speed": 60.0,
  "angle": 100,
  "ambient": false
}
- States: 0=IDLE, 1=FOLLOWING, 2=BRAKING, 3=STOPPED

## Wi‑Fi Modes

- Default (recommended): SoftAP
  - ESP32 creates `OpalCar` hotspot (typical AP IP: 192.168.4.1).
  - Receiver binds to `0.0.0.0:4210` and receives UDP broadcast.
  - Offline, simple, no router required.

- Optional (advanced): Station (STA) on home router
  - Change firmware to `WIFI_STA`, set your router’s SSID/password.
  - Ensure both laptop and car on same subnet.
  - If broadcast is filtered, send Unicast to laptop IP.
  - Keep firewall open for UDP 4210.

## Troubleshooting

- No packets in GUI:
  - Verify laptop is connected to `OpalCar` and has IP `192.168.4.x`.
  - Temporarily disable firewall or add an allow rule for UDP 4210.
  - Disable VPN or other network adapters that might capture routes.
  - Reduce distance; SoftAP range is limited.

- Lag/dropped frames:
  - Move closer; avoid interference-heavy environments.
  - Increase `udpIntervalMs` in firmware (e.g., 50 → 80–100 ms).

- GUI error “ImageItem.scale() takes no arguments”:
  - Fixed in latest `telem_app.py` by initializing image before `setRect/scale`.
  - Update to the current GUI code if you see this.

## Configuration Tips

- Change UDP port:
  - Firmware: update the telemetry port constant (default 4210).
  - Receiver: edit `UDP_PORT` in `telem_app.py`.

- Multiple cars:
  - Use distinct SSIDs or ports per car.
  - Run multiple receiver instances on different ports.

## Development Notes

- Arduino IDE:
  - ESP32 board package via Boards Manager.
  - Serial monitor: 115200 baud.
- Python:
  - Tested on Python 3.9–3.12 (Windows/macOS/Linux).
  - PySide6 (Qt) + pyqtgraph for fast image updates.

## License

- Hardware/firmware: ESP32 + ADS1115 + L298N + Servo
- Telemetry: UDP-only; no web services on the car
- Open to contributions—please file issues/PRs.

