#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP32Servo.h>

// ═══════════════════════════════════════════════════════════════
// تنظیمات سرعت و حرکت
// ═══════════════════════════════════════════════════════════════
float baseSpeedPercent      = 60.0;
float speedStepPercent      = 5.0;
float turnSpeedFactor       = 0.7;

int servoPin                = 5;
int servoCenter             = 100;

float sensorThresholdRatio  = 0.5;

unsigned long ledSpeedShowTime = 2000;
unsigned long buttonDebounceMs = 150;

bool verboseSerial = true;

// ═══════════════════════════════════════════════════════════════
// پین‌های موتور و دکمه
// ═══════════════════════════════════════════════════════════════
#define ENA  23
#define IN1  18
#define IN2  19
#define BTN  34

// ═══════════════════════════════════════════════════════════════
// ADC ها
// ═══════════════════════════════════════════════════════════════
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;
#define RDY1 25
#define RDY2 26
#define RDY3 27

// ═══════════════════════════════════════════════════════════════
// داده‌های سنسور
// ═══════════════════════════════════════════════════════════════
int columns[10];
int baseline[10];

int leds[10] = {15, 2, 14, 4, 16, 17, 13, 12, 32, 33};

// ═══════════════════════════════════════════════════════════════
// متغیرهای وضعیت
// ═══════════════════════════════════════════════════════════════
float speedPercent = baseSpeedPercent;
bool lastBtn = HIGH;
unsigned long lastBtnTime = 0;
unsigned long lastChangeTime = 0;
bool showingSpeed = false;

Servo myServo;

// ═══════════════════════════════════════════════════════════════
// State Machine
// ═══════════════════════════════════════════════════════════════
enum RobotState {
  STATE_IDLE,
  STATE_TRACKING,
  STATE_BRAKING,
  STATE_LOST_SIGNAL
};

RobotState currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
int lastActiveCol = -1;

// ═══════════════════════════════════════════════════════════════
// توابع موتور
// ═══════════════════════════════════════════════════════════════
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void moveBack() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}

void stopMotor() {
  updateMotor(0);
}

void updateMotor(float percent) {
  percent = constrain(percent, 0, 100);
  int pwm = map((int)percent, 0, 100, 0, 255);
  analogWrite(ENA, pwm);
}

// ═══════════════════════════════════════════════════════════════
// توابع LED
// ═══════════════════════════════════════════════════════════════
void updateLedsSpeed() {
  int n = (speedPercent - baseSpeedPercent) / speedStepPercent;
  if (speedPercent == baseSpeedPercent) n = 0;
  for (int i = 0; i < 10; i++) {
    if (i < 2 || (i < n + 2)) {
      digitalWrite(leds[i], HIGH);
    } else {
      digitalWrite(leds[i], LOW);
    }
  }
}

void turnOffLeds() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(leds[i], LOW);
  }
}

void updateLedsSensors() {
  for (int i = 0; i < 10; i++) {
    int dynamicThreshold = (int)(baseline[i] * sensorThresholdRatio);
    bool active = (columns[i] > baseline[i] + dynamicThreshold);
    digitalWrite(leds[9 - i], active ? HIGH : LOW);
  }
}

// ═══════════════════════════════════════════════════════════════
// خواندن سنسورها
// ═══════════════════════════════════════════════════════════════
void readSensors() {
  for (int i = 0; i < 4; i++) {
    int16_t raw = ads1.readADC_SingleEnded(i);
    columns[i] = constrain(map(raw, 0, 32767, 0, 1024), 0, 1024);
  }
  for (int i = 0; i < 4; i++) {
    int16_t raw = ads2.readADC_SingleEnded(i);
    columns[i + 4] = constrain(map(raw, 0, 32767, 0, 1024), 0, 1024);
  }
  for (int i = 0; i < 2; i++) {
    int16_t raw = ads3.readADC_SingleEnded(i);
    columns[i + 8] = constrain(map(raw, 0, 32767, 0, 1024), 0, 1024);
  }
}

// ═══════════════════════════════════════════════════════════════
// کالیبراسیون با نمایش پیشرفت
// ═══════════════════════════════════════════════════════════════
void calibrateWithProgress() {
  const int totalTime = 2000;
  const int samples = 40;
  long sum[10] = {0};

  if (verboseSerial) {
    Serial.println("Starting calibration...");
  }

  for (int s = 0; s < samples; s++) {
    readSensors();
    for (int i = 0; i < 10; i++) {
      sum[i] += columns[i];
    }
    
    int progressStep = map(s, 0, samples - 1, 0, 9);
    for (int i = 0; i < 10; i++) {
      digitalWrite(leds[i], i <= progressStep ? HIGH : LOW);
    }
    
    delay(totalTime / samples);
  }

  for (int i = 0; i < 10; i++) {
    baseline[i] = sum[i] / samples;
    digitalWrite(leds[i], LOW);
  }

  if (verboseSerial) {
    Serial.println("Calibration done. Baseline:");
    for (int i = 0; i < 10; i++) {
      Serial.printf("%4d ", baseline[i]);
    }
    Serial.println();
  }
}

// ═══════════════════════════════════════════════════════════════
// تشخیص ستون فعال
// ═══════════════════════════════════════════════════════════════
int detectActiveColumn() {
  for (int i = 0; i < 10; i++) {
    int dynamicThreshold = (int)(baseline[i] * sensorThresholdRatio);
    if (columns[i] > baseline[i] + dynamicThreshold) {
      return i;
    }
  }
  return -1;
}

// ═══════════════════════════════════════════════════════════════
// ردیابی لیزر (محاسبه زاویه سروو و سرعت)
// ═══════════════════════════════════════════════════════════════
void trackLaser(int activeCol, int &servoAngle, float &motorSpeed) {
  // محاسبه زاویه سروو بر اساس ستون فعال
  if (activeCol == 0) {
    servoAngle = 130;
  } else if (activeCol == 1 || activeCol == 2) {
    servoAngle = 115;
  } else if (activeCol >= 3 && activeCol <= 6) {
    servoAngle = 100;
  } else if (activeCol == 7 || activeCol == 8) {
    servoAngle = 85;
  } else if (activeCol == 9) {
    servoAngle = 70;
  } else {
    servoAngle = servoCenter;
  }

  // محاسبه سرعت موتور
  motorSpeed = speedPercent;
  if (servoAngle != servoCenter) {
    motorSpeed *= turnSpeedFactor;
  }
}

// ═══════════════════════════════════════════════════════════════
// تغییر وضعیت
// ═══════════════════════════════════════════════════════════════
void changeState(RobotState newState) {
  if (currentState != newState) {
    currentState = newState;
    stateStartTime = millis();
    
    if (verboseSerial) {
      Serial.print("State changed to: ");
      switch (newState) {
        case STATE_IDLE:         Serial.println("IDLE"); break;
        case STATE_TRACKING:     Serial.println("TRACKING"); break;
        case STATE_BRAKING:      Serial.println("BRAKING"); break;
        case STATE_LOST_SIGNAL:  Serial.println("LOST_SIGNAL"); break;
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// کنترل ربات (State Machine)
// ═══════════════════════════════════════════════════════════════
void controlRobot() {
  int activeCol = detectActiveColumn();
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - stateStartTime;

  switch (currentState) {
    
    // ───────────────────────────────────────────────────────────
    case STATE_IDLE:
    // ───────────────────────────────────────────────────────────
      if (activeCol != -1) {
        changeState(STATE_TRACKING);
        lastActiveCol = activeCol;
      } else {
        myServo.write(servoCenter);
        stopMotor();
      }
      break;

    // ───────────────────────────────────────────────────────────
    case STATE_TRACKING:
    // ───────────────────────────────────────────────────────────
      if (activeCol != -1) {
        int servoAngle;
        float motorSpeed;
        trackLaser(activeCol, servoAngle, motorSpeed);
        
        myServo.write(servoAngle);
        moveForward();
        updateMotor(motorSpeed);
        
        lastActiveCol = activeCol;
        
        if (verboseSerial) {
          Serial.printf("TRACKING: Col=%d | Servo=%d | Speed=%.1f%%\n", 
                        activeCol + 1, servoAngle, motorSpeed);
        }
      } else {
        // سیگنال از دست رفت - برو به حالت ترمز
        changeState(STATE_BRAKING);
      }
      break;

    // ───────────────────────────────────────────────────────────
    case STATE_BRAKING:
    // ───────────────────────────────────────────────────────────
      if (activeCol != -1) {
        // سیگنال برگشت - برو به حالت ردیابی
        changeState(STATE_TRACKING);
      } else if (elapsedTime < 150) {
        // در حال ترمز (حرکت به عقب)
        myServo.write(servoCenter);
        moveBack();
        updateMotor(speedPercent);
      } else {
        // ترمز تمام شد - توقف و رفتن به حالت گم‌شده
        stopMotor();
        moveForward();
        changeState(STATE_LOST_SIGNAL);
        
        if (verboseSerial) {
          Serial.println("Braking complete -> LOST_SIGNAL");
        }
      }
      break;

    // ───────────────────────────────────────────────────────────
    case STATE_LOST_SIGNAL:
    // ───────────────────────────────────────────────────────────
      if (activeCol != -1) {
        // سیگنال پیدا شد - برگرد به ردیابی
        changeState(STATE_TRACKING);
      } else {
        // منتظر بمان
        myServo.write(servoCenter);
        stopMotor();
      }
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  
  if (verboseSerial) {
    Serial.println("\n\n═══════════════════════════════════════");
    Serial.println("   Laser Tracking Robot - Starting");
    Serial.println("═══════════════════════════════════════");
  }

  // راه‌اندازی I2C
  Wire.begin(21, 22);
  Wire.setClock(400000); // 400kHz برای سرعت بهتر

  // تنظیم پین‌های موتور
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BTN, INPUT_PULLUP);

  // تنظیم LEDها
  for (int i = 0; i < 10; i++) {
    pinMode(leds[i], OUTPUT);
    digitalWrite(leds[i], LOW);
  }

  // راه‌اندازی سروو
  myServo.attach(servoPin, 500, 2400);
  myServo.write(servoCenter);

  // راه‌اندازی ADCها
  if (!ads1.begin(0x48)) {
    Serial.println("ERROR: ADS1 not found!");
    while (1);
  }
  if (!ads2.begin(0x49)) {
    Serial.println("ERROR: ADS2 not found!");
    while (1);
  }
  if (!ads3.begin(0x4A)) {
    Serial.println("ERROR: ADS3 not found!");
    while (1);
  }

  ads1.setGain(GAIN_ONE);
  ads2.setGain(GAIN_ONE);
  ads3.setGain(GAIN_ONE);

  pinMode(RDY1, INPUT);
  pinMode(RDY2, INPUT);
  pinMode(RDY3, INPUT);

  // تست موتور
  if (verboseSerial) Serial.println("Testing motor...");
  moveForward();
  delay(200);
  moveBack();
  delay(200);
  stopMotor();
  moveForward();

  // کالیبراسیون
  calibrateWithProgress();

  // آماده
  currentState = STATE_IDLE;
  stateStartTime = millis();
  
  if (verboseSerial) {
    Serial.println("═══════════════════════════════════════");
    Serial.println("   System Ready - Waiting for laser");
    Serial.println("═══════════════════════════════════════\n");
  }
}

// ═══════════════════════════════════════════════════════════════
// Loop
// ═══════════════════════════════════════════════════════════════
void loop() {
  bool btn = digitalRead(BTN);
  unsigned long now = millis();

  // ─────────────────────────────────────────────────────────────
  // مدیریت دکمه (تغییر سرعت)
  // ─────────────────────────────────────────────────────────────
  if (lastBtn == HIGH && btn == LOW && (now - lastBtnTime) > buttonDebounceMs) {
    speedPercent += speedStepPercent;
    if (speedPercent > 100) {
      speedPercent = baseSpeedPercent;
    }

    if (verboseSerial) {
      Serial.print("Button pressed -> New speed: ");
      Serial.print(speedPercent);
      Serial.println("%");
    }

    updateLedsSpeed();
    showingSpeed = true;
    lastChangeTime = now;
    lastBtnTime = now;
  }

  // ─────────────────────────────────────────────────────────────
  // پایان نمایش سرعت
  // ─────────────────────────────────────────────────────────────
  if (showingSpeed && (now - lastChangeTime >= ledSpeedShowTime)) {
    showingSpeed = false;
    turnOffLeds();
  }

  // ─────────────────────────────────────────────────────────────
  // عملیات اصلی
  // ─────────────────────────────────────────────────────────────
  if (!showingSpeed) {
    readSensors();
    updateLedsSensors();
    controlRobot();
  }

  lastBtn = btn;
  
  // تاخیر کوچک برای جلوگیری از مصرف بیش از حد CPU
  delay(10);
}
