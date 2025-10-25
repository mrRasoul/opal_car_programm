#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <ESP32Servo.h>

// ============ Configuration Parameters ============
float baseSpeedPercent      = 60.0;
float speedStepPercent      = 5.0;

int servoPin                = 5;
int servoCenter             = 100;
int servoLeft               = 130;
int servoRight              = 60;

float sensorThresholdRatio  = 0.5;  // حفظ مقدار 0.5 طبق درخواست

unsigned long ledSpeedShowTime = 2000;
unsigned long buttonDebounceMs = 150;

bool verboseSerial = true;
bool telemetryEnabled = true;

// ============ Hardware Pins ============
#define ENA  23
#define IN1  18
#define IN2  19
#define BTN  34

#define RDY1 25
#define RDY2 26
#define RDY3 27

// ============ Movement Profile Structure ============
struct MovementProfile {
    int startCol;
    int endCol;
    int servoAngle;
    float speedMultiplier;
    const char* description;
};

// Movement profiles configuration
MovementProfile profiles[] = {
    {0, 2, 130, 0.7, "Sharp Left"},   // چپ شدید
    {3, 6, 100, 1.0, "Straight"},     // مستقیم
    {7, 9, 70,  0.7, "Sharp Right"}   // راست شدید
};
const int NUM_PROFILES = sizeof(profiles) / sizeof(profiles[0]);

// ============ Detection Result Structure ============
struct DetectionResult {
    bool detected;
    int primaryColumn;
    int activeColumns[10];
    int activeCount;
    int maxSignalStrength;
};

// ============ Robot States ============
enum RobotState {
    IDLE,
    FOLLOWING,
    BRAKING,
    STOPPED
};

// ============ 🆕 Laser Debouncer Class ============
class LaserDebouncer {
private:
    int lostCounter = 0;      // شمارنده از دست دادن لیزر
    int foundCounter = 0;     // شمارنده پیدا کردن لیزر
    bool lastStableState = false;   // آخرین وضعیت پایدار
    
    // آستانه‌های debouncing - قابل تنظیم
    const int LOST_THRESHOLD = 2;   // باید 5 بار متوالی لیزر دیده نشود
    const int FOUND_THRESHOLD = 3;  // باید 3 بار متوالی لیزر دیده شود
    
public:
    bool getStableDetection(bool currentDetection) {
        if (currentDetection) {
            // لیزر تشخیص داده شد
            foundCounter++;
            lostCounter = 0;  // ریست شمارنده قطعی
            
            if (foundCounter >= FOUND_THRESHOLD) {
                lastStableState = true;
                foundCounter = FOUND_THRESHOLD;  // جلوگیری از overflow
            }
        } else {
            // لیزر تشخیص داده نشد
            lostCounter++;
            foundCounter = 0;  // ریست شمارنده تشخیص
            
            if (lostCounter >= LOST_THRESHOLD) {
                lastStableState = false;
                lostCounter = LOST_THRESHOLD;  // جلوگیری از overflow
            }
        }
        
        return lastStableState;
    }
    
    // متد دیباگ برای نمایش وضعیت
    void printDebugInfo() {
        Serial.print(" | Lost:");
        Serial.print(lostCounter);
        Serial.print(" Found:");
        Serial.print(foundCounter);
        Serial.print(" Stable:");
        Serial.print(lastStableState ? "YES" : "NO");
    }
    
    // ریست کردن debouncer
    void reset() {
        lostCounter = 0;
        foundCounter = 0;
        lastStableState = false;
    }
};

// ============ Enhanced Brake Manager Class ============
class BrakeManager {
private:
    bool stopped = false;

public:
    void start() {
        if (!stopped) {
            // فقط یکبار: حرکت معکوس کوتاه و بعد توقف کامل
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(ENA, 200);
            
            delay(500);  // حرکت معکوس

            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(ENA, 0);
            
            // // توقف کامل - بدون ترمز الکتریکی
            // digitalWrite(IN1, LOW);
            // digitalWrite(IN2, LOW);
            // analogWrite(ENA, 0);
            
            stopped = true;
            Serial.println("[BRAKE] Stopped completely");
        }
    }
    
    void reset() {
        stopped = false;
    }
    
    bool isStopped() {
        return stopped;
    }
};

//     // تنظیم پارامترهای ترمز (اختیاری - برای تست)
//     void configure(int reverseDuration, int reverseSpeed, bool electricBrake) {
//         config.reverseDuration = reverseDuration;
//         config.reverseSpeed = constrain(reverseSpeed, 0, 255);
//         config.useElectricBrake = electricBrake;
//     }
// };

// ============ Movement Controller Class ============
class MovementController {
private:
    float baseSpeed;
    float currentSpeed;
    int currentAngle;
    MovementProfile* activeProfile;
    Servo* servo;

public:
    void init(Servo* servoRef, float speed) {
        servo = servoRef;
        baseSpeed = speed;
        currentSpeed = 0;
        currentAngle = servoCenter;
        activeProfile = nullptr;
    }

    void setBaseSpeed(float speed) {
        baseSpeed = speed;
        if (activeProfile != nullptr) {
            currentSpeed = baseSpeed * activeProfile->speedMultiplier;
        }
    }

    bool applyProfile(int column) {
        if (column < 0 || column > 9) return false;

        for (int i = 0; i < NUM_PROFILES; i++) {
            if (column >= profiles[i].startCol && column <= profiles[i].endCol) {
                activeProfile = &profiles[i];
                currentAngle = profiles[i].servoAngle;
                currentSpeed = baseSpeed * profiles[i].speedMultiplier;
                servo->write(currentAngle);
                return true;
            }
        }
        return false;
    }

    void execute() {
        int pwm = map((int)currentSpeed, 0, 100, 0, 255);
        analogWrite(ENA, pwm);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    
    void stop() {
        currentSpeed = 0;
        analogWrite(ENA, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }

    float getSpeed() { return currentSpeed; }
    int getAngle() { return currentAngle; }
    const char* getProfileName() {
        return activeProfile ? activeProfile->description : "None";
    }
};

// ============ Logger Class ============
class Logger {
private:
    bool verbose;
    bool telemetry;
    unsigned long lastTelemetryTime = 0;
    const unsigned long telemetryInterval = 100;

public:
    void init(bool v, bool t) {
        verbose = v;
        telemetry = t;
    }

    void logState(RobotState from, RobotState to) {
        if (!verbose) return;
        Serial.print("[STATE] ");
        Serial.print(getStateName(from));
        Serial.print(" -> ");
        Serial.println(getStateName(to));
    }

    void logDetection(DetectionResult& result) {
        if (!verbose) return;
        Serial.print("[DETECT] Columns: ");
        for (int i = 0; i < result.activeCount; i++) {
            Serial.print(result.activeColumns[i]);
            Serial.print(" ");
        }
        Serial.print("| Primary: ");
        Serial.print(result.primaryColumn);
        Serial.print(" | Strength: ");
        Serial.println(result.maxSignalStrength);
    }

    void logMovement(int col, int angle, float speed, const char* profile) {
        if (!verbose) return;
        Serial.print("[MOVE] Col:");
        Serial.print(col);
        Serial.print(" Angle:");
        Serial.print(angle);
        Serial.print(" Speed:");
        Serial.print(speed);
        Serial.print(" Profile:");
        Serial.println(profile);
    }

    void logCalibration(int baseline[]) {
        Serial.print("[CALIB] Baseline: ");
        for (int i = 0; i < 10; i++) {
            Serial.print(baseline[i]);
            Serial.print(" ");
        }
        Serial.println();
    }

    void logTelemetry(int columns[], int baseline[], float speed, int angle) {
        if (!telemetry) return;
        if (millis() - lastTelemetryTime < telemetryInterval) return;
        lastTelemetryTime = millis();

        Serial.print("[TELEM] Sensors: ");
        for (int i = 0; i < 10; i++) {
            Serial.printf("%4d", columns[i]);
        }
        Serial.print(" | Speed:");
        Serial.print(speed, 1);
        Serial.print(" | Angle:");
        Serial.println(angle);
    }

private:
    const char* getStateName(RobotState state) {
        switch(state) {
            case IDLE: return "IDLE";
            case FOLLOWING: return "FOLLOWING";
            case BRAKING: return "BRAKING";
            case STOPPED: return "STOPPED";
            default: return "UNKNOWN";
        }
    }
};

// ============ ADC Objects ============
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;

// ============ Global Objects ============
Servo myServo;
BrakeManager brake;
MovementController movement;
Logger logger;
LaserDebouncer debouncer;  // 🆕 اضافه شد

// ============ Global Variables ============
int columns[10];
int baseline[10];
int leds[10] = {15, 2, 14, 4, 16, 17, 13, 12, 32, 33};

float speedPercent = baseSpeedPercent;
bool lastBtn = HIGH;
unsigned long lastBtnTime = 0;
unsigned long lastChangeTime = 0;
bool showingSpeed = false;

RobotState currentState = IDLE;
RobotState previousState = IDLE;
unsigned long stateStartTime = 0;

// ============ State Management ============
void updateState(RobotState newState) {
    if (currentState != newState) {
        previousState = currentState;
        currentState = newState;
        stateStartTime = millis();
        logger.logState(previousState, currentState);
    }
}

// ============ Basic Motor Functions ============
void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
}

void moveBack() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
}

// ============ LED Functions ============
void updateLedsSpeed() {
    int n = (speedPercent - baseSpeedPercent) / speedStepPercent;
    if (speedPercent == baseSpeedPercent) n = 0;
    for (int i = 0; i < 10; i++) {
        if (i < 2 || (i < n + 2)) digitalWrite(leds[i], HIGH);
        else digitalWrite(leds[i], LOW);
    }
}

void turnOffLeds() {
    for (int i = 0; i < 10; i++) digitalWrite(leds[i], LOW);
}

void updateLedsSensors(DetectionResult& result) {
    for (int i = 0; i < 10; i++) {
        bool active = false;
        for (int j = 0; j < result.activeCount; j++) {
            if (result.activeColumns[j] == i) {
                active = true;
                break;
            }
        }
        digitalWrite(leds[9 - i], active ? HIGH : LOW);
    }
}

// ============ Sensor Functions ============
void readSensors() {
    for (int i = 0; i < 4; i++)
        columns[i] = constrain(map(ads1.readADC_SingleEnded(i), 0, 32767, 0, 1024), 0, 1024);
    for (int i = 0; i < 4; i++)
        columns[i + 4] = constrain(map(ads2.readADC_SingleEnded(i), 0, 32767, 0, 1024), 0, 1024);
    for (int i = 0; i < 2; i++)
        columns[i + 8] = constrain(map(ads3.readADC_SingleEnded(i), 0, 32767, 0, 1024), 0, 1024);
}

// ============ Detection Function ============
DetectionResult detectLaser() {
    DetectionResult result;
    result.detected = false;
    result.primaryColumn = -1;
    result.activeCount = 0;
    result.maxSignalStrength = 0;

    int maxStrength = 0;
    int maxColumn = -1;

    // تشخیص همه ستون‌های فعال
    for (int i = 0; i < 10; i++) {
        int dynamicThreshold = (int)(baseline[i] * sensorThresholdRatio);
        int signalStrength = columns[i] - baseline[i];

        if (signalStrength > dynamicThreshold) {
            result.activeColumns[result.activeCount++] = i;
            result.detected = true;

            // پیدا کردن قوی‌ترین سیگنال
            if (signalStrength > maxStrength) {
                maxStrength = signalStrength;
                maxColumn = i;
            }
        }
    }

    if (result.detected) {
        result.primaryColumn = maxColumn;
        result.maxSignalStrength = maxStrength;
    }

    return result;
}

// ============ Calibration Function ============
void calibrateWithProgress() {
    const int totalTime = 2000;
    const int samples = 40;
    int sum[10] = {0};

    Serial.println("[CALIB] Starting calibration...");

    for (int s = 0; s < samples; s++) {
        readSensors();
        for (int i = 0; i < 10; i++) sum[i] += columns[i];

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

    logger.logCalibration(baseline);
}

// ============ 🆕 Main Control Function با Debouncing ============
void controlRobot() {
    DetectionResult detection = detectLaser();
    
    // 🆕 اعمال debouncing روی تشخیص
    bool stableDetection = debouncer.getStableDetection(detection.detected);
    
    // 🔍 لاگ برای دیباگ با اطلاعات debouncing
    if (verboseSerial) {
        Serial.print("State: ");
        Serial.print(currentState);
        Serial.print(" | Raw:");
        Serial.print(detection.detected ? "YES" : "NO");
        Serial.print(" | Signal:");
        Serial.print(detection.maxSignalStrength);
        debouncer.printDebugInfo();  // 🆕 نمایش وضعیت debouncer
        Serial.print(" | StableDetect:");
        Serial.println(stableDetection ? "YES" : "NO");
    }

    // Log detection if verbose and stable
    if (stableDetection && verboseSerial) {
        logger.logDetection(detection);
    }

    switch(currentState) {
        case IDLE:
        case STOPPED:
            // 🆕 استفاده از stableDetection به جای detection.detected
            if (stableDetection) {
                updateState(FOLLOWING);
                if (movement.applyProfile(detection.primaryColumn)) {
                    movement.execute();
                    logger.logMovement(detection.primaryColumn,
                                     movement.getAngle(),
                                     movement.getSpeed(),
                                     movement.getProfileName());
                }
            }
            break;

        case FOLLOWING:
            // 🆕 استفاده از stableDetection به جای detection.detected
            if (!stableDetection) {
                updateState(BRAKING);
                myServo.write(servoCenter);
                brake.start();
                movement.setBaseSpeed(speedPercent);
                movement.execute();
            } else {
                // اگر همچنان لیزر دیده می‌شود، موقعیت را به‌روزرسانی کن
                if (detection.detected) {  // برای به‌روزرسانی موقعیت از detection واقعی استفاده می‌کنیم
                    if (movement.applyProfile(detection.primaryColumn)) {
                        movement.execute();
                        if (verboseSerial) {
                            logger.logMovement(detection.primaryColumn,
                                             movement.getAngle(),
                                             movement.getSpeed(),
                                             movement.getProfileName());
                        }
                    }
                }
            }
            break;

        case BRAKING:
            // صبر می‌کنیم تا لیزر برگرده
            if (stableDetection) {
                brake.reset();  // آماده برای ترمز بعدی
                updateState(FOLLOWING);
                if (movement.applyProfile(detection.primaryColumn)) {
                    movement.execute();
                }
            }
            break;
    }

    // Update LEDs based on detection
    if (!showingSpeed) {
        updateLedsSensors(detection);
    }

    // Telemetry logging
    logger.logTelemetry(columns, baseline, movement.getSpeed(), movement.getAngle());
}

// ============ Button Handling Function ============
void handleButton() {
    bool currentBtn = digitalRead(BTN);
    
    if (currentBtn == LOW && lastBtn == HIGH) {
        if (millis() - lastBtnTime > buttonDebounceMs) {
            speedPercent += speedStepPercent;
            if (speedPercent > 100) speedPercent = baseSpeedPercent;
            
            movement.setBaseSpeed(speedPercent);
            
            Serial.print("[SPEED] Changed to: ");
            Serial.print(speedPercent);
            Serial.println("%");
            
            updateLedsSpeed();
            showingSpeed = true;
            lastChangeTime = millis();
            lastBtnTime = millis();
        }
    }
    
    lastBtn = currentBtn;
    
    if (showingSpeed && (millis() - lastChangeTime > ledSpeedShowTime)) {
        showingSpeed = false;
        turnOffLeds();
    }
}

// ============ Setup Function ============
void setup() {
    Serial.begin(115200);
    Serial.println("\n========= Opal Robot Starting =========");
    Serial.println("Version: OOP with Debouncing");
    Serial.println("=====================================");

    Wire.begin(21, 22);

    // Initialize pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(BTN, INPUT_PULLUP);

    for (int i = 0; i < 10; i++) {
        pinMode(leds[i], OUTPUT);
        digitalWrite(leds[i], LOW);
    }

    // Initialize servo
    myServo.attach(servoPin, 500, 2400);
    myServo.write(servoCenter);

    // Initialize movement controller
    movement.init(&myServo, speedPercent);

    // Initialize logger
    logger.init(verboseSerial, telemetryEnabled);

    // Initialize ADCs
    Serial.println("[INIT] Initializing ADCs...");
    ads1.begin(0x48);
    ads2.begin(0x49);
    ads3.begin(0x4A);
    ads1.setGain(GAIN_ONE);
    ads2.setGain(GAIN_ONE);
    ads3.setGain(GAIN_ONE);

    pinMode(RDY1, INPUT);
    pinMode(RDY2, INPUT);
    pinMode(RDY3, INPUT);

    // Calibration
    delay(1000);
    calibrateWithProgress();

    // Reset debouncer after calibration
    debouncer.reset();
    
    Serial.println("[INIT] System ready!");
    Serial.println("=====================================");
    Serial.print("Threshold Ratio: ");
    Serial.println(sensorThresholdRatio);
    Serial.print("Base Speed: ");
    Serial.print(baseSpeedPercent);
    Serial.println("%");
    Serial.println("Debouncing: ENABLED");
    Serial.print("Lost Threshold: ");
    Serial.println(5);
    Serial.print("Found Threshold: ");
    Serial.println(3);
    Serial.println("=====================================\n");

    updateState(IDLE);
}

// ============ Main Loop Function ============
void loop() {
    // Read sensors
    readSensors();
    
    // Handle button press
    handleButton();
    
    // Main robot control with debouncing
    controlRobot();
    
    // Small delay for stability
    delay(10);
}
