// File: Opal_Robot_FastReaction.ino

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

float sensorThresholdRatio  = 0.5;   // نسبت آستانه نسبت به baseline
const int minAbsThreshold   = 20;    // کف مطلق ساده برای مقیاس 0..1024

unsigned long ledSpeedShowTime = 2000;
unsigned long buttonStableMs   = 700; // برای GPIO34 بدون پول‌آپ
bool verboseSerial = false;           // بار سریال را کم می‌کنیم
bool telemetryEnabled = false;        // برای حداکثر سرعت خاموش

// Brake options (L298N)
bool electricBrakeEnabled     = true;    // ترمز الکتریکی سریع‌تر
unsigned long electricBrakeMs = 140;     // مدت ترمز الکتریکی
bool centerOnBrake            = false;   // سروو را موقع ترمز سنتر نکن

// Loop pacing
const unsigned long loopSleepMs = 1;     // کاهش تاخیر حلقه

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
    {0, 2, 130, 0.7, "Sharp Left"},
    {3, 6, 100, 1.0, "Straight"},
    {7, 9, 65,  0.7, "Sharp Right"}
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

// ============ Debouncer (time-based) ============
class LaserDebouncerMs {
private:
    bool stableState = false;
    bool pendingState = false;
    unsigned long pendingStart = 0;
    unsigned long foundMs;
    unsigned long lostMs;

public:
    LaserDebouncerMs(unsigned long found=40, unsigned long lost=60)
        : foundMs(found), lostMs(lost) {}

    bool getStableDetection(bool currentDetection) {
        // تغییر وضعیت خام → شروع پنجره جدید
        if (currentDetection != pendingState) {
            pendingState = currentDetection;
            pendingStart = millis();
        } else {
            // آیا پنجره پایداری تکمیل شده؟
            if (stableState != pendingState) {
                unsigned long hold = millis() - pendingStart;
                unsigned long need = pendingState ? foundMs : lostMs;
                if (hold >= need) {
                    stableState = pendingState;
                }
            }
        }
        return stableState;
    }

    void reset() {
        stableState = false;
        pendingState = false;
        pendingStart = 0;
    }

    void printDebugInfo() {
        Serial.print(" | Stable:");
        Serial.print(stableState ? "YES" : "NO");
        Serial.print(" | Debounce(ms) F:");
        Serial.print(foundMs);
        Serial.print(" L:");
        Serial.print(lostMs);
    }

    unsigned long getFoundMs() const { return foundMs; }
    unsigned long getLostMs() const { return lostMs; }
};

// ============ Brake Manager for L298N (Non-blocking) ============
class BrakeManager {
private:
    bool brakingActive = false;
    unsigned long brakeEndTime = 0;

public:
    void startCoast() {
        analogWrite(ENA, 0);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        brakingActive = false;
        if (verboseSerial) Serial.println("[BRAKE] Coast stop");
    }

    void startElectricBrake(unsigned long durationMs) {
        // ترمز دینامیکی به زمین
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 255);
        brakingActive = true;
        brakeEndTime = millis() + durationMs;
        if (verboseSerial) Serial.printf("[BRAKE] Electric brake for %lu ms\n", durationMs);
    }

    void update() {
        if (brakingActive && millis() >= brakeEndTime) {
            analogWrite(ENA, 0);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            brakingActive = false;
            if (verboseSerial) Serial.println("[BRAKE] Electric brake ended -> Coast");
        }
    }

    bool isBraking() const { return brakingActive; }
};

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
LaserDebouncerMs debouncer(40, 60);  // FOUND=40ms, LOST=60ms

// ============ Global Variables ============
int columns[10];
int baseline[10];
int leds[10] = {15, 2, 14, 4, 16, 17, 13, 12, 32, 33};

float speedPercent = baseSpeedPercent;
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
    // نگاشت 16بیتی ADS به 0..1024 برای سازگاری
    for (int i = 0; i < 4; i++)
        columns[i] = constrain(map(ads1.readADC_SingleEnded(i), 0, 32767, 0, 1024), 0, 1024);
    for (int i = 0; i < 4; i++)
        columns[i + 4] = constrain(map(ads2.readADC_SingleEnded(i), 0, 32767, 0, 1024), 0, 1024);
    for (int i = 0; i < 2; i++)
        columns[i + 8] = constrain(map(ads3.readADC_SingleEnded(i), 0, 32767, 0, 1024), 0, 1024);
}

// ============ Detection Function (with centroid and dual threshold) ============
DetectionResult detectLaser() {
    DetectionResult result;
    result.detected = false;
    result.primaryColumn = -1;
    result.activeCount = 0;
    result.maxSignalStrength = 0;

    int sumStrength = 0;
    int sumWeightedCol = 0;

    for (int i = 0; i < 10; i++) {
        int dynamicThreshold = (int)(baseline[i] * sensorThresholdRatio);
        int threshold = max(minAbsThreshold, dynamicThreshold);
        int signalStrength = columns[i] - baseline[i];

        if (signalStrength > threshold) {
            result.activeColumns[result.activeCount++] = i;
            result.detected = true;

            sumStrength += signalStrength;
            sumWeightedCol += signalStrength * i;

            if (signalStrength > result.maxSignalStrength) {
                result.maxSignalStrength = signalStrength;
                result.primaryColumn = i; // پیش‌فرض: max
            }
        }
    }

    if (result.detected && sumStrength > 0) {
        int centroid = (sumWeightedCol + sumStrength / 2) / sumStrength; // گرد کردن
        result.primaryColumn = centroid; // مرکز جرم برای نرمی حرکت
    }

    return result;
}

// ============ Adaptive Baseline (compensate ambient drift) ============
void adaptiveBaselineUpdate(bool stableDetection) {
    // فقط وقتی لیزر پایدار نیست، آستانه محیط را آهسته به‌روزرسانی کن
    if (!stableDetection) {
        const float beta = 0.01f; // IIR آرام
        for (int i = 0; i < 10; i++) {
            baseline[i] = baseline[i] + (int)(beta * (columns[i] - baseline[i]));
        }
    }
}

// ============ Calibration Function ============
void calibrateWithProgress() {
    const int totalTimeMs = 2000;
    const int samples = 40;
    const int intervalMs = totalTimeMs / samples;

    int sum[10] = {0};

    Serial.println("[CALIB] Starting calibration...");

    // نمایش پیشرفت با LED و جمع‌آوری نمونه‌ها
    for (int s = 0; s < samples; s++) {
        readSensors();
        for (int i = 0; i < 10; i++) {
            sum[i] += columns[i];
        }

        // نمایش پیشرفت ساده روی LEDها
        int lit = map(s + 1, 0, samples, 0, 10);
        for (int i = 0; i < 10; i++) {
            digitalWrite(leds[i], (i < lit) ? HIGH : LOW);
        }

        delay(intervalMs);
    }

    // محاسبه baseline
    for (int i = 0; i < 10; i++) {
        baseline[i] = sum[i] / samples;
    }

    // خاموش کردن LEDها و گزارش
    turnOffLeds();
    Serial.println("[CALIB] Calibration complete.");
    Logger().logCalibration(baseline); // فقط چاپ؛ Logger محلی ایجاد می‌شود که صرفاً چاپ می‌کند
}

// ============ Main Control Function ============
void controlRobot(DetectionResult& detection, bool stableDetection) {
    if (verboseSerial) {
        Serial.print("State:");
        Serial.print(currentState);
        Serial.print(" | Raw:");
        Serial.print(detection.detected ? "YES" : "NO");
        Serial.print(" | Signal:");
        Serial.print(detection.maxSignalStrength);
        debouncer.printDebugInfo();
        Serial.print(" | StableDetect:");
        Serial.println(stableDetection ? "YES" : "NO");
    }

    if (stableDetection && verboseSerial) {
        logger.logDetection(detection);
    }

    switch(currentState) {
        case IDLE:
        case STOPPED:
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
            if (!stableDetection) {
                updateState(BRAKING);
                if (centerOnBrake) myServo.write(servoCenter);
                movement.stop();

                if (electricBrakeEnabled) {
                    brake.startElectricBrake(electricBrakeMs);
                } else {
                    brake.startCoast();
                }
            } else {
                if (detection.detected) {
                    if (movement.applyProfile(detection.primaryColumn)) {
                        movement.execute();
                        logger.logMovement(detection.primaryColumn,
                                           movement.getAngle(),
                                           movement.getSpeed(),
                                           movement.getProfileName());
                    }
                }
            }
            break;

        case BRAKING:
            if (stableDetection) {
                updateState(FOLLOWING);
                if (movement.applyProfile(detection.primaryColumn)) {
                    movement.execute();
                }
            }
            break;
    }

    if (!showingSpeed) {
        updateLedsSensors(detection);
    }

    logger.logTelemetry(columns, baseline, movement.getSpeed(), movement.getAngle());
}

// ============ Button Handling (stable latch for GPIO34) ============
void handleButtonStable() {
    static bool stableLevel = false;
    static unsigned long levelStart = 0;

    bool level = (digitalRead(BTN) == LOW);

    if (level != stableLevel) {
        if (levelStart == 0) {
            levelStart = millis();
        } else {
            unsigned long holdMs = millis() - levelStart;
            if (holdMs >= buttonStableMs) {
                stableLevel = level;
                levelStart = 0;

                if (stableLevel) {
                    speedPercent += speedStepPercent;
                    if (speedPercent > 100) speedPercent = baseSpeedPercent;
                    movement.setBaseSpeed(speedPercent);
                    Serial.printf("[SPEED] Changed to: %.1f%%\n", speedPercent);

                    updateLedsSpeed();
                    showingSpeed = true;
                    stateStartTime = millis();
                }
            }
        }
    } else {
        levelStart = 0;
    }

    if (showingSpeed && (millis() - stateStartTime > ledSpeedShowTime)) {
        showingSpeed = false;
        turnOffLeds();
    }
}

// ============ Setup Function ============
void setup() {
    Serial.begin(115200);
    Serial.println("\n========= Opal Robot Starting =========");
    Serial.println("Version: Fast Reaction (Unified)");
    Serial.println("=====================================");

    Wire.begin(21, 22);
    Wire.setClock(400000); // I2C سریع‌تر

    // Initialize pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(BTN, INPUT); // GPIO34 پول‌آپ داخلی ندارد

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

    // اگر کتابخانه شما این توابع/ثابت‌ها را ندارد، کامنت کنید:
    ads1.setDataRate(RATE_ADS1115_860SPS);
    ads2.setDataRate(RATE_ADS1115_860SPS);
    ads3.setDataRate(RATE_ADS1115_860SPS);

    pinMode(RDY1, INPUT);
    pinMode(RDY2, INPUT);
    pinMode(RDY3, INPUT);

    // Calibration
    delay(2000);
    calibrateWithProgress();

    // Reset debouncer after calibration
    debouncer.reset();

    Serial.println("[INIT] System ready!");
    Serial.println("=====================================");
    Serial.print("Threshold Ratio: ");
    Serial.println(sensorThresholdRatio);
    Serial.print("Min Abs Threshold: ");
    Serial.println(minAbsThreshold);
    Serial.print("Debounce (ms) Found/Lost: ");
    Serial.print(debouncer.getFoundMs()); Serial.print("/"); Serial.println(debouncer.getLostMs());
    Serial.print("Base Speed: ");
    Serial.print(baseSpeedPercent);
    Serial.println("%");
    Serial.print("Brake Mode: ");
    Serial.println(electricBrakeEnabled ? "Electric (non-blocking)" : "Coast");
    Serial.print("Brake Duration (ms): ");
    Serial.println(electricBrakeMs);
    Serial.println("=====================================\n");

    updateState(IDLE);
}

// ============ Main Loop Function ============
void loop() {
    readSensors();
    handleButtonStable();

    DetectionResult detection = detectLaser();
    bool stableDetection = debouncer.getStableDetection(detection.detected);

    adaptiveBaselineUpdate(stableDetection);
    controlRobot(detection, stableDetection);

    brake.update();

    if (loopSleepMs) delay(loopSleepMs);
}
