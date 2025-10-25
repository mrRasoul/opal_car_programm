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
int servoRight              = 70;

float sensorThresholdRatio  = 0.2;

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

// ============ Brake Manager Class ============
class BrakeManager {
private:
    bool isActive = false;
    unsigned long startTime = 0;
    const unsigned long duration = 150;
    
public:
    void start() {
        if (!isActive) {
            isActive = true;
            startTime = millis();
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
    }
    
    bool update() {
        if (!isActive) return false;
        
        if (millis() - startTime >= duration) {
            isActive = false;
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            return true;
        }
        return false;
    }
    
    bool isBraking() { return isActive; }
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
        for (int i = 0; i < NUM_PROFILES; i++) {
            if (column >= profiles[i].startCol && column <= profiles[i].endCol) {
                activeProfile = &profiles[i];
                currentAngle = profiles[i].servoAngle;
                currentSpeed = baseSpeed * profiles[i].speedMultiplier;
                return true;
            }
        }
        return false;
    }
    
    void execute() {
        servo->write(currentAngle);
        int pwm = map((int)currentSpeed, 0, 100, 0, 255);
        analogWrite(ENA, constrain(pwm, 0, 255));
    }
    
    void stop() {
        currentSpeed = 0;
        currentAngle = servoCenter;
        execute();
    }
    
    int getAngle() { return currentAngle; }
    float getSpeed() { return currentSpeed; }
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
    
    void logMovement(int col, int angle, float speed, const char* profile) {
        if (!verbose) return;
        Serial.printf("[MOVE] Col:%d | Angle:%d° | Speed:%.1f%% | Profile:%s\n", 
                     col, angle, speed, profile);
    }
    
    void logState(RobotState oldState, RobotState newState) {
        if (!verbose) return;
        const char* stateNames[] = {"IDLE", "FOLLOWING", "BRAKING", "STOPPED"};
        Serial.printf("[STATE] %s -> %s\n", stateNames[oldState], stateNames[newState]);
    }
    
    void logDetection(DetectionResult& result) {
        if (!verbose) return;
        if (result.detected) {
            Serial.printf("[DETECT] Primary:%d | Active:", result.primaryColumn);
            for (int i = 0; i < result.activeCount; i++) {
                Serial.printf("%d ", result.activeColumns[i]);
            }
            Serial.printf("| Strength:%d\n", result.maxSignalStrength);
        }
    }
    
    void logCalibration(int* baseline) {
        if (!verbose) return;
        Serial.print("[CALIB] Baseline: ");
        for (int i = 0; i < 10; i++) {
            Serial.printf("%4d ", baseline[i]);
        }
        Serial.println();
    }
    
    void logTelemetry(int* columns, int* baseline, float speed, int angle) {
        if (!telemetry) return;
        unsigned long now = millis();
        if (now - lastTelemetryTime < telemetryInterval) return;
        lastTelemetryTime = now;
        
        Serial.print("[TELEM] Sensors:");
        for (int i = 0; i < 10; i++) {
            Serial.printf("%4d", columns[i]);
        }
        Serial.printf(" | Speed:%.1f | Angle:%d\n", speed, angle);
    }
};

// ============ Global Objects ============
Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_ADS1115 ads3;

Servo myServo;
BrakeManager brake;
MovementController movement;
Logger logger;

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

// ============ Main Control Function ============
void controlRobot() {
    DetectionResult detection = detectLaser();
     // 🔍 لاگ برای دیباگ
    Serial.print("State: ");
    Serial.print(currentState);
    Serial.print(" | Detected: ");
    Serial.print(detection.detected);
    Serial.print(" | Signal: ");
    Serial.println(detection.maxSignalStrength);
    
    // Log detection if verbose
    if (detection.detected && verboseSerial) {
        logger.logDetection(detection);
    }
    
    switch(currentState) {
        case IDLE:
        case STOPPED:
            if (detection.detected) {
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
            if (!detection.detected) {
                updateState(BRAKING);
                myServo.write(servoCenter);
                brake.start();
                movement.setBaseSpeed(speedPercent);
                movement.execute();
            } else {
                if (movement.applyProfile(detection.primaryColumn)) {
                    movement.execute();
                    logger.logMovement(detection.primaryColumn, 
                                     movement.getAngle(), 
                                     movement.getSpeed(),
                                     movement.getProfileName());
                }
            }
            break;
            
        case BRAKING:
            if (brake.update()) {
                updateState(STOPPED);
                movement.stop();
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

// ============ Setup Function ============
void setup() {
    Serial.begin(115200);
    Serial.println("\n========= Opal Robot Starting =========");
    
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
    ads1.begin(0x48);
    ads2.begin(0x49);
    ads3.begin(0x4A);
    ads1.setGain(GAIN_ONE);
    ads2.setGain(GAIN_ONE);
    ads3.setGain(GAIN_ONE);
    
    pinMode(RDY1, INPUT);
    pinMode(RDY2, INPUT);
    pinMode(RDY3, INPUT);
    
    // Motor test
    Serial.println("[INIT] Testing motors...");
    moveForward();
    analogWrite(ENA, 100);
    delay(200);
    moveBack();
    delay(200);
    analogWrite(ENA, 0);
    moveForward();
      // Calibration
    calibrateWithProgress();
    updateState(IDLE);

    Serial.println("[READY] Servo centered, waiting for sensors...");
}

// ============ Loop Function ============
void loop() {
    bool btn = digitalRead(BTN);
    unsigned long now = millis();

    // Speed button handling
    if (lastBtn == HIGH && btn == LOW && (now - lastBtnTime) > buttonDebounceMs) {
        speedPercent += speedStepPercent;
        if (speedPercent > 100) speedPercent = baseSpeedPercent;

        movement.setBaseSpeed(speedPercent);

        if (verboseSerial) {
            Serial.print("[BUTTON] New speed: ");
            Serial.println(speedPercent);
        }

        updateLedsSpeed();
        showingSpeed = true;
        lastChangeTime = now;
        lastBtnTime = now;
    }

    // Turn off speed LEDs after display time
    if (showingSpeed && (now - lastChangeTime >= ledSpeedShowTime)) {
        showingSpeed = false;
        turnOffLeds();
    }

    // Read sensors and run control logic
    if (!showingSpeed) {
        readSensors();
        controlRobot();
    }

    lastBtn = btn;
}
  
