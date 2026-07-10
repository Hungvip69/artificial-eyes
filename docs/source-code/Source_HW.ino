#include <ESP32Servo.h>
#include "BluetoothSerial.h"

// =======================================================
//  Artificial Eyes - ESP32 hardware
//  Commands from MIT App Inventor:
//    HEIGHT|167   -> update user height in cm
//    ON           -> enable walking scan
//    OFF          -> disable walking scan
//    GET_CONFIG   -> print/send current thresholds
//
//  Alert codes sent to the app:
//    2 = low obstacle
//    3 = hole / open space below
//    4 = high obstacle
//    5 = front obstacle / wall
// =======================================================

BluetoothSerial SerialBT;

// ----------------------- Bluetooth ----------------------
const char* BT_DEVICE_NAME = "ArtificialEye_Hardware";

bool bluetoothConnected = false;
unsigned long lastBTCheckTime = 0;
const unsigned long BT_CHECK_INTERVAL_MS = 1000;

String btCommandBuffer;
unsigned long lastBTByteTime = 0;
const unsigned long BT_COMMAND_FLUSH_MS = 120;  // Allows commands without \n, e.g. "OFF".

// ----------------------- User config --------------------
const float DEFAULT_USER_HEIGHT_CM = 170.0;
const float MIN_USER_HEIGHT_CM = 100.0;
const float MAX_USER_HEIGHT_CM = 220.0;

const float SENSOR_HEIGHT_RATIO = 0.60;  // Device position is about 60% of user height.
const float HOLE_EXTRA_CM = 25.0;

float userHeightCm = DEFAULT_USER_HEIGHT_CM;
float sensorHeightCm = DEFAULT_USER_HEIGHT_CM * SENSOR_HEIGHT_RATIO;

// Dynamic thresholds. They are updated by updateThresholdsByHeight().
float WARNING_LOW = 40.0;     // Low obstacle / ground-level obstacle.
float HOLE_DISTANCE = 120.0;  // Hole / open space below.
float WALL_DISTANCE = 70.0;   // Front obstacle / wall.
float WARNING_HIGH = 60.0;    // High obstacle.

// ----------------------- Servo config -------------------
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const int SERVO_PIN_1 = 13;
const int SERVO_PIN_2 = 12;  // Note: GPIO12 is an ESP32 strapping pin; avoid external pull-up/down here.
const int SERVO_PIN_3 = 14;
const int SERVO_PIN_4 = 27;

const int SERVO_12_HOME = 50;
const int SERVO_34_HOME = 130;

const int SERVO_SMOOTH_STEPS = 80;
const int SERVO_SMOOTH_DELAY_MS = 12;
const int SERVO_PAIR_DELAY_MS = 3;

bool servoScanEnabled = false;  // Safer: wait for ON command instead of scanning immediately after boot.
int currentServo12Angle = SERVO_12_HOME;
int currentServo34Angle = SERVO_34_HOME;

struct ScanPoint {
  int servo12Angle;
  int servo34Angle;
  int level;
};

const ScanPoint SCAN_POINTS[] = {
  {65, 120, 1},
  {90, 99, 2},
  {100, 90, 3},
};
const int SCAN_POINT_COUNT = sizeof(SCAN_POINTS) / sizeof(SCAN_POINTS[0]);

// ----------------------- Ultrasonic config --------------
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
};

const UltrasonicSensor SENSORS[] = {
  {26, 25},
  {33, 32},
  {5, 18},
  {19, 21},
};
const int SENSOR_COUNT = sizeof(SENSORS) / sizeof(SENSORS[0]);

const unsigned long PULSE_TIMEOUT_US = 25000;  // About 4.3m max distance.
const float SOUND_SPEED_CM_PER_US = 0.0343;
const int SENSOR_CROSSTALK_DELAY_MS = 25;

// ----------------------- Function declarations ----------
void setupBluetooth();
void setupServos();
void setupUltrasonicSensors();

void checkBluetoothConnection();
void readBluetoothCommands();
void processBluetoothCommand(String command);
void sendBluetoothLine(const String& text);

void updateThresholdsByHeight(float heightCm);
void printThresholdConfig();

void runScanCycle();
void waitAndService(unsigned long waitMs);
void setServoHome();
void writeServoPairs(int servo12Angle, int servo34Angle);
void moveServoSmooth(int start12, int end12, int start34, int end34);

float getDistanceCm(const UltrasonicSensor& sensor);
void readAllDistances(float distances[], int count);
void scanAtPosition(const ScanPoint& point);
void checkWarningAndSend(int scanLevel, const float distances[], int count);
bool isValidDistance(float distanceCm);
bool hasAnyDistanceBelow(const float distances[], int count, float thresholdCm);
bool hasAnyDistanceAbove(const float distances[], int count, float thresholdCm);

void setup() {
  Serial.begin(115200);

  btCommandBuffer.reserve(64);

  setupBluetooth();
  updateThresholdsByHeight(userHeightCm);
  setupServos();
  setupUltrasonicSensors();

  delay(1000);

  Serial.println("=== ARTIFICIAL EYES HARDWARE READY ===");
  Serial.println("Commands: HEIGHT|167, ON, OFF, GET_CONFIG");
}

void loop() {
  checkBluetoothConnection();
  readBluetoothCommands();

  if (servoScanEnabled) {
    runScanCycle();
  } else {
    waitAndService(50);
  }
}

void setupBluetooth() {
  SerialBT.begin(BT_DEVICE_NAME);
  Serial.print("Bluetooth started: ");
  Serial.println(BT_DEVICE_NAME);
}

void setupServos() {
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);

  servo1.attach(SERVO_PIN_1, 500, 2500);
  servo2.attach(SERVO_PIN_2, 500, 2500);
  servo3.attach(SERVO_PIN_3, 500, 2500);
  servo4.attach(SERVO_PIN_4, 500, 2500);

  setServoHome();
}

void setupUltrasonicSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(SENSORS[i].trigPin, OUTPUT);
    pinMode(SENSORS[i].echoPin, INPUT);
    digitalWrite(SENSORS[i].trigPin, LOW);
  }
}

void checkBluetoothConnection() {
  if (millis() - lastBTCheckTime < BT_CHECK_INTERVAL_MS) return;

  lastBTCheckTime = millis();
  bool currentState = SerialBT.hasClient();

  if (currentState == bluetoothConnected) return;

  bluetoothConnected = currentState;

  if (bluetoothConnected) {
    Serial.println("Bluetooth device connected");
    sendBluetoothLine("ArtificialEye_Hardware_CONNECTED");
    printThresholdConfig();
  } else {
    Serial.println("Bluetooth device disconnected");
  }
}

void readBluetoothCommands() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    lastBTByteTime = millis();

    if (c == '\n' || c == '\r') {
      processBluetoothCommand(btCommandBuffer);
      btCommandBuffer = "";
    } else {
      btCommandBuffer += c;

      if (btCommandBuffer.length() > 63) {
        Serial.println("Bluetooth command too long, clearing buffer");
        btCommandBuffer = "";
      }
    }
  }

  // App Inventor may send simple commands such as "OFF" without a newline.
  if (btCommandBuffer.length() > 0 && millis() - lastBTByteTime > BT_COMMAND_FLUSH_MS) {
    processBluetoothCommand(btCommandBuffer);
    btCommandBuffer = "";
  }
}

void processBluetoothCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  String upperCommand = command;
  upperCommand.toUpperCase();

  Serial.print("Bluetooth command: ");
  Serial.println(command);

  if (upperCommand.startsWith("HEIGHT|")) {
    String heightText = command.substring(7);
    heightText.trim();
    updateThresholdsByHeight(heightText.toFloat());
    return;
  }

  if (upperCommand == "ON") {
    servoScanEnabled = true;
    Serial.println("Servo scan ON");
    sendBluetoothLine("SERVO_SCAN_ON");
    return;
  }

  if (upperCommand == "OFF") {
    servoScanEnabled = false;
    Serial.println("Servo scan OFF");
    sendBluetoothLine("SERVO_SCAN_OFF");
    setServoHome();
    return;
  }

  if (upperCommand == "GET_CONFIG") {
    printThresholdConfig();
    return;
  }

  Serial.print("Unknown Bluetooth command: ");
  Serial.println(command);
}

void sendBluetoothLine(const String& text) {
  if (SerialBT.hasClient()) {
    SerialBT.println(text);
  }
}

void updateThresholdsByHeight(float heightCm) {
  if (heightCm < MIN_USER_HEIGHT_CM || heightCm > MAX_USER_HEIGHT_CM) {
    Serial.print("HEIGHT invalid, ignored: ");
    Serial.println(heightCm);
    sendBluetoothLine("HEIGHT_ERROR|INVALID");
    return;
  }

  userHeightCm = heightCm;
  sensorHeightCm = userHeightCm * SENSOR_HEIGHT_RATIO;

  // Example: user = 167cm -> sensor ≈ 100cm -> LOW ≈ 40, HOLE ≈ 125,
  // WALL ≈ 70, HIGH ≈ 60.
  WARNING_LOW = sensorHeightCm * 0.40;
  HOLE_DISTANCE = sensorHeightCm + HOLE_EXTRA_CM;
  WALL_DISTANCE = sensorHeightCm * 0.70;
  WARNING_HIGH = sensorHeightCm * 0.60;

  printThresholdConfig();
}

void printThresholdConfig() {
  Serial.print("HEIGHT_CONFIG | USER=");
  Serial.print(userHeightCm, 0);
  Serial.print(" SENSOR=");
  Serial.print(sensorHeightCm, 0);
  Serial.print(" LOW=");
  Serial.print(WARNING_LOW, 0);
  Serial.print(" HOLE=");
  Serial.print(HOLE_DISTANCE, 0);
  Serial.print(" WALL=");
  Serial.print(WALL_DISTANCE, 0);
  Serial.print(" HIGH=");
  Serial.println(WARNING_HIGH, 0);

  if (SerialBT.hasClient()) {
    SerialBT.print("HEIGHT_OK|USER=");
    SerialBT.print(userHeightCm, 0);
    SerialBT.print("|SENSOR=");
    SerialBT.print(sensorHeightCm, 0);
    SerialBT.print("|LOW=");
    SerialBT.print(WARNING_LOW, 0);
    SerialBT.print("|HOLE=");
    SerialBT.print(HOLE_DISTANCE, 0);
    SerialBT.print("|WALL=");
    SerialBT.print(WALL_DISTANCE, 0);
    SerialBT.print("|HIGH=");
    SerialBT.println(WARNING_HIGH, 0);
  }
}

void runScanCycle() {
  for (int i = 0; i < SCAN_POINT_COUNT; i++) {
    const ScanPoint& point = SCAN_POINTS[i];

    moveServoSmooth(currentServo12Angle, point.servo12Angle,
                    currentServo34Angle, point.servo34Angle);
    if (!servoScanEnabled) return;

    currentServo12Angle = point.servo12Angle;
    currentServo34Angle = point.servo34Angle;

    scanAtPosition(point);
    if (!servoScanEnabled) return;

    waitAndService(200);
    if (!servoScanEnabled) return;
  }

  moveServoSmooth(currentServo12Angle, SERVO_12_HOME,
                  currentServo34Angle, SERVO_34_HOME);
  currentServo12Angle = SERVO_12_HOME;
  currentServo34Angle = SERVO_34_HOME;

  waitAndService(300);
}

void waitAndService(unsigned long waitMs) {
  unsigned long startTime = millis();

  while (millis() - startTime < waitMs) {
    checkBluetoothConnection();
    readBluetoothCommands();

    if (!servoScanEnabled && waitMs > 50) return;
    delay(10);
  }
}

void setServoHome() {
  writeServoPairs(SERVO_12_HOME, SERVO_34_HOME);
  currentServo12Angle = SERVO_12_HOME;
  currentServo34Angle = SERVO_34_HOME;
}

void writeServoPairs(int servo12Angle, int servo34Angle) {
  servo1.write(servo12Angle);
  servo2.write(servo12Angle);

  delay(SERVO_PAIR_DELAY_MS);

  servo3.write(servo34Angle);
  servo4.write(servo34Angle);
}

void moveServoSmooth(int start12, int end12, int start34, int end34) {
  for (int i = 0; i <= SERVO_SMOOTH_STEPS; i++) {
    checkBluetoothConnection();
    readBluetoothCommands();

    if (!servoScanEnabled) {
      setServoHome();
      return;
    }

    float t = (float)i / SERVO_SMOOTH_STEPS;
    float smoothT = t * t * (3.0 - 2.0 * t);

    int angle12 = start12 + (end12 - start12) * smoothT;
    int angle34 = start34 + (end34 - start34) * smoothT;

    writeServoPairs(angle12, angle34);
    delay(SERVO_SMOOTH_DELAY_MS);
  }
}

float getDistanceCm(const UltrasonicSensor& sensor) {
  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(sensor.trigPin, LOW);

  unsigned long duration = pulseIn(sensor.echoPin, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) return -1.0;

  return duration * SOUND_SPEED_CM_PER_US / 2.0;
}

void readAllDistances(float distances[], int count) {
  for (int i = 0; i < count; i++) {
    distances[i] = getDistanceCm(SENSORS[i]);
    delay(SENSOR_CROSSTALK_DELAY_MS);
  }
}

void scanAtPosition(const ScanPoint& point) {
  checkBluetoothConnection();
  readBluetoothCommands();

  if (!servoScanEnabled) {
    setServoHome();
    return;
  }

  writeServoPairs(point.servo12Angle, point.servo34Angle);
  delay(400);

  checkBluetoothConnection();
  readBluetoothCommands();

  if (!servoScanEnabled) {
    setServoHome();
    return;
  }

  float distances[SENSOR_COUNT];
  readAllDistances(distances, SENSOR_COUNT);
  checkWarningAndSend(point.level, distances, SENSOR_COUNT);

  Serial.print("LEVEL=");
  Serial.print(point.level);
  Serial.print(" | S12=");
  Serial.print(point.servo12Angle);
  Serial.print(" | S34=");
  Serial.print(point.servo34Angle);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(" | d");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(distances[i]);
  }
  Serial.println();
}

void checkWarningAndSend(int scanLevel, const float distances[], int count) {
  if (scanLevel == 1) {
    if (hasAnyDistanceBelow(distances, count, WARNING_LOW)) {
      sendBluetoothLine("2");
    }

    if (hasAnyDistanceAbove(distances, count, HOLE_DISTANCE)) {
      sendBluetoothLine("3");
    }
    return;
  }

  if (scanLevel == 2) {
    if (hasAnyDistanceBelow(distances, count, WALL_DISTANCE)) {
      sendBluetoothLine("5");
    }
    return;
  }

  if (scanLevel == 3) {
    if (hasAnyDistanceBelow(distances, count, WARNING_HIGH)) {
      sendBluetoothLine("4");
    }
    return;
  }
}

bool isValidDistance(float distanceCm) {
  return distanceCm > 0.0;
}

bool hasAnyDistanceBelow(const float distances[], int count, float thresholdCm) {
  for (int i = 0; i < count; i++) {
    if (isValidDistance(distances[i]) && distances[i] < thresholdCm) {
      return true;
    }
  }
  return false;
}

bool hasAnyDistanceAbove(const float distances[], int count, float thresholdCm) {
  for (int i = 0; i < count; i++) {
    if (isValidDistance(distances[i]) && distances[i] > thresholdCm) {
      return true;
    }
  }
  return false;
}
