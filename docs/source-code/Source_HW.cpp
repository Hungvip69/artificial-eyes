#include <ESP32Servo.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define WARNING_LOW   40
#define HOLE_DISTANCE 120
#define WALL_DISTANCE 70
#define WARNING_HIGH  60

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const int servoPin1 = 13;
const int servoPin2 = 12;
const int servoPin3 = 14;
const int servoPin4 = 27;

const int SERVO12_MIN = 50;
const int SERVO12_MAX = 100;

const int SERVO34_MIN = 90;
const int SERVO34_MAX = 130;

const int smoothSteps = 80;

const int smoothDelay = 12;

const int servoPairDelay = 3;

bool servoScanEnabled = true;

bool bluetoothConnected = false;
unsigned long lastBTCheckTime = 0;
const unsigned long btCheckInterval = 1000;

const int trigPin1 = 26;
const int echoPin1 = 25;

const int trigPin2 = 33;
const int echoPin2 = 32;

const int trigPin3 = 5;
const int echoPin3 = 18;

const int trigPin4 = 19;
const int echoPin4 = 21;

float getDistance(int trigPin, int echoPin);

void scanAtPosition(int servo12Angle, int servo34Angle, int scanLevel);
void moveServoSmooth(int start12, int end12, int start34, int end34);
void checkWarningAndSend(int scanLevel, float d1, float d2, float d3, float d4);

void readBluetoothCommand();
void checkBluetoothConnection();
void setServoHome();

void setup() {
  Serial.begin(115200);

  SerialBT.begin("ArtificialEye_Hardware");
  Serial.println("Bluetooth started: ArtificialEye_Hardware");

  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  servo4.setPeriodHertz(50);

  servo1.attach(servoPin1, 500, 2500);
  servo2.attach(servoPin2, 500, 2500);
  servo3.attach(servoPin3, 500, 2500);
  servo4.attach(servoPin4, 500, 2500);

  setServoHome();

  // Cảm biến siêu âm
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);

  delay(1000);

  Serial.println("=== BAT DAU HE THONG ===");
  Serial.println("Gui ON de bat quet servo");
  Serial.println("Gui OFF de tat quet servo");
}

void loop() {
  checkBluetoothConnection();
  readBluetoothCommand();

  if (servoScanEnabled) {
    moveServoSmooth(50, 65, 130, 120);
    scanAtPosition(65, 120, 1);

    delay(200);
    checkBluetoothConnection();
    readBluetoothCommand();

    if (!servoScanEnabled) return;

    moveServoSmooth(65, 90, 120, 99);
    scanAtPosition(90, 99, 2);

    delay(200);
    checkBluetoothConnection();
    readBluetoothCommand();

    if (!servoScanEnabled) return;

    moveServoSmooth(90, 100, 99, 90);
    scanAtPosition(100, 90, 3);

    delay(200);
    checkBluetoothConnection();
    readBluetoothCommand();

    if (!servoScanEnabled) return;

    moveServoSmooth(100, 50, 90, 130);

    delay(300);
  } 
  else {
    checkBluetoothConnection();
    readBluetoothCommand();
    delay(50);
  }
}

void checkBluetoothConnection() {
  if (millis() - lastBTCheckTime >= btCheckInterval) {
    lastBTCheckTime = millis();

    bool currentState = SerialBT.hasClient();

    if (currentState != bluetoothConnected) {
      bluetoothConnected = currentState;

      if (bluetoothConnected) {
        Serial.println("Bluetooth device connected");
        SerialBT.println("ArtificialEye_Hardware_CONNECTED");
      } 
      else {
        Serial.println("Bluetooth device disconnected");
      }
    }
  }
}

void readBluetoothCommand() {
  if (SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    Serial.print("Bluetooth command: ");
    Serial.println(cmd);

    if (cmd == "ON") {
      servoScanEnabled = true;

      Serial.println("Servo scan ON");
      SerialBT.println("SERVO_SCAN_ON");
    } 
    else if (cmd == "OFF") {
      servoScanEnabled = false;

      Serial.println("Servo scan OFF");
      SerialBT.println("SERVO_SCAN_OFF");

      setServoHome();
    }
  }
}

void setServoHome() {
  servo1.write(SERVO12_MIN);
  servo2.write(SERVO12_MIN);

  delay(servoPairDelay);

  servo3.write(SERVO34_MAX);
  servo4.write(SERVO34_MAX);
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);

  if (duration == 0) {
    return -1;
  }

  float distance = duration * 0.0343 / 2.0;
  return distance;
}

void moveServoSmooth(int start12, int end12, int start34, int end34) {
  for (int i = 0; i <= smoothSteps; i++) {
    checkBluetoothConnection();
    readBluetoothCommand();

    if (!servoScanEnabled) {
      setServoHome();
      return;
    }

    float t = (float)i / smoothSteps;

    float smoothT = t * t * (3 - 2 * t);

    int angle12 = start12 + (end12 - start12) * smoothT;
    int angle34 = start34 + (end34 - start34) * smoothT;

    servo1.write(angle12);
    servo2.write(angle12);

    delay(servoPairDelay);

    servo3.write(angle34);
    servo4.write(angle34);

    delay(smoothDelay);
  }
}

void scanAtPosition(int servo12Angle, int servo34Angle, int scanLevel) {
  checkBluetoothConnection();
  readBluetoothCommand();

  if (!servoScanEnabled) {
    setServoHome();
    return;
  }

  servo1.write(servo12Angle);
  servo2.write(servo12Angle);

  delay(servoPairDelay);

  servo3.write(servo34Angle);
  servo4.write(servo34Angle);

  delay(400);

  checkBluetoothConnection();
  readBluetoothCommand();

  if (!servoScanEnabled) {
    setServoHome();
    return;
  }

  float d1 = getDistance(trigPin1, echoPin1);
  float d2 = getDistance(trigPin2, echoPin2);
  float d3 = getDistance(trigPin3, echoPin3);
  float d4 = getDistance(trigPin4, echoPin4);

  checkWarningAndSend(scanLevel, d1, d2, d3, d4);

  Serial.print("LEVEL=");
  Serial.print(scanLevel);

  Serial.print(" | S12=");
  Serial.print(servo12Angle);

  Serial.print(" | S34=");
  Serial.print(servo34Angle);

  Serial.print(" | d1=");
  Serial.print(d1);

  Serial.print(" d2=");
  Serial.print(d2);

  Serial.print(" d3=");
  Serial.print(d3);

  Serial.print(" d4=");
  Serial.println(d4);
}

void checkWarningAndSend(int scanLevel, float d1, float d2, float d3, float d4) {
  if (scanLevel == 1) {
    if (d1 > 0 && d1 < WARNING_LOW) SerialBT.println("2");
    if (d2 > 0 && d2 < WARNING_LOW) SerialBT.println("2");
    if (d3 > 0 && d3 < WARNING_LOW) SerialBT.println("2");
    if (d4 > 0 && d4 < WARNING_LOW) SerialBT.println("2");

    if (d1 > HOLE_DISTANCE || d2 > HOLE_DISTANCE || d3 > HOLE_DISTANCE || d4 > HOLE_DISTANCE) {
      SerialBT.println("3");
    }
  }


  if (scanLevel == 2) {
    if ((d1 > 0 && d1 < WALL_DISTANCE) ||
        (d2 > 0 && d2 < WALL_DISTANCE) ||
        (d3 > 0 && d3 < WALL_DISTANCE) ||
        (d4 > 0 && d4 < WALL_DISTANCE)) {
      SerialBT.println("5");
    }
  }


  if (scanLevel == 3) {
    if ((d1 > 0 && d1 < WARNING_HIGH) ||
        (d2 > 0 && d2 < WARNING_HIGH) ||
        (d3 > 0 && d3 < WARNING_HIGH) ||
        (d4 > 0 && d4 < WARNING_HIGH)) {
      SerialBT.println("4");
    }
  }
}