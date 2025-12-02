// Robot car sketch for Arduino UNO WiFi R3 (ATmega328P + ESP8266)
// Supports multiple control modes: line following, obstacle avoidance, combo,
// IR remote driving, Wi‑Fi serial control, and Bluetooth serial control.
//
// Hardware mapping:
// L298N: ENA -> D5 (PWM), ENB -> D6 (PWM), IN1->D12, IN2->D11, IN3->D10, IN4->D9
// Line sensors: left -> D4, right -> D2
// Ultrasonic (HC-SR04): TRIG -> D8, ECHO -> D7
// Servo (SG90) for ultrasonic sweep: signal -> D3
// IR receiver: signal -> D13
// Bluetooth HC-05 (optional): hardware Serial at 9600 or SoftwareSerial if desired
// ESP8266 Wi‑Fi: hardware Serial shared at configured baud (e.g., 115200)

#include <Servo.h>
#include <IRremote.h>

// --------------------------- Pin definitions ---------------------------
// Change these to match your motor driver pins if you rewired the L298N.
const uint8_t ENA_PIN = 5;
const uint8_t ENB_PIN = 6;
const uint8_t IN1_PIN = 12;
const uint8_t IN2_PIN = 11;
const uint8_t IN3_PIN = 10;
const uint8_t IN4_PIN = 9;

// Adjust the digital inputs below if your line sensors are connected elsewhere.
const uint8_t LEFT_IR_PIN  = 4;
const uint8_t RIGHT_IR_PIN = 2;

// Ultrasonic trigger/echo pins; update if you use a different pair.
const uint8_t TRIG_PIN = 8;
const uint8_t ECHO_PIN = 7;
const uint8_t SERVO_PIN = 3;

const uint8_t IR_RECEIVER_PIN = 13;

// --------------------------- Mode handling ---------------------------
enum Mode {
  MODE_LINE = 0,
  MODE_OBSTACLE,
  MODE_COMBO,
  MODE_IR_REMOTE,
  MODE_WIFI,
  MODE_BLUETOOTH
};

Mode currentMode = MODE_LINE;

// --------------------------- Control constants ---------------------------
// Tune these to fit your hardware (battery voltage, motor driver, sensor noise).
const uint16_t SERIAL_DEBUG_BAUD = 115200; // Match ESP8266 baud; monitor at this speed
const uint8_t DEFAULT_SPEED = 180;         // PWM 0-255; raise/lower to match your motors
const uint8_t TURN_SPEED = 160;            // Slower helps turning accuracy
const uint16_t OBSTACLE_THRESHOLD_CM = 15; // Increase if your ultrasonic sensor is noisy
const uint16_t SCAN_DELAY_MS = 200;        // Time for servo to settle when scanning

// --------------------------- IR remote codes ---------------------------
// Replace these with the codes from your MP3 remote using the IRrecvDump example.
const unsigned long IR_CODE_FWD = 0x00FFA25D;      // Replace with your remote's codes
const unsigned long IR_CODE_BACK = 0x00FF629D;
const unsigned long IR_CODE_LEFT = 0x00FF22DD;
const unsigned long IR_CODE_RIGHT = 0x00FFC23D;
const unsigned long IR_CODE_STOP = 0x00FF02FD;
const unsigned long IR_CODE_MODE_LINE = 0x00FF30CF;     // Button 1
const unsigned long IR_CODE_MODE_OBSTACLE = 0x00FF18E7; // Button 2
const unsigned long IR_CODE_MODE_COMBO = 0x00FF7A85;    // Button 3
const unsigned long IR_CODE_MODE_IR = 0x00FF10EF;       // Button 4
const unsigned long IR_CODE_MODE_WIFI = 0x00FF38C7;     // Button 5
const unsigned long IR_CODE_MODE_BT = 0x00FF5AA5;       // Button 6

// --------------------------- Globals ---------------------------
Servo scanServo;
// IRremote 4.x uses the singleton IrReceiver instance instead of IRrecv objects.
// Keeping it global simplifies command processing and avoids compatibility issues.

uint8_t currentSpeed = DEFAULT_SPEED;
unsigned long lastDistanceCheck = 0;
const uint16_t DISTANCE_CHECK_INTERVAL = 100; // ms

// --------------------------- Motor helpers ---------------------------
void setMotorPins(bool in1, bool in2, bool in3, bool in4) {
  digitalWrite(IN1_PIN, in1);
  digitalWrite(IN2_PIN, in2);
  digitalWrite(IN3_PIN, in3);
  digitalWrite(IN4_PIN, in4);
}

void setMotorSpeed(uint8_t leftSpeed, uint8_t rightSpeed) {
  analogWrite(ENA_PIN, leftSpeed);
  analogWrite(ENB_PIN, rightSpeed);
}

void driveForward(uint8_t speedValue) {
  setMotorPins(HIGH, LOW, HIGH, LOW);
  setMotorSpeed(speedValue, speedValue);
}

void driveBackward(uint8_t speedValue) {
  setMotorPins(LOW, HIGH, LOW, HIGH);
  setMotorSpeed(speedValue, speedValue);
}

void turnLeft(uint8_t speedValue) {
  setMotorPins(LOW, HIGH, HIGH, LOW);
  setMotorSpeed(speedValue, speedValue);
}

void turnRight(uint8_t speedValue) {
  setMotorPins(HIGH, LOW, LOW, HIGH);
  setMotorSpeed(speedValue, speedValue);
}

void stopMotors() {
  setMotorSpeed(0, 0);
}

// --------------------------- Sensor helpers ---------------------------
bool readLeftSensor() {
  return digitalRead(LEFT_IR_PIN) == HIGH;
}

bool readRightSensor() {
  return digitalRead(RIGHT_IR_PIN) == HIGH;
}

long readDistanceCm() {
  // Trigger the ultrasonic module.
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout ~5m
  long distance = duration / 58; // Convert to cm
  return distance;
}

// Rotate servo to given angle with a short pause for settling.
void moveServoAndWait(int angle) {
  scanServo.write(angle);
  delay(SCAN_DELAY_MS);
}

// Sweep servo to right and left to find free path.
long scanForObstacle() {
  moveServoAndWait(120); // Right
  long rightDistance = readDistanceCm();
  moveServoAndWait(60);  // Left
  long leftDistance = readDistanceCm();
  moveServoAndWait(90);  // Center again

  return leftDistance > rightDistance ? leftDistance : rightDistance;
}

// Decide turn direction based on measured distances.
void compareDistanceAndTurn() {
  moveServoAndWait(120);
  long rightDistance = readDistanceCm();
  moveServoAndWait(60);
  long leftDistance = readDistanceCm();
  moveServoAndWait(90);

  if (rightDistance > leftDistance) {
    Serial.println(F("Turning right to avoid obstacle"));
    turnRight(TURN_SPEED);
  } else {
    Serial.println(F("Turning left to avoid obstacle"));
    turnLeft(TURN_SPEED);
  }
  delay(400); // Short turn duration
  stopMotors();
}

// --------------------------- Mode handlers ---------------------------
// Basic line follower: uses only the two IR reflectance sensors.
void handleLineFollower() {
  bool leftOnLine = readLeftSensor();
  bool rightOnLine = readRightSensor();

  if (!leftOnLine && !rightOnLine) {
    driveForward(currentSpeed);
  } else if (leftOnLine && !rightOnLine) {
    turnLeft(TURN_SPEED);
  } else if (!leftOnLine && rightOnLine) {
    turnRight(TURN_SPEED);
  } else {
    stopMotors();
  }
}

// Obstacle avoider: checks distance at a fixed interval and turns away.
void handleObstacleAvoider() {
  unsigned long now = millis();
  if (now - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
    lastDistanceCheck = now;
    long distance = readDistanceCm();
    Serial.print(F("Distance: "));
    Serial.println(distance);

    if (distance > 0 && distance <= OBSTACLE_THRESHOLD_CM) {
      stopMotors();
      compareDistanceAndTurn();
    } else {
      driveForward(currentSpeed);
    }
  }
}

void handleComboMode() {
  unsigned long now = millis();
  long distance = -1;
  if (now - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
    lastDistanceCheck = now;
    distance = readDistanceCm();
  }

  if (distance > 0 && distance <= OBSTACLE_THRESHOLD_CM) {
    stopMotors();
    compareDistanceAndTurn();
    return;
  }

  handleLineFollower();
}

void handleIRDriving() {
  // Manual driving; nothing happens until an IR command sets motors.
}

void handleWifiAndBluetooth() {
  // Commands from Serial set motors directly in loop().
}

// --------------------------- Command processing ---------------------------
void setMode(Mode newMode) {
  if (currentMode != newMode) {
    currentMode = newMode;
    stopMotors();
    Serial.print(F("Mode changed to: "));
    Serial.println(currentMode);
  }
}

void processMovementCommand(char cmd) {
  switch (cmd) {
    case 'F':
      driveForward(currentSpeed);
      break;
    case 'B':
      driveBackward(currentSpeed);
      break;
    case 'L':
      turnLeft(TURN_SPEED);
      break;
    case 'R':
      turnRight(TURN_SPEED);
      break;
    default:
      stopMotors();
      break;
  }
}

void processModeCommand(char cmd) {
  switch (cmd) {
    case '1': setMode(MODE_LINE); break;
    case '2': setMode(MODE_OBSTACLE); break;
    case '3': setMode(MODE_COMBO); break;
    case '4': setMode(MODE_IR_REMOTE); break;
    case '5': setMode(MODE_WIFI); break;
    case '6': setMode(MODE_BLUETOOTH); break;
    default: break;
  }
}

void processSerialCommand(Stream &port) {
  if (port.available()) {
    char c = port.read();
    Serial.print(F("Serial cmd: "));
    Serial.println(c);

    if (c >= '1' && c <= '6') {
      processModeCommand(c);
    } else if (c >= '0' && c <= '9') {
      // Optional speed value; map 0-9 to PWM range.
      currentSpeed = map(c - '0', 0, 9, 100, 255);
      Serial.print(F("Speed set to: "));
      Serial.println(currentSpeed);
    } else {
      processMovementCommand(c);
    }
  }
}

void processIRRemote() {
  if (IrReceiver.decode()) {
    unsigned long code = IrReceiver.decodedIRData.decodedRawData;
    Serial.print(F("IR code: 0x"));
    Serial.println(code, HEX);

    if (code == IR_CODE_FWD) {
      processMovementCommand('F');
    } else if (code == IR_CODE_BACK) {
      processMovementCommand('B');
    } else if (code == IR_CODE_LEFT) {
      processMovementCommand('L');
    } else if (code == IR_CODE_RIGHT) {
      processMovementCommand('R');
    } else if (code == IR_CODE_STOP) {
      processMovementCommand('S');
    } else if (code == IR_CODE_MODE_LINE) {
      processModeCommand('1');
    } else if (code == IR_CODE_MODE_OBSTACLE) {
      processModeCommand('2');
    } else if (code == IR_CODE_MODE_COMBO) {
      processModeCommand('3');
    } else if (code == IR_CODE_MODE_IR) {
      processModeCommand('4');
    } else if (code == IR_CODE_MODE_WIFI) {
      processModeCommand('5');
    } else if (code == IR_CODE_MODE_BT) {
      processModeCommand('6');
    }

    IrReceiver.resume();
  }
}

// --------------------------- Setup and loop ---------------------------
// Initialize peripherals; adjust baud and servo center angle for your build.
void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial.println(F("Robot car starting"));

  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);

  digitalWrite(TRIG_PIN, LOW);

  scanServo.attach(SERVO_PIN);
  moveServoAndWait(90); // Center; change if your servo horn is offset

  stopMotors();
}

void loop() {
  processIRRemote();

  // Process Wi‑Fi and Bluetooth commands (same character set).
  if (currentMode == MODE_WIFI || currentMode == MODE_BLUETOOTH) {
    processSerialCommand(Serial);
  }

  switch (currentMode) {
    case MODE_LINE:
      handleLineFollower();
      break;
    case MODE_OBSTACLE:
      handleObstacleAvoider();
      break;
    case MODE_COMBO:
      handleComboMode();
      break;
    case MODE_IR_REMOTE:
      handleIRDriving();
      break;
    case MODE_WIFI:
    case MODE_BLUETOOTH:
      handleWifiAndBluetooth();
      break;
  }
}
