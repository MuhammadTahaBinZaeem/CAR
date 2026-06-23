// Minimal robot car sketch: drive straight at max speed and allow a single IR-triggered U-turn.
// Targets Arduino UNO WiFi R3 (ATmega328P + ESP8266) using an L298N motor driver.

#include <IRremote.h>

// --------------------------- Pin definitions ---------------------------
const uint8_t ENA_PIN = 5;  // L298N ENA (left motor PWM)
const uint8_t ENB_PIN = 6;  // L298N ENB (right motor PWM)
const uint8_t IN1_PIN = 12; // L298N IN1
const uint8_t IN2_PIN = 11; // L298N IN2
const uint8_t IN3_PIN = 10; // L298N IN3
const uint8_t IN4_PIN = 9;  // L298N IN4

const uint8_t IR_RECEIVER_PIN = 13; // IR receiver signal pin

// --------------------------- Control constants ---------------------------
const uint8_t MAX_SPEED = 255;            // Full power for continuous forward drive
const uint16_t UTURN_DURATION_MS = 700;   // Adjust to match your chassis
const uint8_t UTURN_SPEED = MAX_SPEED;    // Use full speed for the turn as well

// Use a single IR code to trigger the 180-degree turn. Replace with your remote's code if needed.
const unsigned long IR_CODE_UTURN = 0xA857FF00; // Existing "U-turn" code from prior setup

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

void turnRight(uint8_t speedValue) {
  setMotorPins(HIGH, LOW, LOW, HIGH);
  setMotorSpeed(speedValue, speedValue);
}

void stopMotors() {
  setMotorSpeed(0, 0);
}

// Turn in place to roughly 180 degrees; tune duration to match your chassis.
void performUTurn(uint8_t turnSpeed, uint16_t durationMs) {
  turnRight(turnSpeed);
  delay(durationMs);
  stopMotors();
}

// --------------------------- Setup and loop ---------------------------
void setup() {
  Serial.begin(9600);
  Serial.println(F("Straight-line mode with IR U-turn"));

  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  IrReceiver.begin(IR_RECEIVER_PIN, ENABLE_LED_FEEDBACK);

  stopMotors();
}

void loop() {
  if (IrReceiver.decode()) {
    unsigned long code = IrReceiver.decodedIRData.decodedRawData;
    if (code == IR_CODE_UTURN) {
      performUTurn(UTURN_SPEED, UTURN_DURATION_MS);
    }
    IrReceiver.resume();
  }

  // Always drive forward at maximum speed.
  driveForward(MAX_SPEED);
}
