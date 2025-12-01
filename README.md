# CAR

Arduino sketch for a multi-mode robot car built around the Arduino UNO WiFi R3 (ATmega328P + ESP8266). The program supports line following, obstacle avoidance, combined line/obstacle behavior, IR-remote driving, and simple serial control over Wi‑Fi or Bluetooth.

## Features
- **Multiple driving modes:** line follower, obstacle avoider, combo (line + obstacle), IR-remote driving, Wi‑Fi serial, and Bluetooth serial.
- **Motor driver support:** L298N with PWM speed control on `D5`/`D6` and direction on `D10–D13`.
- **Sensors:** dual IR line sensors on `D4`/`D2`, ultrasonic distance sensor on `D9`/`D8`, and an SG90 servo on `A5` for sweeping the ultrasonic sensor.
- **Remote control:** works with an IR receiver on `D7` plus serial commands shared with the ESP8266 or an HC-05 module.

## Dependencies
- [Servo](https://www.arduino.cc/reference/en/libraries/servo/)
- [IRremote 4.x](https://github.com/Arduino-IRremote/Arduino-IRremote)

Install these libraries through the Arduino Library Manager before compiling.

## Wiring reference
| Component | Pin mapping |
| --- | --- |
| L298N ENA / ENB | `D5` (PWM) / `D6` (PWM) |
| L298N IN1 / IN2 | `D12` / `D13` |
| L298N IN3 / IN4 | `D10` / `D11` |
| IR line sensors | Left -> `D4`, Right -> `D2` |
| Ultrasonic HC-SR04 | TRIG -> `D9`, ECHO -> `D8` |
| Servo (SG90) | Signal -> `A5` |
| IR receiver | Signal -> `D7` |

Adjust the pin assignments in `robot_car.ino` if your wiring differs.

## Serial commands
Send single-character commands over the hardware serial port (shared with Wi‑Fi/Bluetooth modules):

- Movement: `F` (forward), `B` (backward), `L` (left), `R` (right), `S` or any other char (stop).
- Modes: `1` line follower, `2` obstacle avoider, `3` combo, `4` IR-remote driving, `5` Wi‑Fi serial, `6` Bluetooth serial.
- Speed: digits `0–9` map to PWM values 100–255.

## IR remote mapping
Replace the IR codes in the `IR_CODE_*` constants with the values from your remote (use the `IRrecvDump` example to capture them). Default mapping uses common MP3 remote buttons: forward/back/left/right, stop, and buttons 1–6 to switch modes.

## Mode behavior
- **Line follower:** keeps moving forward; turns when a sensor detects the line.
- **Obstacle avoider:** checks distance every 100 ms; stops and turns toward the clearer side when an object is within 15 cm.
- **Combo:** follows the line but falls back to obstacle avoidance when something blocks the path.
- **IR remote:** waits for IR commands to set movement; no autonomous behavior.
- **Wi‑Fi/Bluetooth:** expects the serial commands listed above.

## Building and uploading
1. Open `robot_car.ino` in the Arduino IDE.
2. Select the **Arduino UNO WiFi** board and set the serial monitor baud rate to **115200** to match the sketch.
3. Verify the libraries are installed, then upload the sketch.
4. With the serial monitor open, you should see `Robot car starting` on boot.

## Tuning tips
- Adjust `DEFAULT_SPEED`, `TURN_SPEED`, and `OBSTACLE_THRESHOLD_CM` near the top of the sketch to fit your motors and sensors.
- If your servo horn is off-center, change the initial `moveServoAndWait(90);` call in `setup()` to your neutral angle.
- Increase `SCAN_DELAY_MS` if the ultrasonic readings seem unstable during servo sweeps.
