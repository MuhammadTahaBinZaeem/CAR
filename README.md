# Arduino Robot Car (UNO WiFi R3)

Hands-on robotics project that turns an Arduino UNO WiFi R3 (ATmega328P + ESP8266) into a versatile, classroom-ready robot car. The sketch powers multiple autonomous and remote-control modes, adds safety protections for edges/cliffs, and documents all wiring so you can recreate or showcase the build on LinkedIn.

## What it does
- **Six driving modes:**
  - Line follower (dual IR sensors on D4/D2).
  - Obstacle avoider with ultrasonic scanning (HC-SR04 on D8/D7 plus a servo on D3).
  - Combo sprint mode that prioritizes max-speed line following.
  - IR-remote driving (receiver on D13 with configurable codes).
  - Wi‑Fi serial control via the UNO WiFi’s shared hardware serial port.
  - Bluetooth serial control (e.g., HC-05) using the same serial command set.
- **Motor control:** L298N H-bridge with PWM speed on D5/D6 and direction on D9–D12; helper functions handle forward, reverse, in-place turns, and a dedicated U-turn routine.
- **Safety features:** cliff/edge protection using the line sensors, emergency stop from the IR remote, obstacle avoidance fallback logic, and timed servo pauses for stable distance reads.
- **Tunable performance:** adjust default/turn speeds, max speed, servo settle delay, and obstacle thresholds at the top of the sketch.

## Circuit & wiring (match these pins or update the constants)
| Component | Pin mapping |
| --- | --- |
| L298N ENA / ENB | `D5` (PWM) / `D6` (PWM) |
| L298N IN1 / IN2 | `D12` / `D11` |
| L298N IN3 / IN4 | `D10` / `D9` |
| IR line sensors | Left -> `D4`, Right -> `D2` |
| Ultrasonic HC-SR04 | TRIG -> `D8`, ECHO -> `D7` |
| Servo (SG90) | Signal -> `D3` |
| IR receiver | Signal -> `D13` |
| Wi‑Fi (ESP8266) / Bluetooth HC-05 | Hardware serial (baud selectable; sketch uses `115200` by default) |

> If your wiring differs, edit the pin constants near the top of `robot_car.ino`.

## Controls
- **IR remote**
  - Direction: `IR_CODE_FWD`, `IR_CODE_BACK`, `IR_CODE_LEFT`, `IR_CODE_RIGHT`, `IR_CODE_STOP` (replace with your remote’s codes).
  - Modes: buttons mapped to `IR_CODE_MODE_LINE/OBSTACLE/COMBO/IR/WIFI/BT` plus combo-only `IR_CODE_COMBO_STOP` and `IR_CODE_COMBO_UTURN` for a quick escape.
- **Serial (Wi‑Fi or Bluetooth)**
  - Movement: `F`, `B`, `L`, `R`, `S` (stop), `U` (U-turn + resume forward).
  - Modes: `1` line follower, `2` obstacle avoider, `3` combo sprint, `4` IR driving, `5` Wi‑Fi serial, `6` Bluetooth serial.
  - Speed: digits `0–9` map to PWM `100–255`.

## How the modes behave
- **Line follower:** stays centered on the track; slows to turn when a single sensor sees the line and creeps forward when both lose it.
- **Obstacle avoider:** checks distance every 100 ms; backs up and turns toward the clearer side when an object is within the threshold (15 cm by default).
- **Combo sprint:** drives at `MAX_SPEED` on the line for showcase laps; intentionally ignores obstacle/cliff logic unless you hit the emergency stop.
- **IR driving:** IR codes set movement instantly; emergency stop halts any mode.
- **Wi‑Fi/Bluetooth:** processes the same serial commands as above so you can drive from a phone app or serial console.

## Build & upload
1. Open `robot_car.ino` in the Arduino IDE.
2. Select **Arduino UNO WiFi** and set the serial monitor to **115200 baud** (match `SERIAL_DEBUG_BAUD`).
3. Install libraries via Library Manager: [Servo](https://www.arduino.cc/reference/en/libraries/servo/) and [IRremote 4.x](https://github.com/Arduino-IRremote/Arduino-IRremote).
4. Upload. On boot you’ll see `Robot car starting` in the serial monitor.

## Tuning tips for a polished demo
- Adjust `DEFAULT_SPEED`, `TURN_SPEED`, and `OBSTACLE_THRESHOLD_CM` to fit your motor torque and sensor noise.
- If the servo horn is off-center, change the initial `moveServoAndWait(90);` call in `setup()` to your neutral angle.
- Increase `SCAN_DELAY_MS` if ultrasonic readings seem unstable during scans.
- Customize IR codes (`IR_CODE_*` constants) with values captured via the `IRrecvDump` example so your remote matches.
