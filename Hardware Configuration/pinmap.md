# 📌 MicroMouse Hardware Pin Mapping Guide

This document outlines the standard hardware pin assignments for the **Arduino Mega 2560** used across all legacy Arduino test sketches and the modular PlatformIO codebase (`PlatformIO/include/config.h`).

All team members must follow this pin layout during physical assembly, testing, and debugging.

---

## 1. Summary Table

| Hardware Component | Component Pin / Function | Arduino Mega Pin | Signal Type | Notes / Description |
| :--- | :--- | :--- | :--- | :--- |
| **Motor Drivers (BTS7960 / IBT-2)** | **`PIN_MOTOR_EN`** | **D7** | Digital Output | Shared Enable Pin for both Left & Right drivers. |
| | **`PIN_LEFT_RPWM`** | **D4** | PWM Output | Left Driver Forward PWM. |
| | **`PIN_LEFT_LPWM`** | **D6** | PWM Output | Left Driver Reverse PWM. |
| | **`PIN_RIGHT_RPWM`** | **D9** | PWM Output | Right Driver Forward PWM. |
| | **`PIN_RIGHT_LPWM`** | **D8** | PWM Output | Right Driver Reverse PWM. |
| **Quadrature Encoders** | **`PIN_LEFT_ENC_A`** | **D2** | Interrupt (INT0) | Left Encoder Phase A (RISING interrupt trigger). |
| | **`PIN_LEFT_ENC_B`** | **D3** | Interrupt (INT1) | Left Encoder Phase B (Direction checking). |
| | **`PIN_RIGHT_ENC_A`** | **D18** | Interrupt (INT5) | Right Encoder Phase A (RISING interrupt trigger). |
| | **`PIN_RIGHT_ENC_B`** | **D19** | Interrupt (INT4) | Right Encoder Phase B (Direction checking). |
| **IR Sensor Array (8-Chan)** | **`PIN_IR_EN`** | **D49** | Digital Output | Power control pin for infrared emitter LEDs. |
| | **`IR0`** (Sensor 1) | **A15** | Analog Input | Left-most IR sensor channel. |
| | **`IR1`** (Sensor 2) | **A14** | Analog Input | IR sensor channel 2. |
| | **`IR2`** (Sensor 3) | **A13** | Analog Input | IR sensor channel 3. |
| | **`IR3`** (Sensor 4) | **A12** | Analog Input | Center-left IR sensor channel. |
| | **`IR4`** (Sensor 5) | **A11** | Analog Input | Center-right IR sensor channel. |
| | **`IR5`** (Sensor 6) | **A10** | Analog Input | IR sensor channel 6. |
| | **`IR6`** (Sensor 7) | **A9** | Analog Input | IR sensor channel 7. |
| | **`IR7`** (Sensor 8) | **A8** | Analog Input | Right-most IR sensor channel. |
| **Ultrasonic Sensor (HC-SR04)** | **`PIN_ULTRA_TRIG`** | **D12** | Digital Output | Ultrasonic Trigger pulse pin. |
| | **`PIN_ULTRA_ECHO`** | **D13** | Digital Input | Ultrasonic Echo pulse-width measurement pin. |

---

## 2. Detailed Pin Breakdown

### 🚗 Motor Drivers & Encoders
* **Motor Driver Style:** BTS7960 / IBT-2 High Current Drivers.
* **Enable Logic:** Setting `D7` (`PIN_MOTOR_EN`) to `HIGH` enables output power to both wheel channels.
* **Encoders:** Connected to hardware interrupt-capable pins on the ATmega2560.
  * `D2` and `D18` trigger Phase A interrupts.
  * `D3` and `D19` read Phase B high/low state inside the ISR to determine forward vs. backward pulse incrementing.

### 🚨 8-Channel Analog IR Sensor Array
* **Enable Pin:** `D49` (`PIN_IR_EN`) powers on the IR transmitter LEDs. Always ensure this pin is set to `OUTPUT` and written `HIGH` during `setup()`.
* **Channel Ordering:**
  ```text
  [Left]  A15 ──── A14 ──── A13 ──── A12 ──── A11 ──── A10 ──── A9 ──── A8  [Right]

### 🦇 Ultrasonic Distance Sensor (HC-SR04)
* **Trigger (`D12`):** Requires a 10µs high pulse to send out acoustic bursts.
* **Echo (`D13`):** Measures high signal duration with `pulseIn()` to calculate obstacle clearance distance in centimeters.

---

## 3. Physical Calibration Constants

Keep these physical values in mind when writing or tuning motion routines:

| Constant Parameter | Calibrated Value | Notes |
| :--- | :--- | :--- |
| **Cruising Speed (`BASE_PWM`)** | `70` | Standard motor PWM output (0–255 range). |
| **1 Cell Distance (`TICKS_PER_CELL_25CM`)** | `288.0` ticks | Equivalent to 25 cm linear travel. |
| **90° Pivot Turn (`TICKS_PER_90_DEG`)** | `116.5` ticks | Average encoder pulse count for 90° turn. |
| **180° U-Turn (`TICKS_PER_180_DEG`)** | `232.5` ticks | Average encoder pulse count for 180° turn. |