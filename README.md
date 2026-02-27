<<<<<<< HEAD
# AutOBot & AutOBotAI

An ESP32-based robot control library for mobile robots with **differential**, **3-wheel omni**, and **4-wheel mecanum** drive systems. The library supports **BLE teleoperation** and **AI-assisted control** (human-following / line-following) via serial communication.

---

## Features

### AutOBot (Low-level Control)

* Multiple drive modes:

  * Differential Drive
  * 3-Wheel Omni Drive
  * 4-Wheel Mecanum Drive
* Motor abstraction with direction + PWM
* High-level motion helpers (forward, slide, rotate)
* BLE teleoperation (ESP32)
* Auto-generated BLE name & UUID (based on MAC address)
* Auto-stop on BLE disconnect

### AutOBotAI (High-level AI Interface)

* Serial interface to external AI / vision system
* Human-following mode
* Line-following mode
* Connection watchdog (auto-stop on signal loss)

---

## AutOBot Usage

### Create Robot Object

```cpp
#include "AutOBot.h"

AutOBot robot;
```

---

### Assign Motor Pins

```cpp
robot.setMotor1Pins(m1a, m1b);
robot.setMotor2Pins(m2a, m2b);
robot.setMotor3Pins(m3a, m3b); // omni / mecanum
robot.setMotor4Pins(m4a, m4b); // mecanum only
```

---

### Initialize Drive Type

```cpp
robot.begin("DRIVE_DIFFERENTIAL", 0);
```

Supported drive types:

* `DRIVE_DIFFERENTIAL`
* `DRIVE_OMNI_3W`
* `DRIVE_MECANUM`

`deviation` compensates for mechanical bias.

---

### Drive Control

```cpp
robot.drive(x, y, w);
```

| Parameter | Description        |
| --------- | ------------------ |
| `x`       | Left / Right       |
| `y`       | Forward / Backward |
| `w`       | Rotation           |

Typical range: **-100 to 100**

---

### Basic Movement Helpers

```cpp
robot.goForward(50, 1000);
robot.goBackward(50, 1000);
robot.slideLeft(40, 800);
robot.slideRight(40, 800);
robot.rotateCW(30, 500);
robot.rotateCCW(30, 500);
```

---

### Slide at Arbitrary Angle

```cpp
robot.slide(45, 50, 1000); // degrees
```

---

### Stop Robot

```cpp
robot.stop();
```

---

## BLE Teleoperation

### Start BLE Teleop

```cpp
robot.teleop();
```

* Auto device name: `BOT_XXXX`
* UUIDs derived from ESP32 MAC address
* Advertising restarts on disconnect

### BLE Payload Format

```
x,y,w
```

Example:

```
0.5,-0.2,0.1
```

---

## AutOBotAI Usage

### Create AI Controller

```cpp
#include "AutOBotAI.h"

AutOBotAI ai;
```

---

### Initialize AI Communication

```cpp
ai.begin(Serial2, robot);
ai.setRxTx(rxPin, txPin);
```

---

### Request Human-Following Mode

```cpp
ai.requestHuman(
  baseSpeed,
  turnGain,
  boxSize,
  deadzone
);
```

---

### Request Line-Following Mode

```cpp
ai.requestLine(
  baseSpeed,
  turnGain,
  threshold,
  "black",
  defaultSpeed,
  defaultDirection,
  defaultAngle
);
```

---

### Stop AI Control

```cpp
ai.requestStop();
```

---

### Handle AI Updates (loop)

```cpp
void loop() {
  ai.handle();
}
```

AI data format:

```
<speed,direction,angle>
```

If no data is received for **> 1 second**, the robot stops automatically.

---


## Notes

* Ensure PWM channels are configured for ESP32
* BLE teleop and AI control can coexist
* Suitable for research, education, and competition robots

---

## License

MIT License – free to use, modify, and distribute.

---

=======

# Autobot Library

Autobot_Library is a modular and efficient Arduino library designed to simplify mobile robot development. It abstracts complex kinematics for various drive types and provides built-in communication handlers for remote control via WiFi (UDP) or AI coprocessors (Raspberry Pi/Jetson)


# Requirements

## 1. Arduino (Basic Control)
Hardware and software required for basic robot movement (without AI).

### Hardware
* **Microcontroller:** ESP32 Development Board (Recommended for `HardwareSerial` support).
* **Motor Driver:** PWM-compatible driver (e.g., L298N, TB6612FNG).
    * 2/4 Channels for Differential Drive.
    * 4 Channels for Mecanum/Omni Drive.
* **Motors:** DC Motors (2 or 4 units depending on drive type).
* **Power Supply:** Li-ion Battery (2S or 3S) suitable for your motors.

### Software
* **Arduino IDE:** Latest version.
* **Library:** `ArduinoRobot.h` (Must be included in your project folder).

---

## 2. Arduino + Raspberry Pi (AI & Vision)
Additional requirements for Human Tracking and Line Tracking features.

### Hardware
* **SBC:** Raspberry Pi 4 or 5 (Recommended for YOLOv8 performance).
* **Camera:** USB Webcam or Raspberry Pi Camera.
* **Connection:** 3x Jumper wires for UART communication (RX, TX, GND).

### Wiring (ESP32 <-> Raspberry Pi)
Connect the ESP32 to the Raspberry Pi GPIO headers as follows:

| Signal | ESP32 Pin | Raspberry Pi Pin |
| :--- | :--- | :--- |
| **RX** | Pin 16 | GPIO 14 (TXD) |
| **TX** | Pin 17 | GPIO 15 (RXD) |
| **GND** | GND | GND |

### Python Dependencies (Raspberry Pi)
Run the following commands on your Raspberry Pi to install necessary libraries:

```bash
# System dependencies
sudo apt-get install libopencv-dev python3-pip

# Python libraries
pip install opencv-python-headless numpy gpiod pyserial ultralytics
```


## Installation

Download  [ArduinoRobot.h](https://github.com/CannabiZz9/Autobot_Library/blob/main/ArduinoRobot.h)
Add to Arduino IDE``` #include "ArduinoRobot.h"```

(With Raspberry Pi)
Downlaod [RaspberryPi Image.bin](https://archive.org/embed/autobot_202602)
Use [BalenaEtcher](https://etcher.balena.io) to install image to sd card

## Raspberry Pi : autobot@autobot-pi
## Password :  (press spacebar one time)
    
## Demo

[2-Wheel Differential Drive Robot](https://github.com/CannabiZz9/Autobot_Library/tree/main/Arduino-client-diff)

[4-Wheel Mecanum Drive Robot](https://github.com/CannabiZz9/Autobot_Library/tree/main/Arduino-client-mac)


## Function Reference

#### **Create Class Robot(Init)**

```
  #include "ArduinoRobot.h"
  robot.init(type, motorpins, number of motors, deviation); 
```
 
| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `type` | `string` | DRIVE_DIFFERENTIAL / DRIVE_OMNI_3W / DRIVE_MECANUM |
| `motorpins (diff 2w)` | `array` |{FL_IN1, FL_IN2, FR_IN1, FR_IN2} |
| `motorpins (diff 4w)` | `array` |{FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2} |
| `motorpins (omni)` | `array` |{M1_IN1, M1_IN2, M2_IN1, M2_IN2, M3_IN1, M3_IN2} |
| `motorpins (mac)` | `array` |{FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2} |
| `number of motors` | `int` |just a number for motors(2, 3, 4, 6, ...) |
| `deviation` | `int` |use pwm to control deviation of robot rectilinear motion (min -100 max 100) | 


`Negative Values (-1 to -100):` Reduces the speed of the RIGHT motors.
Use this if your robot naturally drifts to the LEFT.

`Positive Values (1 to 100):` Reduces the speed of the LEFT motors.
Use this if your robot naturally drifts to the RIGHT.


#
#### **Drive Function** 

```
  robot.drive(speed, dir, ang);
```

| Parameter | Type     | Range |Description                       | 
| :-------- | :------- | ------- |:-------------------------------- |
| `speed`      | `int` | 0 -> 100 | speed of robot move | 
| `dir`      | `int` | 0 -> 360 | direction 0 forward / 180 backward |
| `ang`      | `int` | -100 -> 100 | rotation -100 left / 100 right |

#
#### **PI Communication( For AI )**

in void setup
```
  PiSerial.begin(bautrate, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  piComm.init();
```

| Parameter | Type     | Default |Description                       | 
| :-------- | :------- | ------- |:-------------------------------- |
| `bautrate`      | `int` | 115200 | bautrate for communication | 
| `PI_RX_PIN`      | `int` | 16 | arduino pin that connect to PI rx |
| `PI_TX_PIN`      | `int` | 17 | arduino pin that connect to PI tx |

#
to request AI (Human tracking)
```
  requestHuman(baseSpeed, turnGain, boxSize, deadzone)
```
| Parameter | Type     | Default |Description                       | 
| :-------- | :------- | ------- |:-------------------------------- |
| `baseSpeed`      | `int` | 0 | 0 for only rotation. 100 robot always move forward | 
| `turnGain`      | `float` | 0.5 | more gain = more sensitivity (undershoot, overshoot) |
| `boxSize`      | `int` | 30000 | more number = robot closer to human |
| `deadzone`      | `int` | 5000 | a zone that robot will not turn |

#
to request AI (Line tracking)
```
  requestLine(baseSpeed, turnGain, thresh, color, defS, defD, defA)
```
| Parameter | Type     | Default |Description                       | 
| :-------- | :------- | ------- |:-------------------------------- |
| `baseSpeed`      | `int` | 0 | 0 for only rotation. 100 robot always move forward | 
| `turnGain`      | `float` | 0.5 | more gain = more sensitivity (undershoot, overshoot) |
| `thresh`      | `int` | 100 | threshold for masked frame  |
| `color`      | `String` | black | line color |
| `defS`      | `int` | 0 | default speed if line not found |
| `defD`      | `int` | 0 | default direction if line not found |
| `defA`      | `int` | 0 | default angular if line not found |


#
to Set drive type (for mecanum to slide)
```
  setDriveType(String type)
```
#
to stop all AI processing
```
  requestStop()
```


#
in void loop

use to receive (speed, dir, ang) from pi and drive robot.
```
  piComm.handle(); 
```

##

#### **Communication Protocal** 
start with ```:M```|
```parameter=value```|
```parameter=value```|....

example 
```:M|spd=50|sen=0.8|LN=1```  -> Sets speed to 50, sensitivity to 0.8, and starts Line Follow mode.

| Parameter | Type      |Description                       | 
| :-------- | :-------  |:-------------------------------- |
| `LN`      | `int`  | line track mode (0,stop) (1,start) | 
| `HM`      | `int`  | human track mode (0,stop) (1,start)| 
| `spd`      | `int`  | Moving speed (0-100) (both)| 
| `sen`      | `float`  | Sensitivity (both) |
| `thr`      | `int`  | Threshold (0-100) (line tracking)|
| `type`      | `String`  | type of drive(diff, omni, mac) |
| `box`      | `int` | human box size (human tracking) |
| `dzone`      | `int`| deadzone (both) |
| `col`      | `int`  | line color  (line tracking)|
| `def`      | `int`  | def=speed,direction,angular (e.g., 50,180,0) (line tracking)|


#
[![AGPL License](https://img.shields.io/badge/license-AGPL-blue.svg)](http://www.gnu.org/licenses/agpl-3.0)
>>>>>>> aa92eb520cf7601f92098fa8d663d211d55150bf

