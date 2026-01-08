
# Autobot Library

Autobot_Library is a modular and efficient Arduino library designed to simplify mobile robot development. It abstracts complex kinematics for various drive types and provides built-in communication handlers for remote control via WiFi (UDP) or AI coprocessors (Raspberry Pi/Jetson)


# Requirements

## 1. Arduino (Basic Control)
Hardware and software required for basic robot movement (without AI).

### Hardware
* **Microcontroller:** ESP32 Development Board (Recommended for `HardwareSerial` support).
* **Motor Driver:** PWM-compatible driver (e.g., L298N, TB6612FNG).
    * 2 Channels for Differential Drive.
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

