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


