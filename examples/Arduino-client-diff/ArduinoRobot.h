#ifndef ARDUINO_ROBOT_H
#define ARDUINO_ROBOT_H

#include <Arduino.h>

// ================= CONFIGURATION =================
#define DECEL_STEP 10      // Acceleration smoothness
#define PWM_FREQ   100     // 100Hz = More Torque (Low frequency for power)
#define PWM_RES    8       // 8-bit resolution (0-255)

// ================= ENUMS =================
enum DriveType {
  DRIVE_DIFFERENTIAL, 
  DRIVE_OMNI_3W,      
  DRIVE_MECANUM       
};

// ================= CLASS: ArduinoClient =================
class ArduinoClient {
  private:
    DriveType _type;
    int* _pins = nullptr; 
    int* _motorValues = nullptr; 
    int _deviation; 
    int _numMotors;
    
    // Ramping variables
    float _currSpeed;
    float _currAngular;
    float _lastDirection; 

    // Helper: Control single motor
    void setMotor(int pin1, int pin2, int speed) {
      speed = constrain(speed, -100, 100);
      
      // 1. Deadzone
      if (abs(speed) < 2) { 
        ledcWrite(pin1, 0); 
        ledcWrite(pin2, 0); 
        return;
      }

      // 2. Minimum PWM Floor (Stiction Fix)
      int minPWM = 60; 
      
      int pwm = map(abs(speed), 0, 100, minPWM, 255);
      
      // 3. Output to Pins (ESP32 v3.0 API)
      if (speed > 0) { 
        ledcWrite(pin1, pwm); 
        ledcWrite(pin2, 0); 
      } else { 
        ledcWrite(pin1, 0); 
        ledcWrite(pin2, pwm); 
      }
    }

  public:
    ArduinoClient() { 
      _currSpeed = 0;
      _currAngular = 0;
      _lastDirection = 0;
      _numMotors = 0;
    }

    void init(DriveType type, int pins[], int numMotors, int deviation) {
      _type = type;
      _numMotors = numMotors; 
      _deviation = deviation;
      
      if (_pins != nullptr) delete[] _pins;
      if (_motorValues != nullptr) delete[] _motorValues;

      int totalPins = _numMotors * 2;
      _pins = new int[totalPins];
      _motorValues = new int[_numMotors];

      // Setup Pins
      for (int i = 0; i < totalPins; i++) {
        _pins[i] = pins[i];
        
        pinMode(_pins[i], OUTPUT);
        ledcAttach(_pins[i], PWM_FREQ, PWM_RES);
        ledcWrite(_pins[i], 0);
      }
    }

    // Main Drive Function
    void drive(float targetSpeed, float targetDir, float targetAngular) {
      
      // --- RAMPING ---
      if (_currSpeed < targetSpeed) {
        _currSpeed += DECEL_STEP; 
        if (_currSpeed > targetSpeed) _currSpeed = targetSpeed;
      } else if (_currSpeed > targetSpeed) {
        _currSpeed -= DECEL_STEP; 
        if (_currSpeed < targetSpeed) _currSpeed = targetSpeed;
      }

      if (_currAngular < targetAngular) {
        _currAngular += DECEL_STEP; 
        if (_currAngular > targetAngular) _currAngular = targetAngular;
      } else if (_currAngular > targetAngular) {
        _currAngular -= DECEL_STEP; 
        if (_currAngular < targetAngular) _currAngular = targetAngular;
      }

      if (targetSpeed > 0) _lastDirection = targetDir;
      float useDir = (targetSpeed == 0 && abs(_currSpeed) > 0) ? _lastDirection : targetDir;

      // --- KINEMATICS ---
      float theta = useDir * PI / 180.0;
      float Vx = _currSpeed * sin(theta);
      float Vy = _currSpeed * cos(theta);
      float W  = _currAngular;           

      if (_motorValues == nullptr) return;

      for(int i=0; i<_numMotors; i++) _motorValues[i] = 0;

      switch (_type) {
        case DRIVE_DIFFERENTIAL:
          {
            float left = Vy + W;
            float right = Vy - W;
            for(int i=0; i<_numMotors; i++) _motorValues[i] = (i % 2 == 0) ? left : right;
          }
          break;
          
        case DRIVE_OMNI_3W:
          if (_numMotors >= 3) {
            _motorValues[0] = -Vx + W; 
            _motorValues[1] = (0.5 * Vx) - (0.866 * Vy) + W;
            _motorValues[2] = (0.5 * Vx) + (0.866 * Vy) + W;
          }
          break;
          
        case DRIVE_MECANUM:
          if (_numMotors >= 4) {
            _motorValues[0] = Vy + Vx + W; 
            _motorValues[1] = Vy - Vx - W; 
            _motorValues[2] = Vy - Vx + W; 
            _motorValues[3] = Vy + Vx - W; 
          }
          break;
      }

      // --- NORMALIZATION ---
      float leftFactor = 1.0; float rightFactor = 1.0;
      if (_deviation < 0) rightFactor = 1.0 - (abs(_deviation) / 200.0);
      else if (_deviation > 0) leftFactor = 1.0 - (abs(_deviation) / 200.0);
      
      int maxVal = 0;
      for(int i=0; i<_numMotors; i++) {
        if (_type == DRIVE_DIFFERENTIAL) {
           if (i % 2 == 0) _motorValues[i] *= leftFactor; else _motorValues[i] *= rightFactor;
        } else if (_type == DRIVE_MECANUM && i < 4) {
           if (i == 0 || i == 2) _motorValues[i] *= leftFactor; else _motorValues[i] *= rightFactor;
        }
        if (abs(_motorValues[i]) > maxVal) maxVal = abs(_motorValues[i]);
      }

      // --- OUTPUT ---
      for(int i=0; i<_numMotors; i++) {
        if (maxVal > 100) {
          _motorValues[i] = map(_motorValues[i], -maxVal, maxVal, -100, 100);
        }
        setMotor(_pins[2*i], _pins[2*i+1], _motorValues[i]);
      }
    }

    void stop() {
      _currSpeed = 0;
      _currAngular = 0;
      if (_pins != nullptr) {
        for(int i=0; i<_numMotors * 2; i++) {
          ledcWrite(_pins[i], 0);
        }
      }
    }
};

// ================= CLASS: PiCommunication =================
class PiCommunication {
  private:
    Stream& _serial;
    ArduinoClient& _robot;
    unsigned long _lastUpdate;
    bool _connected;

  public:
    PiCommunication(Stream& serialPort, ArduinoClient& robot) 
      : _serial(serialPort), _robot(robot) { _connected = false; }

    void init() { _lastUpdate = millis(); }

    void requestHuman(int baseSpeed = 0, float turnGain = 0.5, int boxSize = 30000, int deadzone = 5000) { 
      _serial.print(":M|HM=1|spd="); _serial.print(baseSpeed);
      _serial.print("|sen="); _serial.print(turnGain);
      _serial.print("|box="); _serial.print(boxSize);
      _serial.print("|dzone="); _serial.print(deadzone);
      _serial.println("|LN=0");
    }

    void requestLine(int baseSpeed = 0, float turnGain = 0.5, int thresh = 100, String color = "black", int defS = 0, int defD = 0, int defA = 0) { 
      _serial.print(":M|LN=1|spd="); _serial.print(baseSpeed);
      _serial.print("|sen="); _serial.print(turnGain);
      _serial.print("|thr="); _serial.print(thresh);
      _serial.print("|col="); _serial.print(color);
      // Format: def=speed,dir,ang
      _serial.print("|def="); _serial.print(defS); _serial.print(","); _serial.print(defD); _serial.print(","); _serial.print(defA);
      _serial.println("|HM=0");
    }

    void setDriveType(String type) {
      _serial.print(":M|type="); _serial.println(type);
    }

    void requestStop() { _serial.println(":M|HM=0|LN=0"); }

    void handle() {
      while (_serial.available() > 0) {
        char c = _serial.read();
        if (c == '<') {
          int s = _serial.parseInt(); 
          int d = _serial.parseInt(); 
          int a = _serial.parseInt(); 
          if (_serial.read() == '>') {
             _robot.drive(s, d, a);
             _lastUpdate = millis();
             _connected = true;
          }
        }
      }
      if (millis() - _lastUpdate > 1000 && _connected) {
        _connected = false;
        _robot.stop(); 
      }
    }
};

#endif