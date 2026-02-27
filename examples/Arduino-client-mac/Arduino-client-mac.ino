#include "ArduinoRobot.h"

// ================= PINS (MECANUM) =================
// FL (Front Left)
const uint8_t FL_IN1 = 12; const uint8_t FL_IN2 = 13;
// FR (Front Right)
const uint8_t FR_IN1 = 23; const uint8_t FR_IN2 = 22;
// RL (Rear Left)
const uint8_t RL_IN1 = 32; const uint8_t RL_IN2 = 33;
// RR (Rear Right)
const uint8_t RR_IN1 = 21; const uint8_t RR_IN2 = 19;

// Pin Array: [FL1, FL2, FR1, FR2, RL1, RL2, RR1, RR2]
int motorPins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, RL_IN1, RL_IN2, RR_IN1, RR_IN2}; 

// ================= SERIAL CONFIG FOR PI =================
HardwareSerial PiSerial(2); 

#define PI_RX_PIN 16
#define PI_TX_PIN 17

// ================= OBJECTS =================
ArduinoClient robot;
PiCommunication piComm(PiSerial, robot);

void setup() {
  // 1. Start USB Serial (For Debugging)
  Serial.begin(115200);
  Serial.println("ESP32 Ready. Type commands to send to Pi...");

  // 2. Start Pi Serial
  PiSerial.begin(115200, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  
  // [CHANGE 1] Set Timeout to fix Lag
  PiSerial.setTimeout(5); 

  // [CHANGE 2] Fix init parameters
  robot.init(DRIVE_MECANUM, motorPins, 4, 0); 
  
  piComm.init();
  piComm.requestHuman();
}

void loop() {
  // 3. Handle Communication
  piComm.handle(); 

  //  //JUST DEBUG READING
  // if (PiSerial.available()) {
  //   // อ่านรวดเดียวจนกว่าจะเจอ \n (สิ้นสุดบรรทัด)
  //   String msg = PiSerial.readStringUntil('\n'); 
    
  //   // ปริ้นออกมาทีเดียวทั้งประโยค
  //   Serial.println(msg);
  // }
  // // JUST DEBUG SENDING (This is okay to keep)
  // // Allows you to type in Serial Monitor and send to Pi
  // if (Serial.available()) {
  //   String msg = Serial.readStringUntil('\n'); 
  //   PiSerial.println(msg); 
  //   Serial.print("Sent to Pi: ");
  //   Serial.println(msg);
  }
}