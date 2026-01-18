#include "HardwareSerial.h"
#include "AutOBot.h"
#include "AutOBotAI.h"


// ================= SERIAL CONFIG FOR PI =================
HardwareSerial PiSerial(2); 

#define PI_RX_PIN 16
#define PI_TX_PIN 17

// ================= OBJECTS =================
AutOBot robot;
AutOBotAI piComm;

void setup() {
  // 1. Start USB Serial (For Debugging)
  Serial.begin(115200);
  Serial.println("ESP32 Ready. Type commands to send to Pi...");

  // 2. Start Pi Serial
  PiSerial.begin(115200, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  
  // [CHANGE 1] Set Timeout to fix Lag
  PiSerial.setTimeout(5); 

  // [CHANGE 2] Fix init parameters
  robot.begin("DRIVE_MECANUM", 0); 

  piComm.begin(PiSerial, robot);
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