#include "ArduinoRobot.h"

// ================= PIN CONFIGURATION =================
// ขามอเตอร์ที่คุณกำหนดมา (ESP32 38-pin)
int motorPins[] = {18, 19, 22, 23}; 

// ================= SERIAL CONFIG FOR PI =================
// สร้าง Object HardwareSerial สำหรับคุยกับ Pi โดยใช้ UART2
HardwareSerial PiSerial(2); 

// กำหนดขาสำหรับคุยกับ Pi (สามารถเปลี่ยนได้ถ้าขา 16/17 ไม่ว่าง)
#define PI_RX_PIN 16
#define PI_TX_PIN 17

// ================= OBJECTS =================
ArduinoClient robot;
PiCommunication piComm(PiSerial, robot);

void setup() {
  // 1. เริ่มต้น Serial Monitor (สำหรับต่อคอมพิวเตอร์ผ่าน USB)
  Serial.begin(115200);
  Serial.println("ESP32 Ready. Type commands to send to Pi...");

  // 2. เริ่มต้น Serial สำหรับ Pi (Baud rate ต้องตรงกับ Python)
  // รูปแบบ: .begin(baud, config, rx_pin, tx_pin)
  PiSerial.begin(115200, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);

  // 3. เริ่มต้น Robot (Drive Type: Differential 2 ล้อ)
  robot.init(DRIVE_DIFFERENTIAL, motorPins, 2, 0);
  
  // 4. เริ่มต้นระบบสื่อสาร
  piComm.init();
}

void loop() {
  piComm.handle(); 



  //JUST DEBUG READING
  if (PiSerial.available()) {
    // อ่านรวดเดียวจนกว่าจะเจอ \n (สิ้นสุดบรรทัด)
    String msg = PiSerial.readStringUntil('\n'); 
    
    // ปริ้นออกมาทีเดียวทั้งประโยค
    Serial.println(msg);
  }
  //JUST DEBUG SENDING 
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n'); 
    PiSerial.println(msg); 
    Serial.print("Sent to Pi: ");
    Serial.println(msg);
  }
}