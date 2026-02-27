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

// Timing Variables
unsigned long startTime;
bool phaseLineStarted = false;
bool phaseStopStarted = false;

void setup() {
  // 1. Start USB Serial (For Debugging)
  Serial.begin(115200);
  Serial.println("ESP32 Ready. Starting Automated Sequence...");

  // 2. Start Pi Serial
  PiSerial.begin(115200, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
  PiSerial.setTimeout(5); 

  // 3. Init Robot (Mecanum, 4 Motors, Deviation=0)
  robot.init(DRIVE_MECANUM, motorPins, 4, 0); 
  
  // 4. Init Comms
  piComm.init();
  
  // 5. START PHASE 1: Human Mode (Immediately)
  Serial.println("PHASE 1: Human Mode (10s)");
  // Params: speed=30, gain=0.6, boxSize=30000, deadzone=5000
  piComm.requestHuman(30, 0.6, 30000, 5000);
  
  startTime = millis();
}

void loop() {
  // 1. REQUIRED: Always keep this running to receive drive commands from Pi
  piComm.handle(); 

  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - startTime;

  // 2. TIMING LOGIC
  
  // Check if 10 seconds have passed -> Switch to Line Mode
  if (!phaseLineStarted && elapsed > 10000) {
    Serial.println("PHASE 2: Switching to Line Mode (15s)");
    // Params: speed=30, gain=0.8, thresh=100, color="black"
    piComm.requestLine(30, 0.8, 100, "black");
    phaseLineStarted = true;
  }

  // Check if 25 seconds have passed (10s Human + 15s Line) -> Stop
  if (!phaseStopStarted && elapsed > 25000) {
    Serial.println("PHASE 3: Sequence Complete. Stopping.");
    piComm.requestStop();
    phaseStopStarted = true;
  }
}