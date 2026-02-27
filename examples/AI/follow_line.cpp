#include <AutOBot.h>
#include <AutOBotAI.h>

AutOBot myBot;
AutOBotAI myAI;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, myAI.rx, myAI.tx); // RX, TX for AI comms
  
  myBot.begin(DRIVE_OMNI_3W, 0);
  myAI.begin(Serial, myBot);
  myAI.requestLine(50, 0.5, 100, "black", 0, 0, 0);

}

void loop() {
  myAI.handle();
}