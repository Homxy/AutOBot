#include "AutOBot.h"
#include "AutOBotAI.h"


AutOBot robot;
AutOBotAI ai;

void setup() {

    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, ai.rx, ai.tx); // RX, TX

    robot.init(DRIVE_DIFFERENTIAL, 0);

    ai.begin(Serial2, robot);
    ai.requestHuman(50, 0.5, 30000, 5000);
  
}

void loop() {

  ai.handle();

}