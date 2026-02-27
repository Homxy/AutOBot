#include "AutOBot.h"
#include "AutOBotAI.h"

HardwareSerial PiSerial(2);

AutOBot robot;
AutOBotAI ai;

void setup() {

    Serial.begin(115200);
    PiSerial.begin(115200);

    robot.init(DRIVE_DIFFERENTIAL, 0);

    ai.begin(PiSerial, robot);
    ai.requestLine(50, 0.5, 100, "black", 0, 0, 0);
  
}

void loop() {

  ai.handle();

}