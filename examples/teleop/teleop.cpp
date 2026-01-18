#include <AutOBot.h>

AutOBot myBot;

void setup() {
    Serial.begin(115200);
    
    myBot.setMotor1Pins(18, 19);
    myBot.setMotor2Pins(22, 23);
    myBot.setMotor3Pins(16, 17);
    
    myBot.begin("DRIVE_OMNI_3W", 0);

    myBot.telop();

}

void loop() {

}