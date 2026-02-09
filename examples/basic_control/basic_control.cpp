#include <AutOBot.h>

AutOBot myBot;

void setup() {
    Serial.begin(115200);
    
    myBot.setMotor1Pins(18, 19);
    myBot.setMotor2Pins(22, 23);
    myBot.setMotor3Pins(16, 17);
    //DRIVE_DIFFERENTIAL, DRIVE_OMNI_3W, DRIVE_MECANUM
    myBot.begin("DRIVE_OMNI_3W", 0);
}

void loop() {
    myBot.goForward(80, 2000);
    delay(2000);
    myBot.goBackward(80, 2000);
    delay(2000);
    myBot.slideLeft(80, 2000);
    delay(2000);
    myBot.slideRight(80, 2000);
    delay(2000);

}



