#include <AutOBot.h>

AutOBot myBot;

void setup() {
    Serial.begin(115200);
    
    
    myBot.begin(DRIVE_OMNI_3W, 0);


}

void loop() {
    myBot.goForward(50, 1000);
    delay(2000);
    myBot.goBackward(50, 1000);
    delay(2000);
    myBot.slideLeft(50, 1000);
    delay(2000);
    myBot.slideRight(50, 1000);
    delay(2000);
    myBot.rotateCW(50, 1000);
    delay(2000);
    myBot.rotateCCW(50, 1000);
    delay(2000);
}