#include <AutOBot.h>

AutOBot myBot;

void setup() {
    Serial.begin(115200);
    
    
    myBot.begin(DRIVE_OMNI_3W, 0);

    myBot.telop();

}

void loop() {

}