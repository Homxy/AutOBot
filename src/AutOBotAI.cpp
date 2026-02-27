#include "AutOBotAI.h"

AutOBotAI::AutOBotAI() {}

void AutOBotAI::begin(AutOBot& robot) {
    this->robot = &robot;
    connected = false;
    lastUpdateTime = millis(); 
}

void AutOBotAI::requestHuman(int baseSpeed, float turnGain, int boxSize, int deadzone) { 
    Serial.print(":M|HM=1|spd="); Serial.print(baseSpeed);
    Serial.print("|sen="); Serial.print(turnGain);
    Serial.print("|box="); Serial.print(boxSize);
    Serial.print("|dzone="); Serial.print(deadzone);
    Serial.println("|LN=0");
}

void AutOBotAI::requestLine(int baseSpeed, float turnGain, int thresh, String color, int defS, int defD, int defA) { 
    Serial.print(":M|LN=1|spd="); Serial.print(baseSpeed);
    Serial.print("|sen="); Serial.print(turnGain);
    Serial.print("|thr="); Serial.print(thresh);
    Serial.print("|col="); Serial.print(color);
    Serial.print("|def="); Serial.print(defS); Serial.print(","); 
    Serial.print(defD); Serial.print(","); 
    Serial.print(defA);
    Serial.println("|HM=0");
}

void AutOBotAI::setDriveType(String type) {
    Serial.print(":M|type="); Serial.println(type);
}

void AutOBotAI::requestStop() { 
    Serial.println(":M|HM=0|LN=0"); 
}

void AutOBotAI::handle() {
    // Parse incoming data: <speed,direction,angular>
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '<') {
            int s = Serial.parseInt(); 
            int d = Serial.parseInt(); 
            int a = Serial.parseInt(); 
            
            // Check for closing tag to ensure packet integrity
            if (Serial.read() == '>') {
                robot->drive(s, d, a);
                lastUpdateTime = millis();
                connected = true;
            }
        }
    }

    // Safety Watchdog: Stop robot if Pi disconnects (1 second timeout)
    if (connected && (millis() - lastUpdateTime > 1000)) {
        connected = false;
        robot->stop(); 
    }
}