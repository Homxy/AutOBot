#include "AutOBotAI.h"
#include "AutOBot.h"

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
  Serial.print("|def="); Serial.print(defS); Serial.print(","); Serial.print(defD); Serial.print(","); Serial.print(defA);
  Serial.println("|HM=0");
}

void AutOBotAI::requestStop() {
    Serial.println(":M|HM=0|LN=0");
}

void AutOBotAI::handle() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '<') {
            int s = Serial.parseInt();
            int d = Serial.parseInt();
            int a = Serial.parseInt();
            if (Serial.read() == '>') {
                lastUpdateTime = millis();
                connected = true;
            }
        }
    }
    if (millis() - lastUpdateTime > 1000 && connected) {
        connected = false;
        robot->drive(0, 0, 0);
    }
}