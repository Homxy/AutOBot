#ifndef AutOBotAI_h
#define AutOBotAI_h

#include <Arduino.h>
#include "AutOBot.h"

class AutOBotAI {
private:
    AutOBot* robot = nullptr;
    long lastUpdateTime = 0;
    bool connected = false;

public:
    AutOBotAI();
    
    void begin(AutOBot& robot);
    void requestHuman(int baseSpeed = 0, float turnGain = 0.5, int boxSize = 30000, int deadzone = 5000);
    void requestLine(int baseSpeed = 0, float turnGain = 0.5, int thresh = 100, String color = "black", int defS = 0, int defD = 0, int defA = 0);
    void setDriveType(String type);
    void requestStop();
    
    // Main processing loop
    void handle();
};

#endif