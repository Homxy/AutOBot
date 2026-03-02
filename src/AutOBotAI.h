#ifndef AutOBotAI_h
#define AutOBotAI_h

#include <Arduino.h>
#include "AutOBot.h"

class AutOBotAI {
private:
    
    Stream* _serial = nullptr; 
    AutOBot* _robot = nullptr;
    unsigned long _lastUpdateTime = 0;
    bool _connected = false;

public:
    AutOBotAI();

    uint8_t rx = 16, tx = 17;

    void setRX(uint8_t a);
    void setTX(uint8_t b);
    
    void begin(Stream& serial, AutOBot& robot);

    void requestHuman(int baseSpeed = 0,
                      float turnGain = 0.5,
                      int boxSize = 30000,
                      int deadzone = 5000);

    void requestLine(int baseSpeed = 0,
                     float turnGain = 0.5,
                     int thresh = 100,
                     const char* color = "black",
                     int defS = 0,
                     int defD = 0,
                     int defA = 0);

    void setDriveType(const char* type);

    void requestStop();

    void handle();

};

#endif