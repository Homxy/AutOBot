#ifndef AutOBot_h
#define AutOBot_h

#include <Arduino.h>
#include <esp_system.h>
#include <esp_mac.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

class AutOBot : public BLEServerCallbacks, public BLECharacteristicCallbacks {
    private:
        uint8_t m1a = 12, m1b = 13; 
        uint8_t m2a = 14, m2b = 27; 
        uint8_t m3a = 26, m3b = 25;
        uint8_t m4a = 33, m4b = 32;
        
        int deviation;
        String driveType;

        void setMotor(uint8_t a, uint8_t b, int speed);
        String generateUUID(uint8_t* mac, uint8_t salt);
        String generateName(uint8_t* mac);

    public:
        AutOBot();
        void setMotor1Pins(uint8_t a, uint8_t b);
        void setMotor2Pins(uint8_t a, uint8_t b);
        void setMotor3Pins(uint8_t a, uint8_t b);
        void setMotor4Pins(uint8_t a, uint8_t b);
        
        void begin(String IdriveType, int Ideviation);
        void drive(float x, float y, float w);
        void stop();

        //baicmove
        void goForward(float speed, int timeMs);
        void goBackward(float speed, int timeMs);
        void slideLeft(float speed, int timeMs);
        void slideRight(float speed, int timeMs);
        void rotateCW(float speed, int timeMs);
        void rotateCCW(float speed, int timeMs);
        void slide(int degree, float speed, int timeMs);
        
        //teleop
        void telop();
        void onConnect(BLEServer* pServer) override;
        void onDisconnect(BLEServer* pServer) override;
        void onWrite(BLECharacteristic* pCharacteristic) override;
};

#endif