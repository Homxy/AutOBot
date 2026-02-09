#include "AutOBot.h"

AutOBot::AutOBot() {}

void AutOBot::setMotor1Pins(uint8_t a, uint8_t b) { m1a = a; m1b = b; }
void AutOBot::setMotor2Pins(uint8_t a, uint8_t b) { m2a = a; m2b = b; }
void AutOBot::setMotor3Pins(uint8_t a, uint8_t b) { m3a = a; m3b = b; }
void AutOBot::setMotor4Pins(uint8_t a, uint8_t b) { m4a = a; m4b = b; }

void AutOBot::begin(String IdriveType, int Ideviation) {
    this->driveType = IdriveType;
    this->deviation = Ideviation;
    if(driveType == "DRIVE_DIFFERENTIAL"){
        pinMode(m1a, OUTPUT); pinMode(m1b, OUTPUT);
        pinMode(m2a, OUTPUT); pinMode(m2b, OUTPUT);
    }
    if (driveType == "DRIVE_OMNI_3W" || driveType == "DRIVE_MECANUM") {
        pinMode(m3a, OUTPUT); pinMode(m3b, OUTPUT);
    }
    if (driveType == "DRIVE_MECANUM") {
        pinMode(m4a, OUTPUT); pinMode(m4b, OUTPUT);
    }
}

void AutOBot::setMotor(uint8_t a, uint8_t b, int speed) {
    speed = constrain(speed, -100, 100);
    int pwm = map(abs(speed), 0, 100, 0, 255);
    if (speed > 0) {
        analogWrite(a, pwm);
        analogWrite(b, 0);
    } else if (speed < 0) {
        analogWrite(a, 0);
        analogWrite(b, pwm);
    } else {
        analogWrite(a, 0);
        analogWrite(b, 0);
    }
}

void AutOBot::drive(float x, float y, float w) {
    int vx = constrain(x, -1.0, 1.0) * 255;
    int vy = constrain(y, -1.0, 1.0) * 255;
    int vw = constrain(w, -1.0, 1.0) * 255;
    float leftFactor = 1.0; 
    float rightFactor = 1.0;

    if (deviation < 0) {
        rightFactor = 1.0 - (abs(deviation) / 200.0);
    }
    else if (deviation > 0) {
        leftFactor = 1.0 - (abs(deviation) / 200.0);
    }

    if (driveType == "DRIVE_DIFFERENTIAL") {
        setMotor(m1a, m1b, vy - vx);
        setMotor(m2a, m2b, vy + vx);
    } 
    else if (driveType == "DRIVE_OMNI_3W") {
        setMotor(m1a, m1b, vy + vw);
        setMotor(m2a, m2b, -0.5 * vy + 0.866 * vx + vw);
        setMotor(m3a, m3b, -0.5 * vy - 0.866 * vx + vw);
    } 
    else if (driveType == "DRIVE_MECANUM") {
        setMotor(m1a, m1b, vy + vx + vw);
        setMotor(m2a, m2b, vy - vx - vw);
        setMotor(m3a, m3b, vy + vx + vw);
        setMotor(m4a, m4b, vy - vx - vw);
    }
}

void AutOBot::stop() { 
    drive(0, 0, 0); 
}
//basic move
void AutOBot::goForward(float speed, int timeMs) {
    drive(speed, 0, 0);
    delay(timeMs);
    stop();
}

void AutOBot::goBackward(float speed, int timeMs) {
    drive(-speed, 0, 0);
    delay(timeMs);
    stop();
}

void AutOBot::slideLeft(float speed, int timeMs) {
    drive(0, -speed, 0);
    delay(timeMs);
    stop();
}

void AutOBot::slideRight(float speed, int timeMs) {
    drive(0, speed, 0);
    delay(timeMs);
    stop();
}

void AutOBot::rotateCW(float speed, int timeMs) {
    drive(speed, speed, 0);
    delay(timeMs);
    stop();
}

void AutOBot::rotateCCW(float speed, int timeMs) {
    drive(-speed, speed, 0);
    delay(timeMs);
    stop();
}

void AutOBot::slide(int degree, float speed, int timeMs) {
    float radians = degree * (PI / 180.0);
    float x = cos(radians) * speed;
    float y = sin(radians) * speed;
    drive(x, y, 0);
    delay(timeMs);
    stop();
}


//teleop
void AutOBot::telop() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);

    String deviceName = generateName(mac);
    String serviceUUID = generateUUID(mac, 0xAA);
    String charUUID = generateUUID(mac, 0xBB);
    Serial.println(deviceName + ":" + serviceUUID + ":" + charUUID);
    Serial.println("BLE Device: " + deviceName);
    BLEDevice::init(deviceName.c_str());
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(this);

    BLEService* pService = pServer->createService(serviceUUID.c_str());
    BLECharacteristic* pChar = pService->createCharacteristic(
        charUUID.c_str(),
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );

    pChar->setCallbacks(this);
    pService->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(serviceUUID.c_str());
    adv->start();
}


String AutOBot::generateUUID(uint8_t* mac, uint8_t salt) {
    char uuid[37];
    snprintf(uuid, sizeof(uuid), 
        "%02x%02x%02x%02x-%02x%02x-4000-%02x00-%02x%02x%02x%02x%02x%02x",
        mac[0], mac[1], mac[2], mac[3], 
        mac[4], mac[5],                 
        salt,                           
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] 
    );
    return String(uuid);
}

String AutOBot::generateName(uint8_t* mac) {
    esp_read_mac(mac, ESP_MAC_BT); 
    
    char buf[12];
    sprintf(buf, "BOT_%02X%02X", mac[4], mac[5]);
    
    return String(buf);
}

void AutOBot::onConnect(BLEServer* pServer) {
    Serial.println("Connected");
}

void AutOBot::onDisconnect(BLEServer* pServer) { 
    Serial.println("Disconnected");
    stop();
    BLEDevice::startAdvertising(); 
}

void AutOBot::onWrite(BLECharacteristic* c) {
    String value = c->getValue();
    if (value.length() > 0) {
        String s = String(value.c_str());
        // Simple JSON-ish parsing for {"x":0.5, "y":-0.2, "w":0.1}
        Serial.println("Received: " + s);
        int commaIndex = s.indexOf(',');
        if (commaIndex == -1) return;

        float x = s.substring(0, commaIndex).toFloat();
        float y = s.substring(commaIndex + 1).toFloat();
        float w = s.substring(s.lastIndexOf(',') + 1).toFloat();
        drive(x, y, w);
    }
}