#include "AutOBot.h"

// ===== CONSTRUCTOR =====
AutOBot::AutOBot() : currX(0), currY(0), currW(0), deviation(0) {}


// ===== PIN SETUP =====
void AutOBot::setMotor1Pins(uint8_t a,uint8_t b){m1a=a; m1b=b;}
void AutOBot::setMotor2Pins(uint8_t a,uint8_t b){m2a=a; m2b=b;}
void AutOBot::setMotor3Pins(uint8_t a,uint8_t b){m3a=a; m3b=b;}
void AutOBot::setMotor4Pins(uint8_t a,uint8_t b){m4a=a; m4b=b;}


// ===== BEGIN =====
void AutOBot::begin(String type,int dev)
{
    driveType=type; deviation=dev;

    if(type=="DRIVE_DIFFERENTIAL"||type=="DRIVE_MECANUM"){
        pinMode(m1a,OUTPUT); pinMode(m1b,OUTPUT);
        pinMode(m2a,OUTPUT); pinMode(m2b,OUTPUT);
        pinMode(m3a,OUTPUT); pinMode(m3b,OUTPUT);
        pinMode(m4a,OUTPUT); pinMode(m4b,OUTPUT);
    }
    else if(type=="DRIVE_OMNI_3W"){
        pinMode(m1a,OUTPUT); pinMode(m1b,OUTPUT);
        pinMode(m2a,OUTPUT); pinMode(m2b,OUTPUT);
        pinMode(m3a,OUTPUT); pinMode(m3b,OUTPUT);
    }
}


// ===== MOTOR OUTPUT =====
void AutOBot::setMotor(uint8_t a,uint8_t b,int speed)
{
    speed=constrain(speed,-100,100);
    int pwm=map(abs(speed),0,100,0,255);

    if(speed>0){analogWrite(a,pwm); analogWrite(b,0);}
    else if(speed<0){analogWrite(a,0); analogWrite(b,pwm);}
    else{analogWrite(a,0); analogWrite(b,0);}
}


// ===== MAIN DRIVE =====
void AutOBot::drive(float x,float y,float w)
{
    x=constrain(x,-1,1); y=constrain(y,-1,1); w=constrain(w,-1,1);

    float tx=x*100, ty=y*100, tw=w*100;

    if(currX<tx)currX+=DECEL_STEP; else if(currX>tx)currX-=DECEL_STEP;
    if(currY<ty)currY+=DECEL_STEP; else if(currY>ty)currY-=DECEL_STEP;
    if(currW<tw)currW+=DECEL_STEP; else if(currW>tw)currW-=DECEL_STEP;

    float lf=1.0, rf=1.0;
    if(deviation<0)rf-=abs(deviation)/200.0;
    else if(deviation>0)lf-=abs(deviation)/200.0;


    // ---- DIFFERENTIAL 4WD ----
    if(driveType=="DRIVE_DIFFERENTIAL"){
        currX=0;
        int L=(currY-currW)*lf, R=(currY+currW)*rf;
        setMotor(m1a,m1b,L); setMotor(m3a,m3b,L);
        setMotor(m2a,m2b,R); setMotor(m4a,m4b,R);
    }

    // ---- OMNI ----
    else if(driveType=="DRIVE_OMNI_3W"){
        setMotor(m1a,m1b, currY+currW);
        setMotor(m2a,m2b,-0.5*currY+0.866*currX+currW);
        setMotor(m3a,m3b,-0.5*currY-0.866*currX+currW);
    }

    // ---- MECANUM ----
    else if(driveType=="DRIVE_MECANUM"){
        setMotor(m1a,m1b,currY+currX+currW);
        setMotor(m2a,m2b,currY-currX-currW);
        setMotor(m3a,m3b,currY-currX+currW);
        setMotor(m4a,m4b,currY+currX-currW);
    }
}


// ===== STOP =====
void AutOBot::stop()
{
    currX=currY=currW=0;
    setMotor(m1a,m1b,0); setMotor(m2a,m2b,0);
    setMotor(m3a,m3b,0); setMotor(m4a,m4b,0);
}


// ===== BASIC MOVES =====
void AutOBot::goForward(float s,int t){drive(0,s,0); delay(t); stop();}
void AutOBot::goBackward(float s,int t){drive(0,-s,0); delay(t); stop();}
void AutOBot::slideLeft(float s,int t){drive(-s,0,0); delay(t); stop();}
void AutOBot::slideRight(float s,int t){drive(s,0,0); delay(t); stop();}
void AutOBot::rotateCW(float s,int t){drive(0,0,s); delay(t); stop();}
void AutOBot::rotateCCW(float s,int t){drive(0,0,-s); delay(t); stop();}
void AutOBot::slide(int d,float s,int t)
{
    float rad=d*PI/180.0;
    drive(s*cos(rad),s*sin(rad),0);
    delay(t); stop();
}


// ===== BLE TELEOP =====
void AutOBot::telop()
{
    uint8_t mac[6]; esp_read_mac(mac,ESP_MAC_BT);

    String name=generateName(mac);
    String svc =generateUUID(mac,0xAA);
    String chr =generateUUID(mac,0xBB);

    Serial.println(name);

    BLEDevice::init(name.c_str());
    BLEServer* srv=BLEDevice::createServer(); srv->setCallbacks(this);

    BLEService* service=srv->createService(svc.c_str());

    BLECharacteristic* ch=service->createCharacteristic(
        chr.c_str(),
        BLECharacteristic::PROPERTY_WRITE|
        BLECharacteristic::PROPERTY_WRITE_NR
    );

    ch->setCallbacks(this);
    service->start();

    BLEAdvertising* adv=BLEDevice::getAdvertising();
    adv->addServiceUUID(svc.c_str());
    adv->start();
}


// ===== BLE EVENTS =====
void AutOBot::onConnect(BLEServer*){Serial.println("Connected");}

void AutOBot::onDisconnect(BLEServer*)
{
    Serial.println("Disconnected");
    stop();
    BLEDevice::startAdvertising();
}

void AutOBot::onWrite(BLECharacteristic* c)
{
    std::string v=c->getValue();
    if(v.empty())return;

    Serial.println(v.c_str());

    float x=0,y=0,w=0;
    sscanf(v.c_str(),"%f,%f,%f",&x,&y,&w);

    drive(x,y,w);
}


// ===== UUID / NAME =====
String AutOBot::generateUUID(uint8_t* mac,uint8_t salt)
{
    char u[37];
    snprintf(u,sizeof(u),
        "%02X%02X%02X%02X-%02X%02X-4000-%02X00-%02X%02X%02X%02X%02X%02X",
        mac[0],mac[1],mac[2],mac[3],
        mac[4],mac[5],salt,
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]
    );
    return String(u);
}

String AutOBot::generateName(uint8_t* mac)
{
    char n[16];
    sprintf(n,"BOT_%02X%02X",mac[4],mac[5]);
    return String(n);
}