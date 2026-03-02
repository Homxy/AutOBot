#include "AutOBotAI.h"

AutOBotAI::AutOBotAI() {}

void AutOBotAI::setRX(uint8_t a) { rx = a; }
void AutOBotAI::setTX(uint8_t b) { tx = b; }

void AutOBotAI::begin(Stream& serial, AutOBot& robot) {
    _serial = &serial;
    _robot  = &robot;

    _connected = false;
    _lastUpdateTime = millis();
}

// ================= REQUEST COMMANDS =================

void AutOBotAI::requestHuman(int baseSpeed,
                             float turnGain,
                             int boxSize,
                             int deadzone)
{
    if (!_serial) return;

    _serial->print(":M|HM=1|spd=");
    _serial->print(baseSpeed);
    _serial->print("|sen=");
    _serial->print(turnGain);
    _serial->print("|box=");
    _serial->print(boxSize);
    _serial->print("|dzone=");
    _serial->print(deadzone);
    _serial->println("|LN=0");
}

void AutOBotAI::requestLine(int baseSpeed,
                            float turnGain,
                            int thresh,
                            const char* color,
                            int defS,
                            int defD,
                            int defA)
{
    if (!_serial) return;

    _serial->print(":M|LN=1|spd=");
    _serial->print(baseSpeed);
    _serial->print("|sen=");
    _serial->print(turnGain);
    _serial->print("|thr=");
    _serial->print(thresh);
    _serial->print("|col=");
    _serial->print(color);
    _serial->print("|def=");
    _serial->print(defS);
    _serial->print(",");
    _serial->print(defD);
    _serial->print(",");
    _serial->print(defA);
    _serial->println("|HM=0");
}

void AutOBotAI::setDriveType(const char* type)
{
    if (!_serial) return;

    _serial->print(":M|type=");
    _serial->println(type);
}

void AutOBotAI::requestStop()
{
    if (!_serial) return;

    _serial->println(":M|HM=0|LN=0");
}

// ================= MAIN LOOP =================

void AutOBotAI::handle()
{
    if (!_serial || !_robot) return;

    while (_serial->available() > 0)
    {
        char c = _serial->read();

        if (c == '<')
        {
            int s = _serial->parseInt();
            int d = _serial->parseInt();
            int a = _serial->parseInt();

            if (_serial->read() == '>')
            {
                _robot->drive(s, d, a);

                _lastUpdateTime = millis();
                _connected = true;
            }
        }
    }

    // ===== Watchdog (1 second timeout) =====
    if (_connected && (millis() - _lastUpdateTime > 1000))
    {
        _connected = false;
        _robot->stop();
    }
}//