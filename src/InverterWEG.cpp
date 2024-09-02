#include "InverterWEG.h"

uint8_t InverterWEG::_rs485_de = -1;
uint8_t InverterWEG::_rs485_re = -1;

InverterWEG::InverterWEG()
{
    _spin = false;
    _enable = false;
    _direct = true;
    _jog = false;
    _remote = false;
    _ramp2 = false;
    _reset = false;

    _interval = 3;
}

InverterWEG::~InverterWEG()
{
    if(_modbus)
        delete _modbus;
}

void InverterWEG::begin(uint8_t address, Stream &serial)
{
    _rs485_de = -1;
    _rs485_re = -1;
    if(_modbus)
        delete _modbus;
    _modbus = new ModbusMaster;
    _modbus->begin(address, serial);
}

void InverterWEG::begin(uint8_t address, Stream &serial, int rs485_de, int rs485_re)
{
    if(_modbus)
        delete _modbus;
    _modbus = new ModbusMaster;
    _modbus->begin(address, serial);
    _rs485_de = rs485_de;
    _rs485_re = rs485_re;
    _modbus->preTransmission(preTX);
    _modbus->postTransmission(postTX);
}

void InverterWEG::begin(ModbusMaster &modbus)
{
    if(_modbus)
        delete _modbus;
    _modbus = &modbus;
}

/*
    \brief Enable Inverter
    \return Communication Success
*/
bool InverterWEG::enable()
{
    _enable = true;
    uint8_t result = writeHReg(kCmdP, CmdWord());
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Disable Inverter
    \return Communication Success
*/
bool InverterWEG::disable()
{
    _enable = false;
    uint8_t result = writeHReg(kCmdP, CmdWord());
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Start/Spin Motor
    \return Communication Success
*/
bool InverterWEG::start()
{
    _spin = true;
    uint8_t result = writeHReg(kCmdP, CmdWord());
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Stop/Break Motor
    \return Communication Success
*/
bool InverterWEG::stop()
{
    _spin = false;
    uint8_t result = writeHReg(kCmdP, CmdWord());
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Set Reference Speed
    \param speed Frequency in HZ
    \return Communication Success
*/
bool InverterWEG::setSpeed(double speed)
{
    uint16_t value = speed * 8192 / 60;
    uint8_t result = writeHReg(kRefSpeedP, value);
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Set Motor Direction
    \param direct true -> forwards
    \param direct false -> backwards
    \return Communication Success
*/
bool InverterWEG::direction(bool direct)
{
    _direct = direct;
    uint8_t result = writeHReg(kCmdP, CmdWord());
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Chose Acceleration and Deceleration Ramp
    \param ramp 1 -> first ramp
    \param ramp 2 -> second ramp
    \return Communication Success
*/
bool InverterWEG::selectRamp(uint8_t ramp)
{
    if(ramp == 1)
        _ramp2 = false;
    else if(ramp == 2)
        _ramp2 = true;
    else
        return false;

    uint8_t result = writeHReg(kCmdP, CmdWord());
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Setup First Ramp
    \param accelTime Acceleration Time in Seconds
    \param decelTime Deceleration Time in Seconds
    \return Communication Success
*/
bool InverterWEG::configRamp1(double accelTime, double decelTime)
{
    _modbus->setTransmitBuffer(0, accelTime * 10);
    _modbus->setTransmitBuffer(1, decelTime * 10);
    uint8_t result = writeMultiReg(kAccel1P, 2);
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Setup Second Ramp
    \param accelTime Acceleration Time in Seconds
    \param decelTime Deceleration Time in Seconds
    \return Communication Success
*/
bool InverterWEG::configRamp2(double accelTime, double decelTime)
{
    _modbus->setTransmitBuffer(0, accelTime * 10);
    _modbus->setTransmitBuffer(1, decelTime * 10);
    uint8_t result = writeMultiReg(kAccel2P, 2);
    return result == _modbus->ku8MBSuccess;
}

/*
    \brief Reset Inverter Fail Status
    \return Communication Success
*/
bool InverterWEG::resetFaults()
{
    _reset = true;
    uint8_t result = writeHReg(kCmdP, CmdWord());
    _reset = false;
    return result == _modbus->ku8MBSuccess;

}

/*
    \brief Read the Motor Current
    \return Current in Amps (returns -1 if failed)
*/
double InverterWEG::current()
{
    double data = -1;
    uint8_t result = readHReg(kCurrentP, 1);
    if(result == _modbus->ku8MBSuccess)
        data = _modbus->getResponseBuffer(0) / 10.0;
    return data;
}

/*
    \brief Read the Inverter Temperature
    \return Temperature in Celsius (returns -1 if failed)
*/
double InverterWEG::temperature()
{
    double data = -1;
    uint8_t result = readHReg(kInvTempP, 1);
    if(result == _modbus->ku8MBSuccess)
        data = _modbus->getResponseBuffer(0) / 10.0;
    return data;
}

/*
    \brief Read the Motor Speed
    \return Frequency in Hz (returns -1 if failed)
*/
double InverterWEG::motorSpeed()
{
    double data = -1;
    uint8_t result = readHReg(kMotorSpeedP, 1);
    if(result == _modbus->ku8MBSuccess)
        data = _modbus->getResponseBuffer(0) / 10.0;
    return data;
}

/*
    \brief Set the minimum delay between Serial Packets
    \param interval Time in ms
*/
void InverterWEG::setTxInterval(uint32_t interval)
{
    _interval = interval;
}


uint8_t InverterWEG::readHReg(uint16_t address, uint16_t qtd)
{
    uint32_t elapsed = 0;
#ifdef ESP32
        elapsed = pdTICKS_TO_MS(xTaskGetTickCount() - _then);
#else //!ESP32
        elapsed = millis() - _then;
#endif //ESP32

    if(elapsed < _interval)
        delay(_interval - elapsed);

    uint8_t result = _modbus->readHoldingRegisters(address, qtd);
    
#ifdef ESP32
        _then = xTaskGetTickCount();
#else //!ESP32
        _then = millis();
#endif //ESP32
    
    return result;
}

uint8_t InverterWEG::writeHReg(uint16_t address, uint16_t value)
{
    uint32_t elapsed = 0;
#ifdef ESP32
        elapsed = pdTICKS_TO_MS(xTaskGetTickCount() - _then);
#else //!ESP32
        elapsed = millis() - _then;
#endif //ESP32

    if(elapsed < _interval)
        delay(_interval - elapsed);

    uint8_t result = _modbus->writeSingleRegister(address, value);
    
#ifdef ESP32
        _then = xTaskGetTickCount();
#else //!ESP32
        _then = millis();
#endif //ESP32
    
    return result;
}

uint8_t InverterWEG::writeMultiReg(uint16_t address, uint16_t qtd)
{
    uint32_t elapsed = 0;
#ifdef ESP32
        elapsed = pdTICKS_TO_MS(xTaskGetTickCount() - _then);
#else //!ESP32
        elapsed = millis() - _then;
#endif //ESP32

    if(elapsed < _interval)
        delay(_interval - elapsed);

    uint8_t result = _modbus->writeMultipleRegisters(address, qtd);
    
#ifdef ESP32
        _then = xTaskGetTickCount();
#else //!ESP32
        _then = millis();
#endif //ESP32
    
    return result;
}

uint16_t InverterWEG::CmdWord()
{
    uint16_t word = 0;
    word += _spin * kSpin;
    word += _enable * kEnable;
    word += _direct * kDirect;
    word += _jog * kJog;
    word += _remote * kRemote;
    word += _ramp2 * kRamp2;
    word += _reset * kReset;
    return word;
}