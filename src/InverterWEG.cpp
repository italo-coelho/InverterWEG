#include "InverterWEG.h"

uint8_t InverterWEG::_rs485_de = -1;
uint8_t InverterWEG::_rs485_re = -1;

InverterWEG::InverterWEG()
{
    _spin   = false;
    _enable = false;
    _direct =  true;
    _jog    = false;
    _remote = false;
    _ramp2  = false;
    _reset  = false;

    _interval = 3;
}

InverterWEG::~InverterWEG()
{
    if(_modbus)
        delete _modbus;
}

/*
    \brief Use this method when the control of DE and RE pins of the Transceiver is not required
    \param address Inverter Modbus address [1, 255]
    \param serial Serial interface connected to the Transceiver
*/
void InverterWEG::begin(uint8_t address, Stream &serial)
{
    _rs485_de = -1;
    _rs485_re = -1;
    if(_modbus)
        delete _modbus;
    _modbus = new ModbusMaster;
    _modbus->begin(address, serial);
}

/*
    \brief Use this method when the control of DE and RE pins of the Transceiver are connected to a single pin of the MCU
    \param address Inverter Modbus address [1, 255]
    \param serial Serial interface connected to the Transceiver
    \param rs486_de_re MCU pin connected to transceiver's RE and DE pins
*/
void InverterWEG::begin(uint8_t address, Stream &serial, int rs485_de_re)
{
    if(_modbus)
        delete _modbus;
    _modbus = new ModbusMaster;
    _modbus->begin(address, serial);
    _rs485_de = rs485_de_re;
    _rs485_re = rs485_de_re;
    pinMode(_rs485_de, OUTPUT);
    _modbus->preTransmission(preTX);
    _modbus->postTransmission(postTX);
}

/*
    \brief Use this method when the control of DE and RE pins of the Transceiver are connected to two individual pins of the MCU
    \param address Inverter Modbus address [1, 255]
    \param serial Serial interface connected to the Transceiver
    \param rs486_de MCU pin connected to transceiver's DE pin
    \param rs486_re MCU pin connected to transceiver's RE pin
*/
void InverterWEG::begin(uint8_t address, Stream &serial, int rs485_de, int rs485_re)
{
    if(_modbus)
        delete _modbus;
    _modbus = new ModbusMaster;
    _modbus->begin(address, serial);
    _rs485_de = rs485_de;
    _rs485_re = rs485_re;
    pinMode(_rs485_de, OUTPUT);
    pinMode(_rs485_re, OUTPUT);
    _modbus->preTransmission(preTX);
    _modbus->postTransmission(postTX);
}

/*
    \brief Use this method to pass an Modbus Object responsible for the communication
    \param modbus Object from ModbusMaster class
*/
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
    \brief Setup Inverter with Motor Parameters
    \param motor Struct with motor specifications
    \return Communication Success
*/
bool InverterWEG::configMotor(MotorSpecs motor)
{
    _modbus->setTransmitBuffer(0, motor.efficiency * 10);
    _modbus->setTransmitBuffer(1, motor.voltage);
    _modbus->setTransmitBuffer(2, motor.current * 10);
    _modbus->setTransmitBuffer(3, motor.speed);
    _modbus->setTransmitBuffer(4, motor.frequency);
    _modbus->setTransmitBuffer(5, motor.power);

    uint8_t result = writeMultiReg(kEfficiencyRP, 6);
    if(result != _modbus->ku8MBSuccess)
        return false;
    result = writeHReg(kPwFactorRP, motor.pw_factor * 100);
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
    uint32_t elapsed = elapsed = time() - _then;
    if(elapsed < _interval)
        delay(_interval - elapsed);

    uint8_t result = _modbus->readHoldingRegisters(address, qtd);
    
    _then = time();
    
    return result;
}

uint8_t InverterWEG::writeHReg(uint16_t address, uint16_t value)
{
    uint32_t elapsed = elapsed = time() - _then;
    if(elapsed < _interval)
        delay(_interval - elapsed);

    uint8_t result = _modbus->writeSingleRegister(address, value);
    
    _then = time();

    return result;
}

uint8_t InverterWEG::writeMultiReg(uint16_t address, uint16_t qtd)
{
    uint32_t elapsed = elapsed = time() - _then;
    if(elapsed < _interval)
        delay(_interval - elapsed);

    if(elapsed < _interval)
        delay(_interval - elapsed);

    uint8_t result = _modbus->writeMultipleRegisters(address, qtd);
    
    _then = time();
    
    return result;
}

uint32_t InverterWEG::time()
{
#ifdef ESP32
        return pdTICKS_TO_MS(xTaskGetTickCount());
#else //!ESP32
        return millis();
#endif //ESP32
}

uint16_t InverterWEG::CmdWord()
{
    uint16_t word = 0;
    word += _spin   * kSpinW;
    word += _enable * kEnableW;
    word += _direct * kDirectW;
    word += _jog    * kJogW;
    word += _remote * kRemoteW;
    word += _ramp2  * kRamp2W;
    word += _reset  * kResetW;
    return word;
}