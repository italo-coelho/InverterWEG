#ifndef INVERTER_WEG_H
#define INVERTER_WEG_H

#include <Arduino.h>
#include <ModbusMaster.h>

/*
    \brief Motor Specifications
    \param efficiency [50.0, 99,9] (%)
    \param voltage [0, 240] (V)
    \param current [0.0, 10,0] (A)
    \param speed [0, 24000] (rpm)
    \param frequency [0, 400] (Hz)
    \param power [0, 7] (Index - according to VFD Model)
    \param pw_factor [0.50, 0.99] (PF)
*/
struct MotorSpecs
{
    double efficiency;
    unsigned int voltage;
    double current;
    unsigned int speed;
    unsigned int frequency;
    unsigned int power;
    double pw_factor;
};

enum CFW100 {HP1_6, HP1_4, HP_3, HP1_2, HP3_4, HP1, HP3_2, HP2};

class InverterWEG
{
    private:
        Stream* _serial;
        ModbusMaster* _modbus;
        
        uint8_t _address;
        uint16_t _cmd_word;

        bool _spin;
        bool _enable;
        bool _direct;
        bool _jog;
        bool _remote;
        bool _ramp2;
        bool _reset;

        //WEG Control Word Definitions
        static const uint16_t kSpinW    =        0b1;
        static const uint16_t kEnableW  =       0b10;
        static const uint16_t kDirectW  =      0b100;
        static const uint16_t kJogW     =     0b1000;
        static const uint16_t kRemoteW  =    0b10000;
        static const uint16_t kRamp2W   =   0b100000;
        static const uint16_t kResetW   = 0b10000000;

        //WEG Parameters
        static const uint16_t kCmdP         = 682;
        static const uint16_t kRefSpeedP    = 683;
        static const uint16_t kMotorSpeedP  =   2;
        static const uint16_t kCurrentP     =   3;
        static const uint16_t kInvTempP     =  30;
        static const uint16_t kAccel1P      = 100;
        static const uint16_t kDecel1P      = 101;
        static const uint16_t kAccel2P      = 102;
        static const uint16_t kDecel2P      = 103;
        static const uint16_t kEfficiencyRP = 399;
        static const uint16_t kVoltageRP    = 400;
        static const uint16_t kCurrentRP    = 401;
        static const uint16_t kSpeedRP      = 402;
        static const uint16_t kFrequencyRP  = 403;
        static const uint16_t kPowerRP      = 404;
        static const uint16_t kPwFactorRP   = 407;

        static const double kPowerIndex[8];

        uint16_t CmdWord();

        static uint8_t _rs485_de;
        static uint8_t _rs485_re;
        static void preTX()
        {
            digitalWrite(_rs485_de, HIGH);
            digitalWrite(_rs485_re, HIGH);
        }
        static void postTX() 
        {
            digitalWrite(_rs485_de, LOW);
            digitalWrite(_rs485_re, LOW);
        }

        uint32_t _then;
        uint32_t _interval;

        uint32_t time();

        uint8_t readHReg(uint16_t address, uint16_t qtd);
        uint8_t writeHReg(uint16_t address, uint16_t value);
        uint8_t writeMultiReg(uint16_t address, uint16_t qtd);

    public:
        

        InverterWEG();
        ~InverterWEG();
        
        void begin(ModbusMaster &modbus);
        void begin(uint8_t address, Stream &serial);
        void begin(uint8_t address, Stream &serial, int rs485_de_re);
        void begin(uint8_t address, Stream &serial, int rs485_de, int rs485_re);

        bool enable();
        bool disable();

        bool start();
        bool stop();

        bool setSpeed(double speed);
        bool direction(bool direct);
        
        bool selectRamp(uint8_t ramp);
        bool configRamp1(double accelTime, double decelTime);
        bool configRamp2(double accelTime, double decelTime);

        bool configMotor(MotorSpecs motor);
        
        bool resetFaults();

        double current();
        double motorSpeed();
        double temperature();

        void setTxInterval(uint32_t interval);

        

};

#endif //INVERTER_WEG_H