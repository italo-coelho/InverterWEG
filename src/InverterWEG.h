#ifndef INVERTER_WEG_H
#define INVERTER_WEG_H

#include <Arduino.h>
#include <ModbusMaster.h>

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
        static const uint16_t kSpin = 0b1;
        static const uint16_t kEnable = 0b10;
        static const uint16_t kDirect = 0b100;
        static const uint16_t kJog = 0b1000;
        static const uint16_t kRemote = 0b10000;
        static const uint16_t kRamp2 = 0b100000;
        static const uint16_t kReset = 0b10000000;

        //WEG Parameters
        static const uint16_t kCmdP = 682;
        static const uint16_t kRefSpeedP = 683;
        static const uint16_t kMotorSpeedP = 2;
        static const uint16_t kCurrentP = 3;
        static const uint16_t kInvTempP = 30;
        static const uint16_t kAccel1P = 100;
        static const uint16_t kDecel1P = 101;
        static const uint16_t kAccel2P = 102;
        static const uint16_t kDecel2P = 103;


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

        uint32_t _interval;

        uint8_t readHReg(uint16_t address, uint16_t qtd);
        uint8_t writeHReg(uint16_t address, uint16_t value);
        uint8_t writeMultiReg(uint16_t address, uint16_t qtd);
#ifdef ESP32
        TickType_t _then;
#else //!ESP32
        uint32_t _then;
#endif //ESP32

    public:
        InverterWEG();
        ~InverterWEG();
        
        void begin(ModbusMaster &modbus);
        void begin(uint8_t address, Stream &serial);
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
        
        bool resetFaults();

        double current();
        double motorSpeed();
        double temperature();

        void setTxInterval(uint32_t interval);

};

#endif //INVERTER_WEG_H