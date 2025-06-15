/*
  This file is part of the InverterWEG library.
  Copyright (c) 2024 Ítalo Coelho. All rights reserved.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
 *      _____                     _              __          ________ _____ 
 *     |_   _|                   | |             \ \        / /  ____/ ____|
 *       | |  _ ____   _____ _ __| |_ ___ _ __    \ \  /\  / /| |__ | |  __ 
 *       | | | '_ \ \ / / _ \ '__| __/ _ \ '__|    \ \/  \/ / |  __|| | |_ |
 *      _| |_| | | \ V /  __/ |  | ||  __/ |        \  /\  /  | |___| |__| |
 *     |_____|_| |_|\_/ \___|_|   \__\___|_|         \/  \/   |______\_____|
 *
 *     This library provides a simple way to set the most relevant parameters
 *  on WEG inverters for configuration and operation. The Modbus RTU protocol
 *  is used for communication, and the MCU should be connected to the VFD via
 *  RS485 interface.
 * 
 *  Ítalo Coelho
 *          2024
 */

#ifndef INVERTER_WEG_H
#define INVERTER_WEG_H

#include <Arduino.h>
#include "ModbusMaster.h"

#include "definitions.h"

/*
    \brief Motor Specifications
    \param efficiency [50.0, 99,9] (%)
    \param voltage [0, 240] (V)
    \param current [0.0, 10,0] (A)
    \param speed [0, 24000] (rpm)
    \param frequency [0, 400] (Hz)
    \param power [0, N] (Power Index)
    \param pw_factor [0.50, 0.99] (P.F.)
*/
struct MotorSpecs
{
    double efficiency;
    unsigned int voltage;
    double current;
    unsigned int speed;
    unsigned int frequency;
    MotorPW power;
    double pw_factor;
};


class InverterWEG
{
    private:
        Stream* _serial;
        ModbusMaster* _modbus;
        
        uint8_t _address;
        uint16_t _cmd_word;

        bool _spin   = false;
        bool _enable = false;
        bool _direct =  true;
        bool _jog    = false;
        bool _remote = false;
        bool _ramp2  = false;
        bool _reset  = false;

        //WEG Control Word Definitions
        static const uint16_t kSpinW    =        0b1;
        static const uint16_t kEnableW  =       0b10;
        static const uint16_t kDirectW  =      0b100;
        static const uint16_t kJogW     =     0b1000;
        static const uint16_t kRemoteW  =    0b10000;
        static const uint16_t kRamp2W   =   0b100000;
        static const uint16_t kResetW   = 0b10000000;

        //WEG Parameters
        static const uint16_t kCmdWordP     = 682;
        static const uint16_t kRefSpeedP    = 683;
        static const uint16_t kMotorSpeedP  =   2;
        static const uint16_t kCurrentP     =   3;
        static const uint16_t kInvTempP     =  30;
        static const uint16_t kMinFreq      = 133;
        static const uint16_t kMaxFreq      = 134;
        static const uint16_t kAccel1P      = 100;
        static const uint16_t kDecel1P      = 101;
        static const uint16_t kAccel2P      = 102;
        static const uint16_t kDecel2P      = 103;
        static const uint16_t kCntrlMode    = 202;
        static const uint16_t kCntrlSource  = 220;
        static const uint16_t kSpeedSource  = 222;
        static const uint16_t kEnableSource = 227;
        static const uint16_t kJogSource    = 228;
        static const uint16_t kDigIn1Mode   = 263;
        static const uint16_t kFanControl   = 352;
        //WEG Motor Specs Parameters
        static const uint16_t kEfficiencyRP = 399;
        static const uint16_t kVoltageRP    = 400;
        static const uint16_t kCurrentRP    = 401;
        static const uint16_t kSpeedRP      = 402;
        static const uint16_t kFrequencyRP  = 403;
        static const uint16_t kPowerRP      = 404;
        static const uint16_t kPwFactorRP   = 407;

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
        uint32_t _interval = 3;

        uint8_t _result = _modbus->ku8MBResponseTimedOut;

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

        bool connected();

        bool enable();
        bool disable();

        bool start();
        bool stop();
        
        bool setJog(bool jog);
        bool setSpeed(double speed);
        bool setDirection(bool direct);
        
        bool setMinFreq(double min_f);
        bool setMaxFreq(double max_f);
        bool selectRamp(uint8_t ramp);
        bool configRamp1(double accelTime, double decelTime);
        bool configRamp2(double accelTime, double decelTime);

        bool configMotor(MotorSpecs motor);

        bool setControlMode(ControlMode mode);
        bool setInputFunction(uint8_t input, InputFn function);

        bool setJogSource(JogSource source);
        bool setSpeedSource(SpeedSource source);
        bool setEnableSource(EnableSource source);
        bool setControlSource(ControlSource source);

        bool setupModbusRemote();

        uint16_t readParam(uint16_t param);
        uint8_t writeParam(uint16_t param, uint16_t value);
        
        bool resetFaults();

        bool setFan(FanMode mode);
        
        double current();
        double motorSpeed();
        double temperature();

        void setTxInterval(uint32_t interval);
};

#endif //INVERTER_WEG_H