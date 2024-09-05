#ifndef DEFINITIONS_WEG_H
#define DEFINITIONS_WEG_H

#define INVERTER_KEEPALIVE 1000

enum class FanMode {
                      OFF,  //Always OFF
                      ON,   //Always ON
                      AUTO  //According to Temperature
                    };

enum class ControlMode {
                            VF = 0,  //Linear
                            VVW = 5  //Vector
                        };

enum class ControlSource {
                            LOCAL = 0,      //Always Local
                            REMOTE = 1,     //Always Remote
                            DIG_IN = 4,     //Digital Input
                            SERIAL_LOC = 5, //Serial/USB (Local)
                            SERIAL_REM = 6, //Serial/USB (Remote)
                            SOFT_PLC = 11   //SoftPLC
                          };

enum class SpeedSource {
                            HMI = 0,        //HMI Buttons
                            AI1 = 1,        //Analog In1
                            MULTISPEED = 8, //Multispeed
                            SERIAL_USB = 9, //Serial/USB
                            SOFT_PLC = 12   //SoftPLC
                          };

enum class EnableSource {
                            HMI = 0,        //HMI Buttons
                            DIx = 1,        //Digital Input
                            SERIAL_USB = 2, //Serial/USB
                            SOFT_PLC = 5    //SoftPLC
                          };

enum class JogSource    {
                            NONE = 0,       //Inactive
                            DIx = 2,        //Digital Input
                            SERIAL_USB = 3, //Serial/USB
                            SOFT_PLC = 6    //SoftPLC
                          };

enum class InputFn {
                        NONE = 0,
                        START_STOP = 1,
                        ENABLE = 2,
                        E_STOP = 3,
                        FORWARD = 4,
                        BACKWARD = 5,
                        ON = 6,
                        OFF = 7,
                        JOG = 10,
                        MULTISPEED = 13,
                        RAMP_2 = 14,
                        RESET = 20,
                    };

/*
    These values for P404 are known to work with CFW100, CFW300 and CFW500. 
    Do NOT try to set a motor power higher than the one the inverter is rated for.
*/
enum class MotorPW {                      
                        //CFW100      
                        HP1_6,  //0.16 HP
                        HP1_4,  //0.25 HP 
                        HP1_3,  //0.33 HP
                        HP1_2,  //0.50 HP
                        HP3_4,  //0.74 HP 
                        HP1,    //1.00 HP
                        //CFW300
                        HP3_2,  //1.50 HP
                        HP2,    //2.00 HP
                        HP3,    //3.00 HP
                        HP4,    //4.00 HP
                        HP5,    //5.00 HP
                        HP11_2, //5.50 HP
                        HP6,    //6.00 HP
                        HP15_2, //7.50 HP
                        HP10,   //10.0 HP
                        //CFW500
                        HP25_2, //12.5 HP
                        HP15,   //15.0 PH
                        HP20,   //20.0 HP
                        HP25,   //25.0 HP
                        HP30    //30.0 HP
                    };

#endif //DEFINITIONS_WEG_H