#ifndef DEFINITIONS_WEG_H
#define DEFINITIONS_WEG_H

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