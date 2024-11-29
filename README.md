# InverterWEG

This library was developed to interface an ESP32 with a WEG VFD via ModbusRTU.

It is known to work with ESP32 and ESP8266 microcontrollers. If you can successfully use a different one please let me know!

It was tested with a CFW100 inverter but the parameters were also defined using the CFW300 and CFW500 user manuals, and should work with them as well.

## Functionality

Most of the necessary registers are implemented in easy to use methods but any register of the inverter can be read/written mannually.

Note that the user friendly methods are based on the CFW-xxx line of products, and their parameters can be checked on 'definitions.h' file.

The parameters with real values will be read/written with the same number of decimal places as shown in the inverter HMI. When using the available methods this should be transparent to the programmer. 
When using manual read/write make sure to multiply/divide the number according to the number of decimal places shown in VFD screen, as Modbus only supports integers and the driver will always perform the opposite operation when receiving the data.

There is a structure for easy setup of motor specifications and a method for automatic setup of the inverter for full remote control.

## Notes 

Example code is currently not available but all methods and parameters have briefs.
