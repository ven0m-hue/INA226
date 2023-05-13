# INA226
ANA226 is a shunt based current sensor which measures current theoretically up to 40A.

The sensor allows accurate and precise measurement of electrical current in a circuit. It is a high-side current shunt and voltage monitor that operates over a wide voltage range. The sensor integrates a shunt resistor, a voltage amplifier, and an analog-to-digital converter (ADC) into a single package.


This repository contains a driver file written for the INA266 current sensor. The driver provides functions to initialize and calibrate the INA266, as well as core functions to read bus voltage, shunt voltage, current, and power measurements from the sensor.

## Key APIs:

<b> INA226_Init </b>: Initializes the INA226 current sensor.

INA226_Calibrate: Calibrates the INA226 current sensor.

## Core Functions:

INA226_ReadBusVoltage: Reads the bus voltage from the INA226 sensor.

INA226_ReadShuntVoltage: Reads the shunt voltage from the INA226 sensor.

INA226_ReadCurrent: Reads the current measurement from the INA226 sensor.

INA226_ReadPower: Reads the power measurement from the INA226 sensor.

## Helper Functions:

INA226_writeByte: Writes a byte of data to a specific sub-address of the INA226 sensor via I2C communication.

INA226_readByte: Reads a byte of data from a specific sub-address of the INA226 sensor via I2C communication.

INA226_writeMem: Writes multiple bytes of data to consecutive sub-addresses of the INA226 sensor via I2C communication.

INA226_readMem: Reads multiple bytes of data from consecutive sub-addresses of the INA226 sensor via I2C communication.

INA226_writeMemIT: Writes a byte of data to a specific sub-address of the INA226 sensor using interrupt-based I2C communication.

INA226_readMemIT: Reads a byte of data from a specific sub-address of the INA226 sensor using interrupt-based I2C communication.

These APIs provide a convenient interface for interacting with the INA266 current sensor, enabling users to easily configure, calibrate, and read measurements from the sensor in their projects.
