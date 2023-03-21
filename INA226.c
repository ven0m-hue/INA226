/*
 * INA226.c
 *
 *  Created on: Mar 15, 2023
 *      Author: Redwing
 */

#include "INA226.h"


/*
 *  API Calls
 *
 *  1. Config
 *  2. Core
 *  3. Helper
 * 
 */

/*
 * Config Functions 
 */
void INA226_Init(INA226_Handle_t* hINA266)
{

	uint8_t raw[2] = {0};

	INA226_OpStatus_t ret = INA226_ERROR;

    // Read hINA266ice ID to verify communication
	readMem(hINA266->I2Chandle, INA226_ADDRESS, INA226_REG_ID, raw);
	uint16_t id =  ((uint16_t)((uint16_t)raw[0] << 8 | raw[1]));
    if (id != INA226_REG_ID) {
        // Communication error - handle appropriately
        return;
    }

    // Reset hINA266ice configuration to default values
	uint16_t config = 0;

    //INA226_WriteRegister(hINA266, INA226_REG_CONFIG, INA226_CONFIG_RST);
	config = INA226_CONFIG_RST;
	raw[0] = (config & 0xff00) >> 8;  //MSB Byte --> sent first
	raw[1] = (config & 0x00ff);       //LSB Byte
	writeByte(hINA266->I2Chandle, INA226_ADDRESS, INA226_REG_CONFIG, raw);

    // Set configuration register
    config |= INA226_CONFIG_DEF | INA226_CONFIG_AVG | INA226_CONFIG_VBUS | INA226_CONFIG_VSHC | INA226_CONFIG_MODE;
	raw[0] = (config & 0xff00) >> 8;  //MSB Byte --> sent first
	raw[1] = (config & 0x00ff);       //LSB Byte
	writeByte(hINA266, INA226_ADDRESS, INA226_REG_CONFIG, raw);

    // Set shunt resistor value and calibration factor
    hINA266->shuntResistor = INA226_DEFAULT_SHUNT_RESISTOR;
    hINA266->calibrationFactor = INA226_DEFAULT_CALIBRATION_FACTOR;
}




uint8_t INA226_Calibrate(INA226_Handle_t* hINA266)
{
	/*
	 *  Below calculations are based on the formulas provided in the datasheet
	 *  For more info refer @datasheet pg 
	 */

	INA226_OpStatus_t ret = INA226_ERROR;

	uint8_t raw[2] = {0};

	float maxCurrent = hINA266->expectedCurr;
	float shunt = hINA266->shuntResistor;
	float shuntVoltage = abs( maxCurrent * shunt);  
	if (shuntVoltage > 0.080) return INA226_ERR_SHUNTVOLTAGE_HIGH;
	if (maxCurrent < 0.001)   return INA226_ERR_MAXCURRENT_LOW;
	if (shunt < 0.001)        return INA226_ERR_SHUNT_LOW;

	hINA266->current_LSB = maxCurrent * 3.0517578125e-5;    //(maxCurrent / 32768)

	uint32_t calib  = 0;
	uint32_t factor = 1;

	_Bool normalize = true;

	//  normalize the LSB to a round number
	//  LSB will increase
	if (normalize)
	{
		calib = round(0.00512 / (hINA266->current_LSB * shunt));
		hINA266->current_LSB = 0.00512 / (calib * shunt);

		//  auto scale current_LSB
		factor = 1;
		while (hINA266->current_LSB < 1)
		{
		hINA266->current_LSB *= 10;
		factor *= 10;
		}
		hINA266->current_LSB = 1.0 / factor;
	}

	//  auto scale calibration
	calib = round(0.00512 / (hINA266->current_LSB * shunt));
	while (calib > 65535)
	{
		hINA266->current_LSB *= 10;
		calib /= 10;
	}

	/*
	 *	Write to the I2C register
	 *  Explicitly masked as 0x0000ff00 and 0x000000ff
	 * 	However just 0xff00 and 0x00ff works
	 */ 
	
	raw[0] = (calib & 0x0000ff00) >> 8;  //MSB Byte --> sent first
	raw[1] = (calib & 0x000000ff);       //LSB Byte
	writeByte(hINA266, INA226_ADDRESS, INA226_REG_CONFIG, raw);

	maxCurrent = hINA266->current_LSB * 32768;
	hINA266->expectedCurr = maxCurrent;

	ret = INA226_SUCESS;

	return ret;
}

/*
 * Core Functions 
 */
float INA226_ReadBusVoltage(INA226_Handle_t* hINA266);
float INA226_ReadShuntVoltage(INA226_Handle_t* hINA266);

float INA226_ReadCurrent(INA226_Handle_t* hINA266)
{
	INA226_OpStatus_t ret = INA226_ERROR;

	uint8_t raw[2] = {0};

	if((hINA266->I2Chandle == NULL)) return ret;

	if(readMem(hINA266->I2Chandle, INA226_ADDRESS, INA226_REG_CURRENT, raw) != INA226_SUCESS );

	else ret = INA226_SUCESS;

	hINA266->current = ((uint16_t)((uint16_t)raw[0] << 8 | raw[1]));

	return ret;
}



float INA226_ReadPower(INA226_Handle_t* hINA266);


uint8_t writeByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data)
{

	 uint8_t txData[] = {subAddress, data};
	 if(HAL_I2C_Master_Transmit(I2Chandle, Address, txData, 2, AS5600_I2C_TIMEOUT) != HAL_ERROR)
		 return 1;

	 else
		 return 0;

}

uint8_t readByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress)
{
	uint8_t rxData[1] = {0};
	uint8_t txData[] = {subAddress};
	HAL_I2C_Master_Transmit(I2Chandle, Address, txData, 1, AS5600_I2C_TIMEOUT);

	if(HAL_I2C_Master_Receive(I2Chandle, Address, rxData, 1, AS5600_I2C_TIMEOUT) != HAL_ERROR) return rxData[0];

	else return 0;

}

/*
 * Writes 2 bytes of data.
 * This API is non-generic and blocking mode.
 */

uint8_t writeMem(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* data)
{

	if(HAL_I2C_Mem_Write(I2Chandle, Address, subAddress, I2C_MEMADD_SIZE_8BIT, data, 2, INA226_I2C_TIMEOUT) != HAL_ERROR);

	return 1;
}

/*
 * Reads bytes of data
 */

uint8_t readMem(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* raw)
{

	uint8_t rawData[2];

	if(HAL_I2C_Mem_Read(I2Chandle, Address, subAddress, I2C_MEMADD_SIZE_8BIT, rawData, 2, INA226_I2C_TIMEOUT) != HAL_ERROR)
	{
		raw[0] = rawData[0];
		raw[1] = rawData[1];

		return 1;
	}

	else return 0;

}


/*
 * Writes 2 bytes of data.
 * This API is non-generic and non-blocking mode.
 * User can have their own interface for reading the data in a seperate callback function.
 */
uint8_t writeMemIT(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data)
{


	return 0;

}


/*
 * Reads bytes of data
 * This API is non-generic and non-blocking mode.
 * User can have their own interface for reading the data in a seperate callback function.
 */
uint8_t readMemIT(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* raw)
{

	uint8_t rawData[2];

	if(HAL_I2C_Mem_Read_IT(I2Chandle, Address, subAddress, I2C_MEMADD_SIZE_8BIT, rawData, 2) != HAL_ERROR)
	{
		raw[0] = rawData[0];
		raw[1] = rawData[1];

		return 1;
	}

	else return 0;

}