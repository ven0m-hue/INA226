/*
 * INA226.h
 *
 *  Created on: Mar 15, 2023
 *      Author: Venom
 */


#ifndef INC_INA226_H_
#define INC_INA226_H_

#include "main.h"

/*
 * Structure Handler
 */

typedef struct
 {
    //Input
    I2C_HandleTypeDef *I2Chandle;

    float shuntResistor;

    float calibrationFactor;

    float current_LSB;

    float power_LSB;

    float expectedCurr;

    //Output 
    float current;

    float shuntVoltage;

    float busVoltage;

    float power;


    //Limits
    float shuntVoltageMax;
    float BustVoltageMax;


} INA226_Handle_t;


typedef enum
{
    INA226_ERROR = 0,
    INA226_SUCESS
}INA226_OpStatus_t;

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
uint8_t INA226_Init(INA226_Handle_t* hINA266);
uint8_t INA226_Calibrate(INA226_Handle_t* hINA266);

/*
 * Core Functions 
 */
float INA226_ReadBusVoltage(INA226_Handle_t* hINA266);
float INA226_ReadShuntVoltage(INA226_Handle_t* hINA266);

float INA226_ReadCurrent(INA226_Handle_t* hINA266);
float INA226_ReadPower(INA226_Handle_t* hINA266);


/*
 * Helper Functions
 */
static uint8_t INA226_writeByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data);
static uint8_t INA226_readByte(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress);

static uint8_t INA226_writeMem(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* data);
static uint8_t INA226_readMem(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* raw);

static uint8_t INA226_writeMemIT(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t data);
static uint8_t INA226_readMemIT(I2C_HandleTypeDef *I2Chandle, uint8_t Address, uint8_t subAddress, uint8_t* raw);



/*
 *  Register Definations
 */

// INA226 I2C address
#define INA226_ADDRESS      (0x40<<1)

// INA226 register addresses
#define INA226_REG_CONFIG   0x00
#define INA226_REG_SHUNT    0x01
#define INA226_REG_BUS      0x02
#define INA226_REG_POWER    0x03
#define INA226_REG_CURRENT  0x04
#define INA226_REG_CALIB    0x05
#define INA226_REG_MASK     0x06
#define INA226_REG_ALERT    0x07
#define INA226_REG_ID       0xFE
#define INA226_DIE_ID       0xFF

// INA225 Read-Only register values
#define INA226_REG_ID_VALUE	0x5549

// INA226 Default Hardware Setting 
#define INA226_DEFAULT_SHUNT_RESISTOR       0.002f
#define INA226_DEFAULT_CALIBRATION_FACTOR   1 
#define INA226_ERR_SHUNTVOLTAGE_HIGH        0
#define INA226_ERR_MAXCURRENT_LOW           0
#define INA226_ERR_SHUNT_LOW                0

// INA226 configuration register bit masks @more info read Datasheet pg.22-23
#define INA226_CONFIG_RST   (1 << 15)
#define INA226_CONFIG_DEF   (1 << 14)
#define INA226_CONFIG_AVG   (1 << 11)
#define INA226_CONFIG_VBUS  (7 << 7)
#define INA226_CONFIG_VSHC  (7 << 3)
#define INA226_CONFIG_MODE  (7 << 0)

// INA226 measurement modes
#define INA226_MODE_POWERDOWN       0
#define INA226_MODE_SHUNT_TRIGGERED 1
#define INA226_MODE_BUS_TRIGGERED   2
#define INA226_MODE_SHUNT_BUS_TRIGGERED 3
#define INA226_MODE_ADC_OFF         4
#define INA226_MODE_SHUNT_CONTINUOUS 5
#define INA226_MODE_BUS_CONTINUOUS   6
#define INA226_MODE_SHUNT_BUS_CONTINUOUS 7

#define INA226_I2C_TIMEOUT	100
#endif /* INC_INA226_H_ */
