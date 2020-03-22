/*
 * modbus_slave.h
 *
 *  Created on: 29 июня 2015 г.
 *      Author: andru
 */

#ifndef MODBUS_SLAVE_H_
#define MODBUS_SLAVE_H_

#include "port.h"

/* -----------------------Slave Defines ------------------------------------- */
#define S_DISCRETE_INPUT_START        0
#define S_DISCRETE_INPUT_NDISCRETES   6
#define S_COIL_START                  0
#define S_COIL_NCOILS                 6
#define S_REG_INPUT_START             0
#define S_REG_INPUT_NREGS             5
#define S_REG_HOLDING_START           0
#define S_REG_HOLDING_NREGS           4
/* slave mode: holding register's all address */
#define S_HD_RESERVE                  0
#define S_HD_CPU_USAGE_MAJOR          1
#define S_HD_CPU_USAGE_MINOR          2
/* salve mode: input register's all address */
#define S_IN_RESERVE                  0
/* salve mode: coil's all address */
#define S_CO_RESERVE                  0
/* salve mode: discrete's all address */
#define S_DI_RESERVE                  0

//Slave mode:DiscreteInputs variables
extern USHORT   usSDiscInStart;
extern UCHAR    ucSDiscInBuf[];
//Slave mode:Coils variables
extern USHORT   usSCoilStart;
extern UCHAR    ucSCoilBuf[];
//Slave mode:InputRegister variables
extern USHORT   usSRegInStart;
extern SHORT   usSRegInBuf[];
//Slave mode:HoldingRegister variables
extern USHORT   usSRegHoldStart;
extern SHORT   usSRegHoldBuf[];

#define MB_ADDRESS	32
#define MB_BITRATE	MB_BITRATE_9600
#define MB_PARITY	MB_PARITY_NONE

typedef enum {
	OFF = 0,
	ON,
} MB_OFFON_T;

typedef enum {
	MB_DI_IN1,		// external IN1 value
	MB_DI_OUT1,		// relay 1 on/off
	MB_DI_IN2,		// external IN2 value
	MB_DI_OUT2,		// relay 2 on/off
	MB_DI_FLASH,	// eeprom read error
	MB_DI_TEMP,		// temperature read error
} MB_DISCRETE_MAP;

typedef enum {
	MB_CO_IN1,		// IN1 check flag
	MB_CO_OUT1,		// set OUT1
	MB_CO_IN2,		// IN2 check flag
	MB_CO_OUT2,		// set OUT2
	MB_CO_MBADDR,	// set modbus address flag
	MB_CO_MBCONF,	// set modbus parameters flag
} MB_COIL_MAP;

typedef enum {
	MB_IN_FIRMWARE,		// firmware version
	MB_IN_COUNTER,		// dummy counter
	MB_IN_TEMPERATURE,	// stm32 temperature
	MB_IN_TIMER1,		// timer on ch1 value
	MB_IN_TIMER2,		// timer on ch2 value
} MB_INPUT_MAP;

typedef enum {
	MB_HO_MBADDR,	// modbus address
	MB_HO_MBCONF,	// bitrate & parity
	MB_HO_TIMER1,	// timer ON channel 1
	MB_HO_TIMER2,	// timer ON channel 2
} MB_HOLDING_MAP;

typedef enum {
	MB_BITRATE_1200	= 1,
	MB_BITRATE_2400,
	MB_BITRATE_4800,
	MB_BITRATE_9600,
	MB_BITRATE_19200,
	MB_BITRATE_38400,
	MB_BITRATE_57600,
	MB_BITRATE_115200,
} MB_BITRATE_MAP;

typedef enum {
	MB_PARITY_NONE = 0,
	MB_PARITY_EVEN,
	MB_PARITY_ODD,
} MB_PARITY_MAP;

// create & start modbus poll process
void modbus_init(void);
void setDiscBit(uint16_t regAddr, uint8_t ucValue);
uint8_t getCoilBit(uint16_t regAddr);
void setCoilBit(uint16_t regAddr, uint8_t ucValue);
void writeInputReg(uint16_t regAddr, uint16_t regValue);
USHORT readHoldingReg(uint16_t regAddr);
void writeHoldingReg(uint16_t regAddr, USHORT regValue);

#endif /* MODBUS_SLAVE_H_ */
