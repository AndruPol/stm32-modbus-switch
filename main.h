/*
 * main.h
 *
 *  Created on: 24.05.2019
 *      Author: andru
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "ch.h"

#include "modbus_slave.h"

#define FIRMWARE		104		// версия прошивки
#define FLASHERASE		0		// if config size changed set first run to 1

#define DEBUG			0

typedef enum {
	EVT_TEST	= (1 << 0),
	EVT_IN1		= (1 << 1),
	EVT_IN2		= (1 << 2),
	EVT_MBCOIL	= (1 << 3),
	EVT_MBHOLD	= (1 << 4),
} EVT_MASK_t;

// eeprom config length
#define CFGLEN		sizeof(CONFIG_T)

// eeprom configuration format
typedef struct CONFIG CONFIG_T;
struct CONFIG {
	uint8_t mb_addr;			// modbus address
	MB_BITRATE_MAP mb_bitrate;	// modbus bitrate enum
	MB_PARITY_MAP mb_parity;	// modbus parity enum
	bool in1_on;				// input 1 check enabled
	bool in2_on;				// input 2 check enabled
};

#define THD_GOOD	0b1111
#define THD_INIT	0

typedef enum {
	THD_MAIN	= (1 << 0),
	THD_EVENT	= (1 << 1),
	THD_SWITCH	= (1 << 2),
	THD_MODBUS  = (1 << 3),
} thd_check_t;

extern volatile thd_check_t thd_state;
extern event_source_t event_src;
extern CONFIG_T config;

#endif /* MAIN_H_ */
