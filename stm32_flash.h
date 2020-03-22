/*
 * stm32_flash.h
 *
 *  Created on: 29.05.2019
 *      Author: andru
 */

#ifndef STM32_FLASH_H_
#define STM32_FLASH_H_

typedef enum {
	PAGE_UNDEF,
	PAGE_ERASED,
	PAGE_RECEIVED,
	PAGE_VALID,
} page_state_t;

#define NUMPAGES	2

#define MAGIC		0x68AE	// eeprom magic data

#include "main.h"

#define HEADERLEN	4
#define DATALEN		CFGLEN
#define RECORDLEN	(HEADERLEN + DATALEN + 1)

typedef struct _flash_page_t flash_page_t;
struct _flash_page_t {
  uint16_t recordid;
  uint16_t last_record;
  uint8_t pageno;
  page_state_t state;
};

typedef struct _flash_record_t flash_record_t;
struct _flash_record_t {
  uint16_t magic;
  uint16_t id;
  uint8_t data[DATALEN];
  uint8_t crc;
};

typedef struct _flash_info_t flash_info_t;
struct _flash_info_t {
  uint16_t records_on_page;
  uint16_t last_id;
  flash_page_t pages[NUMPAGES];
  uint8_t active_page;
};

bool initFlashInfo(void);
bool flashErase(void);
bool readFlash(uint8_t *data);
bool writeFlash(uint8_t *data);

#endif /* STM32_FLASH_H_ */
