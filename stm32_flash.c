/*
 * stm32_flash.c
 *
 *  Created on: 29.05.2019
 *      Author: andru
 */

#include <stdint.h>
#include <string.h>

#include "ch.h"
#include "hal.h"

#include "stm32_flash.h"
#include "hal_flash_lld.h"
#include "crc8.h"

static flash_info_t flashInfo;
static uint16_t record_len;

// erase flash page
static bool pageErase(uint8_t pageno) {
  BaseFlash *fp = getBaseFlash(&FD1);

  if (fp->state == FLASH_READ)
	fp->state = FLASH_READY;

  if ((flashStartEraseSector(fp, pageno) == FLASH_NO_ERROR) &&
	  (flashWaitErase(fp) == FLASH_NO_ERROR)) {
	return true;
  }
  return false;
}

// erase all flash pages
bool flashErase(void) {
  for (uint16_t i=0; i < NUMPAGES; i++) {
	if (!pageErase(flashInfo.pages[i].pageno))
	  return false;
  }
  return true;
}

// verify erased record on flash
static bool verifyRecord(uint8_t page, uint16_t record) {
  BaseFlash *fp = getBaseFlash(&FD1);
  const flash_descriptor_t *descriptor = flashGetDescriptor(fp);
  uint8_t rec[record_len];
  uint32_t offset  = flashInfo.pages[page].pageno * descriptor->page_size + record * record_len;
  if (flashRead(fp, offset, record_len, rec) != FLASH_NO_ERROR) {
	return false;
  }
  for (uint16_t i=0; i < record_len; i++) {
	if (rec[i] != 0xFF) return false;
  }
  return true;
}

// init flash info structure
bool initFlashInfo(void) {
  BaseFlash *fp = getBaseFlash(&FD1);
  const flash_descriptor_t *descriptor = flashGetDescriptor(fp);

  record_len = (RECORDLEN % 4 > 0) ? (RECORDLEN / 4 * 4 + 4) : RECORDLEN;

  flashInfo.last_id = 0;
  flashInfo.records_on_page = descriptor->page_size / record_len;

  for (uint8_t i=NUMPAGES; i > 0; i--) {
	  flashInfo.pages[NUMPAGES-i].pageno = descriptor->sectors_count - i;
	  flashInfo.pages[NUMPAGES-i].state = PAGE_UNDEF;
  }

  uint8_t rec[RECORDLEN];
  for (uint8_t i=0; i < NUMPAGES; i++) {
	for (uint16_t j=0; j < flashInfo.records_on_page; j++) {
	  uint32_t offset  = flashInfo.pages[i].pageno * descriptor->page_size + j * record_len;
	  if (flashRead(fp, offset, record_len, rec) != FLASH_NO_ERROR) {
		return false;
	  }
	  flash_record_t *recp = (flash_record_t *) rec;
	  if (!(recp->magic == MAGIC && recp->crc == CRC8(rec, RECORDLEN-1))) {
		if (j == 0) {
		  // no records on page
		  if (flashVerifyErase(fp, flashInfo.pages[i].pageno) == FLASH_NO_ERROR) {
			flashInfo.pages[i].state = PAGE_ERASED;
			flashInfo.pages[i].last_record = 0;
			flashInfo.pages[i].recordid = 0;
			break;
		  } else {
			if (pageErase(flashInfo.pages[i].pageno)) {
				flashInfo.pages[i].state = PAGE_ERASED;
				flashInfo.pages[i].last_record = 0;
				flashInfo.pages[i].recordid = 0;
				break;
			} else {
			  return false;
			}
		  }
		}
		break;
	  } else {
		flashInfo.pages[i].state = PAGE_VALID;
		flashInfo.pages[i].last_record = j;
		flashInfo.pages[i].recordid = recp->id;
		if (flashInfo.last_id < recp->id) {
		  flashInfo.active_page = i;
		  flashInfo.last_id = recp->id;
		}
	  }
	}
  }
  if (flashInfo.last_id > 0) {
	flashInfo.pages[flashInfo.active_page].state = PAGE_RECEIVED;
	return true;
  }
  for (uint8_t i=0; i<NUMPAGES; i++) {
	if (flashInfo.pages[i].state == PAGE_ERASED) {
	  flashInfo.pages[i].state = PAGE_RECEIVED;
	  flashInfo.active_page = i;
	  return true;
	}
  }
  return false;
}

// read data from flash record
bool readFlash(uint8_t *data) {
  BaseFlash *fp = getBaseFlash(&FD1);
  const flash_descriptor_t *descriptor = flashGetDescriptor(fp);
  uint8_t page = flashInfo.active_page;
  if (flashInfo.pages[page].state != PAGE_RECEIVED) return false;

  uint8_t rec[record_len];
  uint32_t offset = flashInfo.pages[page].pageno * descriptor->page_size +
					flashInfo.pages[page].last_record * record_len;
  if (flashRead(fp, offset, record_len, rec) != FLASH_NO_ERROR) {
	return false;
  }
  flash_record_t *recp = (flash_record_t *) rec;
  if (!(recp->magic == MAGIC && recp->crc == CRC8(rec, RECORDLEN-1))) {
	return false;
  }
  memcpy(data, recp->data, DATALEN);
  return true;
}

// write data to record on flash
bool writeFlash(uint8_t *data) {
  BaseFlash *fp = getBaseFlash(&FD1);
  const flash_descriptor_t *descriptor = flashGetDescriptor(fp);
  uint8_t last_page = flashInfo.active_page;
  if (flashInfo.pages[last_page].state != PAGE_RECEIVED) return false;

  uint8_t next_page;
  uint16_t next_rec;
  uint16_t next_id = flashInfo.last_id + 1;
  bool found = false;
  for (uint8_t i=0; i<NUMPAGES; i++) {
	if (flashInfo.pages[i].state == PAGE_VALID) {
	  next_page = i;
	  next_rec = flashInfo.pages[i].last_record + 1;
	  if (next_rec < flashInfo.records_on_page) {
		found = true;
		break;
	  }
	  if (pageErase(flashInfo.pages[next_page].pageno)) {
		found = true;
		flashInfo.pages[next_page].state = PAGE_ERASED;
		flashInfo.pages[next_page].last_record = 0;
		flashInfo.pages[next_page].recordid = 0;
		next_rec = 0;
		break;
	  }
	}
  }
  if (!found) {
	for (uint8_t i=0; i<NUMPAGES; i++) {
	  if (flashInfo.pages[i].state == PAGE_ERASED) {
		next_page = i;
		next_rec = 0;
		found = true;
		break;
	  }
	}
  }
  if (!found) return false;

  if (!verifyRecord(next_page, next_rec)) {
	if (!pageErase(flashInfo.pages[next_page].pageno))
	  return false;
	flashInfo.pages[next_page].state = PAGE_ERASED;
	flashInfo.pages[next_page].last_record = 0;
	flashInfo.pages[next_page].recordid = 0;
	next_rec = 0;
  }

  flash_record_t rec;
  rec.id = next_id;
  rec.magic = MAGIC;
  memcpy(rec.data, data, DATALEN);
  uint8_t crc = CRC8((uint8_t *) &rec, RECORDLEN-1);
  rec.crc = crc;
  uint32_t offset = flashInfo.pages[next_page].pageno * descriptor->page_size +
		  	  	  	next_rec * record_len;
  if (flashProgram(fp, offset, record_len, (uint8_t *) &rec) == FLASH_NO_ERROR) {
	flashInfo.pages[last_page].state = PAGE_VALID;
	flashInfo.pages[next_page].state = PAGE_RECEIVED;
	flashInfo.pages[next_page].recordid = next_id;
	flashInfo.pages[next_page].last_record = next_rec;
	flashInfo.active_page = next_page;
	flashInfo.last_id = next_id;
	return true;
  }
  return false;
}
