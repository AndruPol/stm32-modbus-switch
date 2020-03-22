/*
    ChibiOS - Copyright (C) 2017 Marco Bascetta

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    hal_flash_lld.c
 * @brief   Generic flash driver class code.
 *
 * @addtogroup HAL_FLASH
 * @{
 */

#include "hal.h"

#include "hal_flash_lld.h"

#if HAL_USE_FLASH || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if defined(STM32F10X_LD) || defined(STM32F10X_LD_VL)
#   define  STM32F10X_PAGE_COUNT    32
#   define  STM32F10X_PAGE_SIZE     1024U
#elif defined(STM32F10X_MD) || defined(STM32F10X_MD_VL)
#   define  STM32F10X_PAGE_COUNT    128
#   define  STM32F10X_PAGE_SIZE     1024U
#elif defined(STM32F10X_CL)
#   define  STM32F10X_PAGE_COUNT    128
#   define  STM32F10X_PAGE_SIZE     2048U
#elif defined(STM32F10X_HD) || defined(STM32F10X_HD_VL)
#   define  STM32F10X_PAGE_COUNT    256
#   define  STM32F10X_PAGE_SIZE     2048U
#elif defined(STM32F10X_XL)
#   define  STM32F10X_PAGE_COUNT    256 * 2
#   define  STM32F10X_PAGE_SIZE     2048U
#else
#   error   "Missing STM32F10X definition"
#endif

#if !defined(FLASH_LLD_WAIT_TIME_MS)
#   define FLASH_LLD_WAIT_TIME_MS   10
#endif /* !defined(FLASH_LLD_WAIT_TIME_MS) */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

static const struct FlashDriverVMT vmt = {
  flash_lld_get_descriptor, flash_lld_read, flash_lld_program,              \
  flash_lld_start_erase_all, flash_lld_start_erase_sector,                  \
  flash_lld_query_erase, flash_lld_verify_erase
};

/** @brief Flash driver identifier.*/
FlashDriver FD1 = {
  .vmt = &vmt,
  .state = FLASH_READY,
  .flash_p = FLASH
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Internal STM32F10x descriptor.
 */
static flash_descriptor_t flash_descriptor = {
  .attributes       = FLASH_ATTR_ERASED_IS_ONE | FLASH_ATTR_REWRITABLE |
                      FLASH_ATTR_SUSPEND_ERASE_CAPABLE,
  .page_size        = STM32F10X_PAGE_SIZE,
  .sectors_count    = STM32F10X_PAGE_COUNT,
  .sectors          = NULL,
  .sectors_size     = STM32F10X_PAGE_SIZE,
  .address          = FLASH_BASE
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#define FLASH_CR_RESET_FLAG(flag) ((~(flag)) & 0x1FFFF)

#define flash_lld_reset_sr(devp)                                            \
  devp->flash_p->SR = FLASH_SR_BSY | FLASH_SR_PGERR | FLASH_SR_EOP

static void flash_lld_lock(void *instance) {
  FlashDriver *devp = (FlashDriver *)instance;

  devp->flash_p->CR |= FLASH_CR_LOCK;
}

static void flash_lld_unlock(void *instance) {
  FlashDriver *devp = (FlashDriver *)instance;

  devp->flash_p->KEYR = FLASH_KEY1;
  devp->flash_p->KEYR = FLASH_KEY2;
}

static flash_error_t flash_lld_wait_busy(FlashDriver *devp) {
  uint32_t SR = 0;

  do {
    SR = (devp->flash_p->SR);

    if( (SR & FLASH_SR_PGERR) || (SR & FLASH_SR_WRPRTERR) ) {
      return FLASH_ERROR_HW_FAILURE;
    }

  } while( (SR & FLASH_SR_BSY) == FLASH_SR_BSY);

  return FLASH_NO_ERROR;
}

static flash_error_t flash_lld_program_halfword(FlashDriver *devp,
                                                uint32_t address,
                                                uint16_t wdata) {
  flash_lld_wait_busy(devp);

  devp->flash_p->CR |= FLASH_CR_PG;

  *(volatile uint16_t*)address = wdata;

  flash_lld_wait_busy(devp);

  /* Disable the PG Bit */
  devp->flash_p->CR &= FLASH_CR_RESET_FLAG(FLASH_CR_PG);

  if( *(volatile uint16_t*)address != wdata ) {
    return FLASH_ERROR_PROGRAM;
  }

  return FLASH_NO_ERROR; // FLASH_ERROR_ERASE
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void flash_lld_init(void) {
}

/**
 *
 * @param instance
 * @return
 */
const flash_descriptor_t* flash_lld_get_descriptor(void *instance) {
  FlashDriver *devp = (FlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state != FLASH_UNINIT) && (devp->state != FLASH_STOP),
                "invalid state");

  return &flash_descriptor;
}

flash_error_t flash_lld_read(void *instance, flash_offset_t offset,
                             size_t n, uint8_t *rp) {
  FlashDriver *devp = (FlashDriver *)instance;
  uint32_t address  = flash_descriptor.address + offset;

  osalDbgCheck((instance != NULL) && (rp != NULL) && (n > 0U));
  osalDbgCheck((size_t)offset + n <= (size_t)flash_descriptor.sectors_count *
                                     (size_t)flash_descriptor.sectors_size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
               "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READY state while the operation is performed.*/
  devp->state = FLASH_READ;

  while(n > 1) {
    *((uint16_t*)rp) = *(volatile uint16_t*)address;

    address += 2;
    rp      += 2;
    n       -= 2;
  }

  if(n > 0) {
    *rp = (uint8_t)((*(volatile uint16_t*)address) & 0x00FF);
  }

  devp->state = FLASH_READY;

  return FLASH_NO_ERROR;
}

flash_error_t flash_lld_program(void *instance, flash_offset_t offset,
                                size_t n, const uint8_t *pp) {
  FlashDriver *devp = (FlashDriver *)instance;
  uint32_t address  = flash_descriptor.address + offset;
  uint16_t wdata    = 0;
  flash_error_t fe  = FLASH_NO_ERROR;

  osalDbgCheck((instance != NULL) && (pp != NULL) && (n > 0U));
  osalDbgCheck((size_t)offset + n <= (size_t)flash_descriptor.sectors_count *
                                     (size_t)flash_descriptor.sectors_size);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  devp->state = FLASH_PGM;

  flash_lld_unlock(devp);

  while(n > 1) {
    wdata = (*(uint16_t*)pp);

    fe = flash_lld_program_halfword(devp, address, wdata);
    if( fe != FLASH_NO_ERROR ) {
      flash_lld_lock(devp);
      devp->state = FLASH_READY;
      return FLASH_ERROR_PROGRAM;
    }

    address += 2;
    pp      += 2;
    n       -= 2;
  }

  if(n > 0) {
    wdata = *pp | 0xFF00;

    fe = flash_lld_program_halfword(devp, address, wdata);
    if( fe != FLASH_NO_ERROR ) {
      flash_lld_lock(devp);
      devp->state = FLASH_READY;
      return FLASH_ERROR_PROGRAM;
    }
  }

  flash_lld_lock(devp);

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return FLASH_NO_ERROR;
}

flash_error_t flash_lld_start_erase_all(void *instance)
{
  FlashDriver *devp = (FlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_ERASE state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  flash_lld_wait_busy(devp);
  flash_lld_unlock(devp);

  devp->flash_p->CR |= FLASH_CR_MER;
  devp->flash_p->CR |= FLASH_CR_STRT;

  /* flash_lld_lock must be done in query_erase */

  return FLASH_NO_ERROR;
}

flash_error_t flash_lld_start_erase_sector(void *instance,
                                           flash_sector_t sector) {
  FlashDriver *devp = (FlashDriver *)instance;
  uint32_t address  = flash_descriptor.address +
                      (sector * flash_descriptor.sectors_size);

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < flash_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_ERASE state while the operation is performed.*/
  devp->state = FLASH_ERASE;

  flash_lld_wait_busy(devp);
  flash_lld_unlock(devp);

  devp->flash_p->CR |= FLASH_CR_PER;
  devp->flash_p->AR  = address;
  devp->flash_p->CR |= FLASH_CR_STRT;

  /* flash_lld_lock must be done in query_erase */

  return FLASH_NO_ERROR;
}

flash_error_t flash_lld_query_erase(void *instance, uint32_t *wait_time)
{
  FlashDriver *devp = (FlashDriver *)instance;

  osalDbgCheck(instance != NULL);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if( wait_time ) {
      *wait_time = FLASH_LLD_WAIT_TIME_MS;
  }

  /* If there is an erase in progress then the device must be checked.*/
  if (devp->state == FLASH_ERASE) {
    uint32_t SR = devp->flash_p->SR;

    if( (SR & FLASH_SR_BSY) == FLASH_SR_BSY ) {
        return FLASH_BUSY_ERASING;
    }

    /* Erase completed; disable the PER/MER Bit */
    devp->flash_p->CR &= FLASH_CR_RESET_FLAG(FLASH_CR_PER | FLASH_CR_MER);

    flash_lld_lock(devp);

    devp->state = FLASH_READY;

    if( (SR & FLASH_SR_PGERR)     == FLASH_SR_PGERR       ||
        (SR & FLASH_SR_WRPRTERR)  == FLASH_SR_WRPRTERR    ) {

      flash_lld_reset_sr(devp);
      return FLASH_ERROR_ERASE;
    }
  }

  return FLASH_NO_ERROR;
}

flash_error_t flash_lld_verify_erase(void *instance, flash_sector_t sector) {
  FlashDriver *devp = (FlashDriver *)instance;
  uint32_t address  = flash_descriptor.address +
                      (sector * flash_descriptor.sectors_size);

  uint16_t wdata    = 0;

  osalDbgCheck(instance != NULL);
  osalDbgCheck(sector < flash_descriptor.sectors_count);
  osalDbgAssert((devp->state == FLASH_READY) || (devp->state == FLASH_ERASE),
                "invalid state");

  if (devp->state == FLASH_ERASE) {
    return FLASH_BUSY_ERASING;
  }

  /* FLASH_READ state while the operation is performed.*/
  devp->state = FLASH_READ;

  /* Read specified sector. All data must be 0xFFFFU */
  uint32_t address_max = address + flash_descriptor.sectors_size;
  for(; address < address_max; address += 2) {

    wdata = *(volatile uint16_t*)address;

    if( wdata != 0xFFFF ) {
      return FLASH_ERROR_VERIFY;
    }
  }

  /* Ready state again.*/
  devp->state = FLASH_READY;

  return FLASH_NO_ERROR;
}

#endif /* HAL_USE_FLASH */

/** @} */
