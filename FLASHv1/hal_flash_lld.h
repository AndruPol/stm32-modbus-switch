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
 * @file    hal_flash_lld.h
 * @brief   Generic flash driver class header.
 *
 * @addtogroup HAL_FLASH
 * @{
 */

#ifndef HAL_FLASH_LLD_H
#define HAL_FLASH_LLD_H

#include "hal_flash.h"

#if HAL_USE_FLASH || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/* Flag for the whole STM32F1XX family. */
#if defined(STM32F10X_LD_VL) || defined(STM32F10X_MD_VL) || \
    defined(STM32F10X_HD_VL) || defined(STM32F10X_LD)    || \
    defined(STM32F10X_MD)    || defined(STM32F10X_HD)    || \
    defined(STM32F10X_XL)    || defined(STM32F10X_CL)
#define STM32F1XX_FLASH
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if defined(STM32F10X_XL)
#warning "FLASHv1 driver for STM32F10X_XL support only 1st bank"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   @p STM32F1xx Flash specific methods with inherited ones.
 */
#define _flash_driver_methods                                               \
  _base_flash_methods

/**
 * @brief   @p STM32F1xx specific data.
 */
#define _flash_driver_data                                                  \
  _base_flash_data                                                          \
  FLASH_TypeDef     *flash_p;

/**
 * @brief   @p FlashDriver virtual methods table.
 */
struct FlashDriverVMT {
  _flash_driver_methods
};

/**
 * @brief   Base flash class.
 */
typedef struct FlashDriver {
  /** @brief Virtual Methods Table.*/
  const struct FlashDriverVMT *vmt;
  _flash_driver_data
} FlashDriver;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern FlashDriver FD1;

#ifdef __cplusplus
extern "C" {
#endif
  void flash_lld_init(void);

  const flash_descriptor_t* flash_lld_get_descriptor(void *instance);

  flash_error_t flash_lld_read(void *instance, flash_offset_t offset,
                               size_t n, uint8_t *rp);
  flash_error_t flash_lld_program(void *instance, flash_offset_t offset,
                                  size_t n, const uint8_t *pp);
  flash_error_t flash_lld_start_erase_all(void *instance);
  flash_error_t flash_lld_start_erase_sector(void *instance,
                                             flash_sector_t sector);
  flash_error_t flash_lld_query_erase(void *instance, uint32_t *wait_time);
  flash_error_t flash_lld_verify_erase(void *instance, flash_sector_t sector);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_FLASH */

#endif /* HAL_FLASH_LLD_H */

/** @} */
