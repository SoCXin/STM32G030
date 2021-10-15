/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : uart head file
  ******************************************************************************
 */

#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_system.h"
#include "data_type.h"

#define FLASH_START_BASE    0x08000000
#define USER_APP_ADDRESS    0x8004000
#define APP_START_PAGE      ((USER_APP_ADDRESS - FLASH_START_BASE) / FLASH_PAGE_SIZE)


extern uint8_t  u8KeyInputSate;

void ApplicationSelect(void);
void PowerUpCounter(void);
void PowUpKeyInputState(void);
void Iap_Indicator(void);

#ifdef __cplusplus
}
#endif
#endif
