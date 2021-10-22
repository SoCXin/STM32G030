#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_system.h"
#include "data_type.h"

#define FLASH_SIZE_ADDR     0x1FFFF7E0
#define FLASH_START_BASE    0x08000000
#define FLASH_BLT_SIZEMAX   0x4000
#define USER_APP1_ADDRESS   0x08004000
#define USER_APP2_ADDRESS   0x0800A000

#define APP_START_PAGE      ((USER_APP1_ADDRESS - FLASH_START_BASE) / FLASH_PAGE_SIZE)

extern uint8_t  u8KeyInputSate;

void sysReset(void);
void bootinit(void);
void bootloop(void);
void PowerUpCounter(void);
void BLT_Indicator(void);

#ifdef __cplusplus
}
#endif
#endif
/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
