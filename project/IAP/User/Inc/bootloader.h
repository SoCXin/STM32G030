#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "flash.h"
#include "common.h"
#include "ymodem.h"

#ifdef BLT
#define QITAS_BLT           0x2000
#define FLASH_SIZE_ADDR     0x1FFFF7E0
#define FLASH_START_BASE    0x08000000
#define FLASH_BLT_SIZEMAX   0x4000
#define APP_START_PAGE      ((USER_APP1_ADDRESS - FLASH_START_BASE) / FLASH_PAGE_SIZE)
#endif


#define _USE_BKP

#ifdef _USE_BKP
#else
#endif

extern uint8_t  u8KeyInputSate;

void sysReset(void);
void bootinit(void);
void bootloop(void);
void BootTimerInterrupt(void);



#ifdef __cplusplus
}
#endif
#endif
/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
