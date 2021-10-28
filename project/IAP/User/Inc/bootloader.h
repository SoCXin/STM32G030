#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "uart.h"
#include "flash.h"
#include "ymodem.h"

#define FLASH_SIZE_ADDR     0x1FFFF7E0
#define FLASH_START_BASE    0x08000000
#define FLASH_BLT_SIZEMAX   0x4000
#define USER_APP1_ADDRESS   0x08004000
#define USER_APP2_ADDRESS   0x0800A000

#define APP_START_PAGE      ((USER_APP1_ADDRESS - FLASH_START_BASE) / FLASH_PAGE_SIZE)

typedef enum
{
    bkp_app1_addr,
    bkp_app2_addr,
    bkp_app1_mark,
    bkp_app2_mark,
    bkp_boot_mark
}bkp_type;

#define _USE_BKP

#ifdef _USE_BKP
#else
#endif


extern uint8_t  u8KeyInputSate;

void sysReset(void);
void bootinit(void);
void bootloop(void);
void BootTimerInterrupt(void);

uint8_t IAP_Set(bkp_type flag, uint32_t val);
uint32_t IAP_Get(bkp_type flag);

#ifdef __cplusplus
}
#endif
#endif
/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
