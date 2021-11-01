#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "flash.h"
#include "common.h"
#include "ymodem.h"


#define QITAS_BLT_MAX       0x2000
#define QITAS_APP_MAX       0x4000
#ifdef BLT
#endif


void bootinit(void);
void bootloop(void);
void BootTimerInterrupt(void);



#ifdef __cplusplus
}
#endif
#endif
/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
