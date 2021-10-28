#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define FLASH_MARK_BASE  USER_APP1_ADDRESS - 0x10
#define FLASH_WAITETIME  FLASH_TIMEOUT_VALUE


uint8_t  FlashPageRead(uint32_t address, uint8_t *pbuf);
uint8_t  FlashPageWrite(uint32_t address, uint8_t *pbuf);
void FlashReset(uint8_t bank);

#ifdef __cplusplus
}
#endif

#endif
/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
