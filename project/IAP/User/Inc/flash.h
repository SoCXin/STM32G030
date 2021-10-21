#ifndef __FLASH_H
#define __FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_system.h"
#include "data_type.h"


#define FLASH_MARK_BASE  USER_APP_ADDRESS - 0x10

#define FLASH_WAITETIME  1000


u8  FlashPageRead(u32 address, u8 *pbuf);
u8  FlashPageWrite(u32 address, u8 *pbuf);
void FlashTestWR(void);

#ifdef __cplusplus
}
#endif

#endif
