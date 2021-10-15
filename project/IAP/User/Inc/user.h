
#ifndef __USER_H
#define __USER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define AD_CH_NUM 2

extern uint32_t u32aResultDMA[AD_CH_NUM/2];

void HAL_SYSTICK_Callback(void);

#ifdef __cplusplus
}
#endif
#endif

