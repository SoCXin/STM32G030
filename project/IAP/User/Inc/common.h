#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "port.h"

typedef enum
{
    bkp_app1_addr,
    bkp_app2_addr,
    bkp_app1_mark,
    bkp_app2_mark,
    bkp_boot_mark
}bkp_type;

#define USER_APP1_ADDRESS   0x08004000
#define USER_APP2_ADDRESS   0x08008000

uint8_t IAP_Set(bkp_type flag, uint32_t val);
uint32_t IAP_Get(bkp_type flag);


#ifdef __cplusplus
}
#endif
#endif
/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
