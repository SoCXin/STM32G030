
#include "common.h"

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void sysReset(void)
{
    __disable_irq();     //不允许被打断，关总中断
    NVIC_SystemReset();
}
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint32_t Mark_Get(bkp_type flag)
{
    #ifdef _USE_BKP
    return LL_RTC_BKP_GetRegister(TAMP,LL_RTC_BKP_DR0+flag);
    #else
    return LL_RTC_BKP_GetRegister(TAMP,LL_RTC_BKP_DR0+flag);
    #endif

}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint8_t Mark_Set(bkp_type flag, uint32_t val)
{
    #ifdef _USE_BKP
    LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR0+flag,val);
    #else
    #endif
    return 0;
}

#ifdef _USE_BKP
#else
#endif


/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/