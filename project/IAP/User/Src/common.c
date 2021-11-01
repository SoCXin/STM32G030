
#include "common.h"

typedef  void (*pFunction)(void);

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
uint8_t appjump(const uint32_t addr)
{
    if (((*(__IO uint32_t*)addr) & 0x2FFE0000 ) == 0x20000000)
    {
        uint32_t JumpAddress = *(__IO uint32_t*) (addr + 4);
        pFunction Jump_To_Application = (pFunction) JumpAddress;
        __set_MSP(*(__IO uint32_t*) addr);
        Jump_To_Application();
        return 0;
    }
    else return 1;
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
