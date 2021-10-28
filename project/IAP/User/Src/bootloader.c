#include "main.h"
#include <stdio.h>
#include <string.h>
#include "bootloader.h"
#include "ymodem.h"

typedef  void (*pFunction)(void);

pFunction Jump_To_Application;
uint32_t JumpAddress;
uint16_t u16Timer1ms;
uint16_t u16Timer1sec;
uint8_t  u8KeyInputSate;
static uint32_t app_ptr;

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
uint32_t IAP_Get(bkp_type flag)
{
    return LL_RTC_BKP_GetRegister(TAMP,LL_RTC_BKP_DR0+flag);
}
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint8_t IAP_Set(bkp_type flag, uint32_t val)
{
    LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR0+flag,val);
    return 0;
    // switch (flag)
    // {
    // case bkp_app1_addr:
    //     #ifdef _USE_BKP
    //     LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR0+bkp_app1_addr,val);
    //     #else
    //     #endif
    //     break;
    // case bkp_app1_mark:
    //     #ifdef _USE_BKP
    //     LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR0+bkp_app1_mark,val);
    //     #else
    //     #endif
    //     break;
    // case bkp_app2_addr:
    //     #ifdef _USE_BKP
    //     LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR0+bkp_app2_addr,val);
    //     #else
    //     #endif
    //     break;
    // case bkp_app2_mark:
    //     #ifdef _USE_BKP
    //     LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR0+bkp_app2_mark,val);
    //     #else
    //     #endif
    //     break;
    // default:
    //     #ifdef _USE_BKP
    //     LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR0+bkp_boot_mark,val);
    //     #else
    //     #endif
    //     break;
    // }
}


/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint8_t u8Cnt5HzDelay;
uint8_t  u8AdcTrig1ms;

void BootTimerInterrupt(void)
{
    if(++u16Timer1ms >= 1000)
    {
        u16Timer1ms = 0;
        if(u16Timer1sec < 0xffff)
        {
            u16Timer1sec++;
        }
    }
    if(u8AdcTrig1ms < 0xff) u8AdcTrig1ms++;
    Wait10msCountDwn();
    if(u8TranState <= 1)
    {
        u8Cnt5HzDelay = 0;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        if(u16Timer1sec >= 30)
        {
            sysReset();
        }
    }
    else if(u8TranState < 4)
    {
        if(u16Timer1ms < 500)
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        }
    }
    else
    {
        if(u8AdcTrig1ms >= 100)
        {
            u8AdcTrig1ms = 0;
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            if(++u8Cnt5HzDelay >= 30)
            {
                uint8_t buf[] = "\r\nSTM32G030 Bootloader \r\n";
                HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
                u8TranState = 0;
            }
        }
    }
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
        JumpAddress = *(__IO uint32_t*) (addr + 4);
        Jump_To_Application = (pFunction) JumpAddress;
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
void bootinit(void)
{
    uint16_t fsize = *(uint16_t *)(FLASHSIZE_BASE);
    if(IAP_Get(bkp_app1_addr)==0 && IAP_Get(bkp_app2_addr)==0 && IAP_Get(bkp_app1_mark)==0 && IAP_Get(bkp_app2_mark)==0)
    {
        if (((*(__IO uint32_t*)USER_APP1_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        {
            IAP_Set(bkp_app1_addr,USER_APP1_ADDRESS);
            IAP_Set(bkp_app1_mark,1);
        }
        else if (((*(__IO uint32_t*)USER_APP2_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        {
            IAP_Set(bkp_app2_addr,USER_APP2_ADDRESS);
            IAP_Set(bkp_app2_mark,1);
        }
    }
    if(IAP_Get(bkp_app1_addr))
    {
        app_ptr=IAP_Get(bkp_app1_addr);
        if(app_ptr < FLASH_START_BASE+FLASH_BLT_SIZEMAX || app_ptr > FLASH_START_BASE + fsize*0x400 ||  app_ptr%FLASH_PAGE_SIZE)
        {
            IAP_Set(bkp_app1_addr,USER_APP1_ADDRESS);
            app_ptr=USER_APP1_ADDRESS;  //默认地址
        }
        else if(IAP_Get(bkp_app1_mark))
        {
            if (appjump(app_ptr)) IAP_Set(bkp_app1_mark,0);
        }
    }
    if(IAP_Get(bkp_app2_addr))
    {
        app_ptr=IAP_Get(bkp_app2_addr);
        if(app_ptr< FLASH_START_BASE+FLASH_BLT_SIZEMAX || app_ptr > FLASH_START_BASE + fsize*0x400  || app_ptr%FLASH_PAGE_SIZE)
        {
            IAP_Set(bkp_app2_addr,USER_APP2_ADDRESS);
            app_ptr=USER_APP2_ADDRESS;
        }
        else if(IAP_Get(bkp_app2_mark))
        {
            if (appjump(app_ptr)) IAP_Set(bkp_app2_mark,0);
        }
    }
}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void bootloop(void)
{
    if(app_ptr) Ymodem_Transmit(app_ptr);
}

/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
