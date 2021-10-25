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
	// __set_FAULTMASK(1);
    __disable_irq();     //不允许被打断，关总中断
    NVIC_SystemReset();
}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void Mark_Set(uint8_t flag,uint32_t val)
{
    if(flag == 0)
    {
        LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR0,val);
    }
    else if(flag == 1)
    {
        LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR1,val);
    }
    else if(flag == 2)
    {
        LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR2,val);
    }
    else if(flag == 3)
    {
        LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR3,val);
    }
    else if(flag == 4)
    {
        LL_RTC_BKP_SetRegister(TAMP,LL_RTC_BKP_DR4,val);
    }

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
    if(BKP_APP1_ADDR==0 && BKP_APP2_ADDR==0 && BKP_APP1_CHECK==0 && BKP_APP2_CHECK==0)
    {
        if (((*(__IO uint32_t*)USER_APP1_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        {
            Mark_Set(0,USER_APP1_ADDRESS);
            Mark_Set(3,1);
        }
        else if (((*(__IO uint32_t*)USER_APP2_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        {
            Mark_Set(1,USER_APP2_ADDRESS);
            Mark_Set(4,1);
        }
    }
    if(BKP_APP1_ADDR)
    {
        app_ptr=BKP_APP1_ADDR;
        if(app_ptr < FLASH_START_BASE+FLASH_BLT_SIZEMAX || app_ptr > FLASH_START_BASE + fsize*0x400 ||  app_ptr%FLASH_PAGE_SIZE)
        {
            Mark_Set(0,USER_APP1_ADDRESS);
            // Mark_Set(3,0);
            app_ptr=USER_APP1_ADDRESS;  //默认地址
        }
        else if(BKP_APP1_CHECK)
        {
            if (appjump(app_ptr))  Mark_Set(3,0);
        }
    }
    if(BKP_APP2_ADDR)
    {
        app_ptr=BKP_APP2_ADDR;
        if(app_ptr< FLASH_START_BASE+FLASH_BLT_SIZEMAX || app_ptr > FLASH_START_BASE + fsize*0x400  || app_ptr%FLASH_PAGE_SIZE)
        {
            Mark_Set(1,USER_APP2_ADDRESS);
            // Mark_Set(4,0);
            app_ptr=USER_APP2_ADDRESS;
        }
        else if(BKP_APP2_CHECK)
        {
            if (appjump(app_ptr)) Mark_Set(4,0);
        }
    }
    uint8_t buf[50] ;
    sprintf((char *)buf, "IAP:%x,%x,%x,%x-%x\r\n",BKP_APP1_ADDR,BKP_APP2_ADDR,BKP_APP1_CHECK,BKP_APP2_CHECK,app_ptr);
    HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
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
