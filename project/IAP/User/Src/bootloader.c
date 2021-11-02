#include "main.h"
#include <stdio.h>
#include <string.h>
#include "common.h"
#include "ymodem.h"
#include "bootloader.h"


uint16_t u16Timer1ms;
uint16_t u16Timer1sec;

static uint32_t app_ptr;

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
    PortTimerInterrupt();
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
        // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        if(u16Timer1sec >= 30)
        {
            sysReset();
        }
    }
    else if(u8TranState < 4)
    {
        if(u16Timer1ms < 500)
        {
            // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
        }
        else
        {
            // HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
        }
    }
    else
    {
        if(u8AdcTrig1ms >= 100)
        {
            u8AdcTrig1ms = 0;
            // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            if(++u8Cnt5HzDelay >= 30)
            {
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
void bootinit(void)
{
    uart_init();
    flash_init();
    if(Mark_Get(bkp_app1_addr)==0 && Mark_Get(bkp_app2_addr)==0 && Mark_Get(bkp_app1_mark)==0 && Mark_Get(bkp_app2_mark)==0)
    {
        if (((*(__IO uint32_t*)USER_APP1_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        {
            Mark_Set(bkp_app1_addr,USER_APP1_ADDRESS);
            Mark_Set(bkp_app1_mark,1);
        }
        else if (((*(__IO uint32_t*)USER_APP2_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        {
            Mark_Set(bkp_app2_addr,USER_APP2_ADDRESS);
            Mark_Set(bkp_app2_mark,1);
        }
        else
        {
            Mark_Set(bkp_app1_addr,USER_APP1_ADDRESS);
        }
    }
    if(Mark_Get(bkp_app1_addr))
    {
        app_ptr=Mark_Get(bkp_app1_addr);
        if(app_ptr < FLASH_ADDR_BASE || app_ptr > FLASH_ADDR_BASE + flash_size ||  app_ptr%FLASH_PAGE_SIZE)
        {
            Mark_Set(bkp_app1_addr,USER_APP1_ADDRESS);
            app_ptr=USER_APP1_ADDRESS;      //默认地址
        }
        else if(Mark_Get(bkp_app1_mark))
        {
            if (appjump(app_ptr)) Mark_Set(bkp_app1_mark,0);
        }
    }
    if(Mark_Get(bkp_app2_addr))
    {
        app_ptr=Mark_Get(bkp_app2_addr);
        if(app_ptr< FLASH_ADDR_BASE || app_ptr > FLASH_ADDR_BASE + flash_size  || app_ptr%FLASH_PAGE_SIZE)
        {
            Mark_Set(bkp_app2_addr,USER_APP2_ADDRESS);
            app_ptr=USER_APP2_ADDRESS;
        }
        else if(Mark_Get(bkp_app2_mark))
        {
            if (appjump(app_ptr)) Mark_Set(bkp_app2_mark,0);
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
    // uart_tx_char(u16Uart1RxIndex);
    // uart_tx_char(u8CntUart1Timer1ms);
}

/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
