#include "main.h"
#include "string.h"
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
uint8_t  u8AdcTrig1ms;
void PowerUpCounter(void)
{
    if(++u16Timer1ms >= 1000)
    {
        u16Timer1ms = 0;
        if(u16Timer1sec < 0xffff)
        {
            u16Timer1sec++;
        }
    }
    if(u8AdcTrig1ms < 0xff)
    {
        u8AdcTrig1ms++;
    }
}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint8_t u8Cnt5HzDelay;
void BLT_Indicator(void)
{
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
void bootinit(void)
{
    uint16_t fsize = *(uint16_t *)(FLASHSIZE_BASE);
    if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0))
    {
        app_ptr=HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
        if(app_ptr < FLASH_START_BASE+FLASH_BLT_SIZEMAX || app_ptr > FLASH_START_BASE + fsize*0x400 ||  app_ptr%FLASH_PAGE_SIZE)
        {
            app_ptr=USER_APP1_ADDRESS;  //默认地址
        }
        else if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3))
        {
            if (((*(__IO uint32_t*)app_ptr) & 0x2FFE0000 ) == 0x20000000)
            {
                JumpAddress = *(__IO uint32_t*) (app_ptr + 4);
                Jump_To_Application = (pFunction) JumpAddress;
                __set_MSP(*(__IO uint32_t*) app_ptr);
                Jump_To_Application();
            }
        }
    }
    if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1))
    {
        app_ptr=HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
        if(app_ptr< FLASH_START_BASE+FLASH_BLT_SIZEMAX || app_ptr > FLASH_START_BASE + fsize*0x400  || app_ptr%FLASH_PAGE_SIZE)
        {
            app_ptr=USER_APP2_ADDRESS;
        }
        else if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4))
        {
            if (((*(__IO uint32_t*)app_ptr) & 0x2FFE0000 ) == 0x20000000)
            {
                JumpAddress = *(__IO uint32_t*) (app_ptr + 4);
                Jump_To_Application = (pFunction) JumpAddress;
                __set_MSP(*(__IO uint32_t*) app_ptr);
                Jump_To_Application();
            }
        }
    }
    uint8_t buf[50] ;
    sprintf((char *)buf, "IAP:%x,%x,%x,%x-%x\r\n",HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0),HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1),HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3),HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4),app_ptr);
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
