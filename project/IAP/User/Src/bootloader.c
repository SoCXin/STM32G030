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
//==============================================================================
void ApplicationSelect(void)
{
    if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin)==GPIO_PIN_RESET) //(u8KeyInputSate == 0)//
    {
        if (((*(__IO uint32_t*)USER_APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        {
            JumpAddress = *(__IO uint32_t*) (USER_APP_ADDRESS + 4);
            Jump_To_Application = (pFunction) JumpAddress;
            __set_MSP(*(__IO uint32_t*) USER_APP_ADDRESS);
            Jump_To_Application();
        }
        // HAL_UART_Transmit(&huart1,"HAL_GPIO_ReadPin\r\n",15,100);
    }
}

//==============================================================================

//==============================================================================
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
//==============================================================================

//==============================================================================
u8 u8KeyInputChkOver;
void PowUpKeyInputState(void)
{
	uint16_t u16AdValueKey;
	uint16_t u16KeyPress = 0;
	uint8_t u8KeyInputSateOld = 0;
	u8KeyInputSate = 0;
	u8KeyInputChkOver = 0;
	READ_KEY_AD:
	if(u8KeyInputSateOld != u8KeyInputSate)
	{
		u16KeyPress = 0;
		u8KeyInputSateOld = u8KeyInputSate;
	}
	while(u8AdcTrig1ms < 10)
	{
		HAL_IWDG_Refresh(&hiwdg);
	}
	u8AdcTrig1ms = 0;
	// HAL_ADC_Start_DMA(&hadc1, (u32*)&u32aResultDMA, AD_CH_NUM); //
	// u16AdValueKey = (uint16_t)u32aResultDMA[0];

	if(u16AdValueKey >= 800 && u16AdValueKey <= 950)
	{
		u8KeyInputSate = 1;
		if(++u16KeyPress < 100)
		{
			HAL_IWDG_Refresh(&hiwdg);
			goto READ_KEY_AD;
		}
	}
	else
	{
		u8KeyInputSate = 0;
		if(++u16KeyPress < 10)
		{
			HAL_IWDG_Refresh(&hiwdg);
			goto READ_KEY_AD;
		}
	}

	u16Timer1ms = 0;
	u16Timer1sec = 0;
	while(u16Timer1ms < 100);
	u8KeyInputChkOver = 1;
}
//==============================================================================

//==============================================================================
uint8_t u8Cnt5HzDelay;
void Iap_Indicator(void)
{
	if(u8KeyInputChkOver == 0)
	{
		return;
	}
	if(u8TranState <= 1)
	{
		u8Cnt5HzDelay = 0;
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		if(u16Timer1sec >= 30)
		{
			McuReset();
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
				uint8_t buf[] = "\r\nSTM32G070CB UART2(115200) Bootloader V1.0.\r\n";
				HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
				u8TranState = 0;
			}
		}
	}
}
//==============================================================================


