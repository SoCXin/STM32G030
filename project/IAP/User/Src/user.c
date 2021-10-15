
#include "user.h"
#include "uart.h"
#include "flash.h"
#include "bootloader.h"
#include "ymodem.h"

uint32_t u32aResultDMA[AD_CH_NUM/2];

void HAL_SYSTICK_Callback(void)
{
	PowerUpCounter();
	Uart1Receive_TimerOut();
	Wait10msCountDwn();
	ms_count_down();
	Iap_Indicator();
}


int task(void)
{
	PowUpKeyInputState();
	ApplicationSelect();
	FlashTestWR();
	while (1)
	{
		HAL_IWDG_Refresh(&hiwdg);
		Ymodem_Transmit(USER_APP_ADDRESS);
	}
}

