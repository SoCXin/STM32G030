#include "user.h"
#include "uart.h"
uint8_t  u8Uart1RxBuf;
uint16_t u16Uart1RxIndex;
uint8_t  u8UartRxBuf[UART1BUF_SIZE];

uint8_t  u8CntUart1Timer1ms;

uint8_t Uart1Rxing;
//============================================================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uartHandle)
{
	if(uartHandle == &huart1)
	{
		__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE | UART_FLAG_RXFNE);
		__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE | UART_IT_RXFNE);
		__HAL_UART_GET_IT(&huart1, UART_IT_RXNE | UART_IT_RXFNE);
		if(u16Uart1RxIndex >= UART1BUF_SIZE)
		{
			u16Uart1RxIndex = 0;
		}
		//HAL_UART_Receive_IT(&huart1, &u8Uart1RxBuf, 1);
		u8UartRxBuf[u16Uart1RxIndex] = huart1.Instance->RDR;//u8Uart1RxBuf;
		u16Uart1RxIndex++;
		Uart1Rxing = 1;
		u8CntUart1Timer1ms = 0;
	}
}
//============================================================================

//============================================================================
void Uart1Receive_TimerOut(void)
{
	if(u8CntUart1Timer1ms >= 1)
	{
		u8CntUart1Timer1ms = 0;
		u16Uart1RxIndex = 0;
		Uart1Rxing = 0;
		//uint8_t temp_buf[] = "STM32G070CB UART1(115200) Transmit.\r\n";
		//HAL_UART_Transmit(&huart1,temp_buf,sizeof(temp_buf)-1,10);
	}
	else if(Uart1Rxing)
	{
		u8CntUart1Timer1ms++;
	}
}
//============================================================================

//============================================================================
void ClrUartRxBuf(void)
{
	uint16_t i;
	for(i=0; i<UART1BUF_SIZE; i++)
	{
		u8UartRxBuf[i] = 0;
	}
}
//============================================================================
