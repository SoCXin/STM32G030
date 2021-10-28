#include "main.h"
#include "uart.h"

uint16_t u16Uart1RxIndex;
uint8_t  u8UartRxBuf[UART1BUF_SIZE];
uint8_t  u8CntUart1Timer1ms;
uint8_t Uart1Rxing;
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
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
        u8UartRxBuf[u16Uart1RxIndex] = huart1.Instance->RDR;//u8Uart1RxBuf;
        u16Uart1RxIndex++;
        Uart1Rxing = 1;
        u8CntUart1Timer1ms = 0;
        HAL_UART_Receive_IT(&huart1, &u8Uart1RxBuf, 1);
    }
}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void UartTimerInterrupt(void)
{
    if(u8CntUart1Timer1ms >= 10)
    {
        u8CntUart1Timer1ms = 0;
        u16Uart1RxIndex = 0;
        Uart1Rxing = 0;
    }
    else if(Uart1Rxing) u8CntUart1Timer1ms++;
}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
static uint8_t u8Uart1RxBuf;
void uart_init(void)
{
    HAL_UART_Receive_IT(&huart1, &u8Uart1RxBuf, 1);
}
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void uart_tx_str(uint8_t *str, uint16_t Len)
{
    for(uint16_t i=0;i<Len;i++)
    {
        while((USART1->ISR&0X40)==0);
        USART1->TDR = str[i] ;
    }
}
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void uart_tx_char(uint8_t ch)
{
    while((USART1->ISR&0X40)==0);
    USART1->TDR = ch;
    // while((USART1->ISR&0X40)==0);
    // USART1->TDR = 0;
}

/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
