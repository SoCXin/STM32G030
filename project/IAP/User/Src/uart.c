/*******************************************************************************
****版本：
****平台：
****日期：
****作者：Qitas
****版权：
*******************************************************************************/
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
void BootPortInterrupt(void)
{
    if(LL_USART_IsActiveFlag_RXNE(USART1))
    {
        if(u16Uart1RxIndex >= UART1BUF_SIZE) u16Uart1RxIndex = 0;
        u8UartRxBuf[u16Uart1RxIndex++] = USART1->RDR;
        Uart1Rxing = 1;
        u8CntUart1Timer1ms = 0;
        // LL_USART_TransmitData8(USART1, u16Uart1RxIndex);
	}
    // if(LL_USART_IsActiveFlag_PE(USART1))
	// {
    //     LL_USART_ClearFlag_PE(USART1);
    // }
}
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void PortTimerInterrupt(void)
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

void uart_init(void)
{
    // HAL_UART_Receive_IT(&huart1, &u8UartRxChar, 1);
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_EnableIT_PE(USART1);
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
        USART1->TDR = str[i] ;
        while((USART1->ISR&0X40)==0);
        // LL_USART_TransmitData8(USART1, str[i]);
        // while(!LL_USART_IsActiveFlag_TC(USART1));
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
    // LL_USART_TransmitData8(USART1, ch);
    // while(!LL_USART_IsActiveFlag_TC(USART1));
    while((USART1->ISR&0X40)==0);
    USART1->TDR = ch;
    // while((USART1->ISR&0X40)==0);
    // USART1->TDR = 0;
}


/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
