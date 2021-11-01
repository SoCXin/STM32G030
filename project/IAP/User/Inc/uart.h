#ifndef __UART_H
#define __UART_H
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define UART1BUF_SIZE 1029


extern uint8_t  u8UartRxChar;
extern uint16_t u16Uart1RxIndex;
extern uint8_t  u8CntUart1Timer1ms;
extern uint8_t  u8UartRxBuf[UART1BUF_SIZE];

void PortTimerInterrupt(void);
void BootPortInterrupt(void);
void uart_init(void);

void uart_tx_char(uint8_t ch);
void uart_tx_str(uint8_t *str, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif
/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
