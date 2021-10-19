/**
  ******************************************************************************
  * @file           : uart.h
  * @brief          : uart head file
  ******************************************************************************
 */

#ifndef __UART_H
#define __UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_system.h"

#define UART1BUF_SIZE 1029


extern uint8_t  u8Uart1RxBuf;
extern uint16_t u16Uart1RxIndex;
extern uint8_t  u8UartRxBuf[UART1BUF_SIZE];

extern uint8_t  u8CntUart1Timer1ms;


void Uart1Receive_TimerOut(void);
void ClrUartRxBuf(void);
void uart_init(void);

#ifdef __cplusplus
}
#endif

#endif
