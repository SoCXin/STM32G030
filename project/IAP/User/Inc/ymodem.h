#ifndef YMODEM_H_
#define YMODEM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Private define ------------------------------------------------------------*/
#define MODEM_SOH 0x01 //133字节数据包类型，接收正常回应0x06(含文件信息的第一个包接收正常需回应0x06、0x43)
#define MODEM_STX 0x02 //1029字节数据包类型，接收正常回应0x06
#define MODEM_EOT 0x04 //发送文件传输结束命令，接收正常回应0x06、0x43(启动空包发送)
#define MODEM_ACK 0x06 //发送确认应答，接收方crc校验成功或收到已定义的命令
#define MODEM_NAK 0x15 //发送重传当前数据包请求，接收方crc校验出错
#define MODEM_CAN 0x18 //发送取消传输命令,连续发送5个字符
#define MODEM_C   0x43 //发送大写字母C(三种情况下发送该字符: 1.启动通信握手.2.启动数据包发送.3.启动空包发送)
#define TIME_OVER_SETUP1  10000
#define TIME_OVER_SETUP2  30000


/* Extern variables ----------------------------------------------------------*/
extern uint16_t u16Wait10ms;
extern uint8_t  u8TranState;
extern uint8_t  FlashWriteBuf[FLASH_PAGE_SIZE];


void delay_ms(uint16_t ms);
void Send_CMD(uint8_t cmd);
void Wait10msCountDwn(void);
uint16_t Ymodem_CRC(uint8_t * buf, uint16_t len);
uint8_t YmodemReceiveDate(const uint32_t START_ADDR);
void UartRxUpdateCMD(uint32_t start_adress);

uint32_t GetFirmwareSize(uint8_t *rxbuf);
uint32_t GetFirmwareChksum(uint8_t *rxbuf);
uint32_t CalcRomChksum(uint32_t address, uint32_t length);

void Ymodem_Transmit(const uint32_t ADDR);
// void Ymodem_Transmit(uint8_t flag);
#ifdef __cplusplus
}
#endif
#endif
/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
