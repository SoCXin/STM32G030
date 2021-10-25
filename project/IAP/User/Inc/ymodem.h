#ifndef YMODEM_H_
#define YMODEM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Private define ------------------------------------------------------------*/
#define MODEM_SOH 0x01 //133�ֽ����ݰ����ͣ�����������Ӧ0x06(���ļ���Ϣ�ĵ�һ���������������Ӧ0x06��0x43)
#define MODEM_STX 0x02 //1029�ֽ����ݰ����ͣ�����������Ӧ0x06
#define MODEM_EOT 0x04 //�����ļ���������������������Ӧ0x06��0x43(�����հ�����)
#define MODEM_ACK 0x06 //����ȷ��Ӧ�𣬽��շ�crcУ��ɹ����յ��Ѷ��������
#define MODEM_NAK 0x15 //�����ش���ǰ���ݰ����󣬽��շ�crcУ�����
#define MODEM_CAN 0x18 //����ȡ����������,��������5���ַ�
#define MODEM_C   0x43 //���ʹ�д��ĸC(��������·��͸��ַ�: 1.����ͨ������.2.�������ݰ�����.3.�����հ�����)
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
