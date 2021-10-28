#include "main.h"
#include "bootloader.h"
#include "ymodem.h"
#include "uart.h"
#include "flash.h"
#include <stdio.h>

uint8_t  u8YmodeType;   //SOH(128) or STX(1024)
uint8_t  u8TranState;
uint16_t u16Wait10ms;
uint16_t u16CntFlashPage;

uint8_t  FlashWriteBuf[FLASH_PAGE_SIZE];
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint16_t u16Count1ms;

void delay_ms(uint16_t ms)
{
    u16Count1ms = ms;
    while(u16Count1ms)
    {
        HAL_IWDG_Refresh(&hiwdg);
    }
}
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint8_t u8TimeOut250ms;

void TimeOutReset(uint8_t nsec)
{
	if(nsec > 63) nsec = 63;
	if(++u8TimeOut250ms > nsec * 4)
	{
		u8TimeOut250ms = 0;
		sysReset();
	}
}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint16_t Ymodem_CRC(uint8_t * buf, uint16_t len)
{
    uint16_t chsum;
    uint16_t stat;
    uint16_t i;
    uint8_t * in_ptr;
    //指向要计算CRC的缓冲区开头
    in_ptr = buf;
    chsum = 0;
    for (stat = len ; stat > 0; stat--)     //len是所要计算的长度
    {
        chsum = chsum^(uint16_t)(*in_ptr++) << 8;
        for (i=8; i!=0; i--)
        {
            if (chsum & 0x8000)
            {
                chsum = chsum << 1 ^ 0x1021;
            }
            else
            {
                chsum = chsum << 1;
            }
        }
    }
    return chsum;
}
/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/


uint8_t u8Cnt1ms;
void Wait10msCountDwn(void)
{
    if(u16Count1ms) u16Count1ms--;
	if(++u8Cnt1ms >= 10)
	{
		u8Cnt1ms = 0;
		if(u16Wait10ms)
		{
			u16Wait10ms--;
		}
	}
}


//============================================================================

//============================================================================
void msg_updating()
{
	static uint8_t buf[] = "Updating......\r\n";
	// HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
    uart_tx_str((uint8_t *)buf,sizeof((char *)buf));
}
//============================================================================


//============================================================================
void msg_SlaveUpdateFail()
{
	static uint8_t buf[] = "\r\nFirmware Update fail! Time out!\r\n";
	// HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
    uart_tx_str((uint8_t *)buf,sizeof((char *)buf));
}
//============================================================================

//============================================================================
void msg_enter()
{
	static uint8_t buf[] = "\r\n";
    uart_tx_str((uint8_t *)buf,sizeof((char *)buf));
	// HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
}
//============================================================================
// Verify checkSum error!
//============================================================================
void msg_verifChksumError()
{
	static uint8_t buf[] = "Verify checkSum error!\r\n";
	uart_tx_str((uint8_t *)buf,sizeof((char *)buf));
}


const uint16_t TAB_YMODE_LEN[2] = {128, 1024};
uint8_t  u8EndState;
uint8_t  u8CntRxFlame;
uint32_t PragamerAddr;
volatile uint32_t u16FirmeareSize;
volatile uint32_t u16FirmeareChksum;
uint8_t  u8Program1K;

uint8_t YmodemReceiveDate(const uint32_t START_ADDR)
{
    uint16_t len;
    uint16_t temp;
    uint16_t i;
    uint8_t  state = 0;
    if((u8UartRxBuf[0] == MODEM_SOH) || (u8UartRxBuf[0] == MODEM_STX))
    {
        len = TAB_YMODE_LEN[u8UartRxBuf[0] - 1] + 5;
        if(u16Uart1RxIndex >= len)
        {
            u16Uart1RxIndex = 0;

            if((u8UartRxBuf[1] + u8UartRxBuf[2]) == 0xff)
            {
            if((u8EndState == 2) && (u8UartRxBuf[1] == 0))
            {
                state = 1;
                uart_tx_char(MODEM_ACK);
            }
            else
            {
                temp = u8UartRxBuf[len-2];
                temp <<= 8;
                temp += u8UartRxBuf[len-1];
                static uint16_t chk_crc;
                chk_crc = Ymodem_CRC(&u8UartRxBuf[3], len-5);
                if(temp == chk_crc) //Ymodem_CRC(&u8UartRxBuf[3], len-5))
                {
                if(u8TranState == 2)
                {
                    uint32_t uint32_tProgramSize;
                    uint32_tProgramSize = PragamerAddr - START_ADDR;
                    if(u16FirmeareSize > uint32_tProgramSize)
                    {
                        if((u16FirmeareSize - uint32_tProgramSize) >= FLASH_PAGE_SIZE)
                        {//大于等于2K
                            for(i=0; i<(len - 5); i++)
                            {
                                FlashWriteBuf[(len-5) * u8CntRxFlame + i] = u8UartRxBuf[3+i];
                            }
                            if(++u8CntRxFlame >= FLASH_PAGE_SIZE / (len - 5))
                            {
                                FlashPageWrite(PragamerAddr, FlashWriteBuf);
                                u16CntFlashPage++;
                                PragamerAddr += FLASH_PAGE_SIZE;
                                u8CntRxFlame = 0;
                            }
                        }
                        else
                        {//小于2K
                            if(u8Program1K == 0)
                            {
                                if((u16FirmeareSize - uint32_tProgramSize) >= 1024)
                                {//1K~2K
                                    for(i=0; i<1024; i++)
                                    {
                                        FlashWriteBuf[i] = u8UartRxBuf[3+i];
                                    }
                                    u8Program1K = 1;
                                }
                                else
                                {//<1K
                                    for(i=0; i<1024; i++)
                                    {
                                        FlashWriteBuf[i] = u8UartRxBuf[3+i];
                                    }
                                    FlashPageWrite(PragamerAddr, FlashWriteBuf);
                                }
                            }
                            else if(u8Program1K == 1)
                            {
                                for(i=0; i<1024; i++)
                                {
                                    FlashWriteBuf[1024 + i] = u8UartRxBuf[3+i];
                                }
                                //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);//Led1_pin//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);test
                                FlashPageWrite(PragamerAddr, FlashWriteBuf);
                            }
                        }
                    }
                }
                state = 1;
                }
                else
                {
                uart_tx_char(MODEM_NAK); //接收方crc校验出错,重传当前数据包请求
                }
            }
            u16Wait10ms = 25;
            }
        }
    }
    else if(u8UartRxBuf[0] == MODEM_EOT)
    {//接收到传输完成，发送NAK和ACK应答
        if(u8EndState == 0)
        {
            uart_tx_char(MODEM_NAK);

            u8EndState = 1;
        }
        else if(u8EndState == 1)
        {
            uart_tx_char(MODEM_ACK);
            delay_ms(2);
            //uart_tx_char(MODEM_C);
            u8EndState = 2;
        }
    }
    return state;
}



/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/


void Ymodem_Transmit(const uint32_t START_ADDR)
{
    uint32_t chksum;
    switch(u8TranState)
    {
        case 0:
            u16Uart1RxIndex = 0;
            u8YmodeType = 0;
            u8EndState = 0;
            u16CntFlashPage = 0;
            u8CntRxFlame = 0;
            for(uint16_t i=0; i<UART1BUF_SIZE; i++) u8UartRxBuf[i] = 0;
            uart_tx_char(MODEM_C);
            u16Wait10ms = 25;
            u8TranState = 1;
            break;
        case 1:
            if(u16Wait10ms)
            {
                if(YmodemReceiveDate(START_ADDR))
                {
                    if(u8UartRxBuf[1] == 0x00)
                    {
                        //首帧数据包含文件名及数据大小
                        uart_tx_char(MODEM_ACK);
                        delay_ms(2);
                        uart_tx_char(MODEM_C);
                        u16FirmeareSize = GetFirmwareSize(u8UartRxBuf);
                        u16FirmeareChksum = GetFirmwareChksum(u8UartRxBuf);
                        PragamerAddr = START_ADDR;
                        u8Program1K = 0;
                        u8TranState = 2;
                        u16Wait10ms = 25;
                    }
                }
            }
            else
            {
                u8TranState = 0; //超时继续发送“C”等待接收文件
            }
            break;
        case 2:
            if(u16Wait10ms)
            {
                if(YmodemReceiveDate(START_ADDR))
                {
                    uart_tx_char(MODEM_ACK);
                    u8TimeOut250ms = 0;
                }
            }
            else
            {
                if(u8EndState < 2)
                {
                    u16Uart1RxIndex = 0; //接收超时，接收计数清0
                    u16Wait10ms = 25;
                    uart_tx_char(MODEM_NAK); //重传当前数据包请求
                    TimeOutReset(10);
                }
                else
                {
                    msg_enter();
                    u8TranState = 3;
                }
            }
            break;
        case 3:
            if (((*(__IO uint32_t*)START_ADDR) & 0x2FFE0000 ) == 0x20000000)
            {
                chksum = CalcRomChksum(START_ADDR, u16FirmeareSize);
                if(chksum == chksum)    //(chksum == u16FirmeareChksum) 1087080/1082335
                {
                    uint8_t buf[30] ;
                    sprintf((char *)buf, "\r\nIAP success! Chksum %x:%x:%x,%x,%x,%x\r\n",chksum,START_ADDR,IAP_Get(bkp_app1_addr),IAP_Get(bkp_app1_mark),IAP_Get(bkp_app2_mark),IAP_Get(bkp_app2_addr));
                    // HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
                    uart_tx_str((uint8_t *)buf,sizeof(buf));
                    if(IAP_Get(bkp_app1_addr) == START_ADDR) IAP_Set(bkp_app1_mark,chksum);
                    else if(IAP_Get(bkp_app2_addr) == START_ADDR)  IAP_Set(bkp_app2_mark,chksum);
                    sysReset();
                }
                else
                {
                    msg_verifChksumError();
                    u8TranState = 4;        //0 重新发起接收请求
                }
            }
            else
            {
                static uint8_t buf[] = "Verify error!\r\n";
                uart_tx_str(buf,sizeof(buf));
                u8TranState = 4;    //0 重新发起接收请求
            }
            break;

        case 4:
        break;

        default:
        break;
    }
}

//============================================================================

//============================================================================
uint8_t CharToHex(uint8_t str)
{
	uint8_t num;
	num = 0;
	if(str >= 0x30 && str <= 0x39)
	{
		num = str - 0x30;
	}
	else if(str >= 0x41 && str <= 0x46)
	{
		num = str - 0x37;
	}
	else if(str >= 0x61 && str <= 0x66)
	{
		num = str - 0x57;
	}

	return num;
}
//============================================================================

//============================================================================
uint16_t GetRxBufSize(uint8_t *rxbuf)
{
	uint16_t size;
	size = 0;
	if(rxbuf[0] == MODEM_SOH)
	{
		size = 128;
	}
	else if(rxbuf[0] == MODEM_STX)
	{
		size = 1024;
	}

	return size;
}
//============================================================================

//============================================================================
uint32_t GetFirmwareSize(uint8_t *rxbuf)
{
	uint16_t i,index;
	uint32_t get_size;
	index = 0;
	get_size = 0;
	for(i=3; i<GetRxBufSize(rxbuf); i++)
	{
		if(index == 1)
		{
			get_size *= 10;
			get_size += CharToHex(rxbuf[i]);
		}
		else if(index >= 2)
		{
			get_size /= 10;
			break;
		}

		if(rxbuf[i] == 0x00)
		{
			index++;
		}
	}

	return get_size;
}
//============================================================================

//============================================================================
uint32_t GetFirmwareChksum(uint8_t *rxbuf)
{
	uint16_t i,index;
	uint32_t chksum;
	index = 0;
	chksum = 0;
	for(i=3; i<GetRxBufSize(rxbuf); i++)
	{
		if(index == 2)
		{
			chksum <<= 4;
			chksum += CharToHex(rxbuf[i]);
		}
		else if(index >= 3)
		{
			chksum >>= 4;
			break;
		}

		if(rxbuf[i] == 0x00)
		{
			index++;
		}
	}
	return chksum;
}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint32_t CalcRomChksum(uint32_t address, uint32_t length)
{
	uint32_t i,chksum;
	chksum = 0;
	for(i=0; i<length; i++)
	{
		chksum += *(uint8_t *)address;
		address++;
	}
	return chksum;
}

/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
