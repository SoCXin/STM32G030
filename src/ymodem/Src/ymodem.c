/*******************************************************************************
  * @file    ymodem_daxia.c
  * @author  
  * @version V2.0.2
  * @date    2020.11.02
  * @brief   Y-Modem Э���ļ�����
  *****************************************************************************/
#include "main.h"
#include "bootloader.h"
#include "ymodem.h"
#include "uart.h"
#include "flash.h"

uint8_t  u8YmodeType; //SOH(128) or STX(1024)
uint8_t  u8TranState;
uint16_t u16Wait10ms;
uint16_t u16CntFlashPage;

//============================================================================
uint16_t Y_Modem_CRC(uint8_t * buf, uint16_t len)
{
    uint16_t chsum;
    uint16_t stat;
    uint16_t i;
    uint8_t * in_ptr;
   
    //ָ��Ҫ����CRC�Ļ�������ͷ
    in_ptr = buf;
    chsum = 0;
    for (stat = len ; stat > 0; stat--) //len����Ҫ����ĳ���
    {
        chsum = chsum^(uint16_t)(*in_ptr++) << 8;
        for (i=8; i!=0; i--) {
            if (chsum & 0x8000){
                chsum = chsum << 1 ^ 0x1021;
            } else {
                chsum = chsum << 1;
            }
        }
    }
    return chsum;
}
//============================================================================

uint16_t u16Count1ms;
//============================================================================
void delay_ms(uint16_t ms)
{
	u16Count1ms = ms;
	while(u16Count1ms)
	{
		HAL_IWDG_Refresh(&hiwdg);
	}
}
//============================================================================

//============================================================================
void ms_count_down(void)
{
	if(u16Count1ms)
	{
		u16Count1ms--;
	}
}
//============================================================================

//============================================================================
void Send_CMD(uint8_t cmd)
{
	uint8_t buf[2];
	buf[0] = cmd;
	buf[1] = 0;
	HAL_UART_Transmit(&huart2,buf,sizeof(buf)-1,10);
}
//============================================================================

//============================================================================
void txDownloadSuccess()
{
	static uint8_t buf[] = "Firmware download success! Restarting......\r\n";
	HAL_UART_Transmit(&huart2,buf,sizeof(buf)-1,10);
}
//============================================================================

//============================================================================
void msg_updating()
{
	static uint8_t buf[] = "Updating......\r\n";
	HAL_UART_Transmit(&huart2,buf,sizeof(buf)-1,10);
}
//============================================================================


//============================================================================
void msg_SlaveUpdateFail()
{
	static uint8_t buf[] = "\r\nFirmware Update fail! Time out!\r\n";
	HAL_UART_Transmit(&huart2,buf,sizeof(buf)-1,10);
}
//============================================================================

//============================================================================
void msg_enter()
{
	static uint8_t buf[] = "\r\n";
	HAL_UART_Transmit(&huart2,buf,sizeof(buf)-1,10);
}
//============================================================================

//============================================================================
void msg_verifChksumError()
{
	static uint8_t buf[] = "Verify checkSum error!\r\n";
	HAL_UART_Transmit(&huart2,buf,sizeof(buf)-1,10);
}
//============================================================================

//=========================================================================
uint8_t  FlashWriteBuf[FLASH_PAGE_SIZE];

//============================================================================
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
          Send_CMD(MODEM_ACK);
        }   
        else
        {
          temp = u8UartRxBuf[len-2];
          temp <<= 8;
          temp += u8UartRxBuf[len-1];
					static uint16_t chk_crc;
          chk_crc = Y_Modem_CRC(&u8UartRxBuf[3], len-5);
          if(temp == chk_crc) //Y_Modem_CRC(&u8UartRxBuf[3], len-5))
          {
            if(u8TranState == 2)
            {							
							uint32_t u32ProgramSize;
							u32ProgramSize = PragamerAddr - START_ADDR;			
							if(u16FirmeareSize > u32ProgramSize)
							{
								if((u16FirmeareSize - u32ProgramSize) >= FLASH_PAGE_SIZE)
								{//���ڵ���2K
									for(i=0; i<(len - 5); i++)
									{
										FlashWriteBuf[(len-5) * u8CntRxFlame + i] = u8UartRxBuf[3+i];
									}
									
									if(++u8CntRxFlame >= FLASH_PAGE_SIZE / (len - 5))
									{
										//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);//Led1_pin//HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);test										
										FlashPageWrite(PragamerAddr, FlashWriteBuf);									
										u16CntFlashPage++;	
										PragamerAddr += FLASH_PAGE_SIZE;
										u8CntRxFlame = 0;										
									}			
								}
								else 
								{//С��2K
									if(u8Program1K == 0)
									{
										if((u16FirmeareSize - u32ProgramSize) >= 1024)
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
											//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);//Led1_pin//HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);test
											FlashPageWrite(PragamerAddr, FlashWriteBuf);
										}
									}		
									else if(u8Program1K == 1)
									{
										for(i=0; i<1024; i++)
										{
											FlashWriteBuf[1024 + i] = u8UartRxBuf[3+i];											
										}	
										//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);//Led1_pin//HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);test
										FlashPageWrite(PragamerAddr, FlashWriteBuf); 
									}										
								}
							}
            }
                     
            state = 1;
          }
          else
          {
            Send_CMD(MODEM_NAK); //���շ�crcУ�����,�ش���ǰ���ݰ�����
          }
        }
               
        u16Wait10ms = 25;
      }
    }
  }
  else if(u8UartRxBuf[0] == MODEM_EOT)
  {//���յ�������ɣ�����NAK��ACKӦ��
    if(u8EndState == 0)
    {      
      Send_CMD(MODEM_NAK);
			
      u8EndState = 1;
    }
    else if(u8EndState == 1)
    {            
      Send_CMD(MODEM_ACK);
      delay_ms(2);
      //Send_CMD(MODEM_C);
      u8EndState = 2;
    }
  }
    
  return state;
}
//============================================================================

//============================================================================
void McuReset(void)
{
	// �ر������ж�
	__disable_irq();//__set_FAULTMASK(1);//__set_FAULTMASK(1); //ִ��NVIC_SystemReset()��������������ϣ����Թ����ж�
	// ��λ
	NVIC_SystemReset();
}
//============================================================================

uint8_t u8TimeOut250ms;
//============================================================================
void TimeOutReset(uint8_t nsec)
{
	if(nsec > 63) nsec = 63;
	if(++u8TimeOut250ms > nsec * 4)
	{
		u8TimeOut250ms = 0;
		McuReset();
	}
}
//============================================================================

//============================================================================
uint32_t chksum;

void Ymodem_Transmit(const uint32_t START_ADDR)
{
  switch(u8TranState)
  {
    case 0:      
      u16Uart1RxIndex = 0;
      u8YmodeType = 0;
      u8EndState = 0;
      u16CntFlashPage = 0;
			u8CntRxFlame = 0;
			ClrUartRxBuf();
      Send_CMD(MODEM_C);
      u16Wait10ms = 25;
      u8TranState = 1;
    break;
    
    case 1:
      if(u16Wait10ms)
      {
        if(YmodemReceiveDate(START_ADDR))
        {
          if(u8UartRxBuf[1] == 0x00)
          {//��֡���ݰ����ļ��������ݴ�С
            Send_CMD(MODEM_ACK);
            delay_ms(2);
            Send_CMD(MODEM_C);
            
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
        u8TranState = 0; //��ʱ�������͡�C���ȴ������ļ�
      }      
    break;
    
    case 2:           
      if(u16Wait10ms)
      {
        if(YmodemReceiveDate(START_ADDR))
        {
          Send_CMD(MODEM_ACK);
					u8TimeOut250ms = 0;
        }  				
      }
      else 
      {
        if(u8EndState < 2)
        {
          u16Uart1RxIndex = 0; //���ճ�ʱ�����ռ�����0
          u16Wait10ms = 25;
          Send_CMD(MODEM_NAK); //�ش���ǰ���ݰ�����
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
			if (((*(__IO uint32_t*)USER_APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
			{		
        chksum = CalcRomChksum(USER_APP_ADDRESS, u16FirmeareSize);
				if(chksum == u16FirmeareChksum)
				{//����ͷ��͵�У�������������������					
					txDownloadSuccess();
					//u8TranState = 4; //�����������
					delay_ms(500);
					McuReset();
				}
				else
				{
					msg_verifChksumError();
					u8TranState = 4; //0 ���·����������
				}
			}
			else
			{
				static uint8_t buf[] = "Data Verify error!\r\n";
				HAL_UART_Transmit(&huart2,buf,sizeof(buf)-1,10);
				u8TranState = 4; //0 ���·����������
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
uint8_t u8Cnt1ms;
void Wait10msCountDwn(void)
{
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
//============================================================================

//============================================================================
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
//============================================================================

//============================================================================

//============================================================================

//============================================================================