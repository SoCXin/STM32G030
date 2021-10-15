/**
  ******************************************************************************
  * @file           : uart.c
  * @brief          : uart sub program body
  ******************************************************************************
 */
 
#include "main.h"
#include "bootloader.h"
#include "flash.h"
#include "ymodem.h"

//============================================================================


//============================================================================
u32 STMFLASH_ReadWord(u32 addr)
{
	return *(vu32*)addr; 
}
//============================================================================

//============================================================================

u8  FlashPageRead(u32 address, u8 *pbuf)
{
	u32 i;
	u32 readbuf;
	
	u8  state = 0;
	if((address < FLASH_MARK_BASE) || ((address - FLASH_START_BASE) % FLASH_PAGE_SIZE))
	{		
		return state;	//非法地址
	}
	
	i = 0;	
	while(i<FLASH_PAGE_SIZE)
	{
		readbuf = STMFLASH_ReadWord(address);	//读取4个字节.
		pbuf[i++] = readbuf;
		pbuf[i++] = readbuf >> 8;
		pbuf[i++] = readbuf >> 16;
		pbuf[i++] = readbuf >> 24;
		address += 4;									        //偏移4个字节.
	}
	
	state = 1;
	return state;
}
//============================================================================

//============================================================================
#define DATA_64   ((uint64_t)0x1234567887654321)
u16 u16FlashProgState;
u8 FlashPageWrite(u32 address, u8 *pbuf)
{
	FLASH_EraseInitTypeDef FlashEraseInit;
	//HAL_StatusTypeDef FlashStatus = HAL_OK;
	u32 PageError = 0;
	u8  state = 0;

	if((address < FLASH_MARK_BASE) || ((address - FLASH_START_BASE) % FLASH_PAGE_SIZE))
	{		
		return state;	//非法地址
	}
	
	HAL_FLASH_Unlock();       //解锁	
	FlashEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;    //擦除类型，页擦除 
	FlashEraseInit.Page = (address - FLASH_START_BASE) / FLASH_PAGE_SIZE;   						   //从哪页开始擦除
	FlashEraseInit.NbPages = 1;               //一次只擦除一页
	if(HAL_FLASHEx_Erase(&FlashEraseInit,&PageError) != HAL_OK) 
	{
		HAL_FLASH_Lock();
		return state;//发生错误了	
	}
	else
	{
		if(FLASH_WaitForLastOperation(FLASH_WAITETIME) == HAL_OK) ////等待上次操作完成
		{
			uint64_t u64buffer;			
			u32 addr_index = 0;
			u16FlashProgState = 0;
			while(addr_index < FLASH_PAGE_SIZE)									//写数据
			{
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);//Led1_pin//HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);test

				//u64buffer = DATA_64;//*(uint64_t*)pbuf; //
					u64buffer = pbuf[addr_index + 7];
					u64buffer <<= 8;
					u64buffer += pbuf[addr_index + 6];
					u64buffer <<= 8;
					u64buffer += pbuf[addr_index + 5];
					u64buffer <<= 8;
					u64buffer += pbuf[addr_index + 4];
					u64buffer <<= 8;
					u64buffer += pbuf[addr_index + 3];
					u64buffer <<= 8;
					u64buffer += pbuf[addr_index + 2];
					u64buffer <<= 8;
					u64buffer += pbuf[addr_index + 1];
					u64buffer <<= 8;
					u64buffer += pbuf[addr_index + 0];

				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + addr_index, u64buffer)== HAL_OK)//写入数据
				{
					addr_index += 8;
					
				}
				else
				{ 
					HAL_FLASH_Lock();
					break;												//写入异常
				}				
			}  
			
			//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);//Led1_pin//HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);test
			state = 1;
		}
		else
		{
			state = 0;
		}
				
		HAL_FLASH_Lock();
		return state;
	}
}
//============================================================================

//============================================================================
void FlashTestWR(void)
{
	uint16_t i,j;
	j = 255;
	for(i=0; i<FLASH_PAGE_SIZE; i++)
	{
		FlashWriteBuf[i] = j;
		if(j)
		{
			j--;
		}
		else
		{
			j = 255;
		}
	}
	
	FlashPageWrite(USER_APP_ADDRESS, FlashWriteBuf);
	for(i=0; i<FLASH_PAGE_SIZE; i++)
	{
		FlashWriteBuf[i] = 0;
	}
	FlashPageRead(USER_APP_ADDRESS, FlashWriteBuf);
}
//============================================================================
