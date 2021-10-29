
#include "main.h"
#include "bootloader.h"
#include "flash.h"
#include "ymodem.h"


/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint8_t  FlashPageRead(uint32_t address, uint8_t *pbuf)
{
    uint32_t i = 0;
    uint32_t readbuf;
    uint8_t  state = 0;
    if((address < FLASH_MARK_BASE) || ((address - FLASH_START_BASE) % FLASH_PAGE_SIZE))
    {
        return state;	//非法地址
    }
    while(i<FLASH_PAGE_SIZE)
    {
        readbuf = *(uint32_t*)(address);	//读取4个字节.
        pbuf[i++] = readbuf;
        pbuf[i++] = readbuf >> 8;
        pbuf[i++] = readbuf >> 16;
        pbuf[i++] = readbuf >> 24;
        address += 4;									        //偏移4个字节.
    }
    state = 1;
    return state;
}

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
uint16_t u16FlashProgState;
uint8_t FlashPageWrite(uint32_t address, uint8_t *pbuf)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    uint32_t PageError = 0;
    uint8_t  state = 0;
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
            uint32_t addr_index = 0;
            u16FlashProgState = 0;
            while(addr_index < FLASH_PAGE_SIZE)									//写数据
            {
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

/******************************************************************************
**函数信息 ：
**功能描述 ：
**输入参数 ：无
**输出参数 ：无
*******************************************************************************/
void FlashReset(uint8_t bank)
{
    for(uint16_t i=0; i<FLASH_PAGE_SIZE; i++)
    {
        FlashWriteBuf[i] = 0xff;
    }
    FlashPageWrite(bank * FLASH_PAGE_SIZE, FlashWriteBuf);
    // for(i=0; i<FLASH_PAGE_SIZE; i++)
    // {
    // 	FlashWriteBuf[i] = 0;
    // }
    // FlashPageRead(USER_APP1_ADDRESS, FlashWriteBuf);
}


/*------------------------- (C) COPYRIGHT 2021 OS-Q --------------------------*/
