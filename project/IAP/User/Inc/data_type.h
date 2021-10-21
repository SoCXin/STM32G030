
#ifndef _TYPE_H_001
#define _TYPE_H_001

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_system.h"

typedef uint8_t   u8;
typedef int8_t    s8;
typedef uint16_t u16;
typedef int16_t  s16;
typedef uint32_t u32;
typedef int32_t  s32;
typedef uint64_t u64;
typedef int64_t  s64;
typedef volatile unsigned int vu32;
//===============================================
//    type define
//===============================================
typedef struct
{	u8  bit0	:1;
	u8  bit1	:1;
	u8  bit2	:1;
	u8  bit3	:1;
	u8  bit4	:1;
	u8  bit5	:1;
	u8  bit6	:1;
	u8  bit7	:1;
} BYTE_FIELD;
//===============================================
typedef union
{	u8 byte;
	BYTE_FIELD 	bitn;
} TYPE_BYTE_BIT;
//===============================================
typedef struct
{	u8  bit8	:1;
	u8  bit9	:1;
	u8  bit10	:1;
	u8  bit11	:1;
	u8  bit12	:1;
	u8  bit13	:1;
	u8  bit14	:1;
	u8  bit15 :1;

	u8  bit0	:1;
	u8  bit1	:1;
	u8  bit2	:1;
	u8  bit3	:1;
	u8  bit4	:1;
	u8  bit5	:1;
	u8  bit6	:1;
	u8  bit7	:1;
} WORD_FIELD;
//===============================================
typedef union
{
	u16     word;
	u8	    byte[2];
	WORD_FIELD 	b;
} TYPE_WORD_BIT;


typedef union
{
	u16 word;
	struct my_byte_t
	{
		u8 l;
		u8 h;
	}byte;
}WORD_BYTE;

typedef struct
{
	u8 hour;
	u8 minute;
	u8 second;
}tClockRun;

typedef struct
{
	u8 hour;
	u8 minute;
}tClockSet;


typedef union
{
	u8 array[4];
	float F;
}Union_Array_Float;

typedef struct
{
	Union_Array_Float Out;
	Union_Array_Float LastError;
	Union_Array_Float LastError2;
}STRUCT_PID;


#ifdef __cplusplus
}
#endif

#endif

