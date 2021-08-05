#include "main.h"

#define	EE_ADDR 0xa0




#define EE_SCL_PIN  GPIO_PIN_10   //模拟IIC的SCL信号  1.修改引脚即可修改IIC接口
#define EE_SDA_PIN  GPIO_PIN_11   //模拟IIC的SDA信号

void EE_SDA_IN(void) 	//配置成输入
	{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void EE_SDA_OUT(void)//配置成输出
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void EE_SCK_OUT(void) //配置成输出
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


#define EE_IIC_SCL(val)         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,val)                   //SCL      2.修改引脚即可修改IIC接口
#define EE_IIC_SDA(val)         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,val)                    //SDA






unsigned char EE_READ_SDA(void)
{
return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
}



/******************************************************************************
*函  数：void EE_IIC_Delay(void)
*功　能：IIC延时
*参  数：无
*返回值：无
*备  注: 移植时只需要将EE_IIC_Delay()换成自己的延时即可
*******************************************************************************/
void EE_IIC_Delay(uint8_t us)
{
        for(int i = 0; i < 20; i++)
        {
            __asm("NOP");//core bus 160M  情况下大概IIC速率 400K
        }

}
/******************************************************************************
*函  数：void IIC_Init(void)
*功　能：IIC初始化
*参  数：无
*返回值：无
*备  注：无
*******************************************************************************/

void EE_IIC_Init(void)
{


    EE_SCK_OUT();
    EE_SDA_OUT();
    EE_IIC_SCL(1);
    EE_IIC_SDA(1);



}

void EE_IIC_Start(void)
{
	EE_SDA_OUT(); //sda线输出
	EE_IIC_SDA(1);
	EE_IIC_SCL(1);
	EE_IIC_Delay(4);
 	EE_IIC_SDA(0); //START:when CLK is high,DATA change form high to low
	EE_IIC_Delay(4);
	EE_IIC_SCL(0); //钳住I2C总线，准备发送或接收数据
}


void EE_IIC_Stop(void)
{
	EE_SDA_OUT(); //sda线输出
	EE_IIC_SCL(0);
	EE_IIC_SDA(0); //STOP:when CLK is high DATA change form low to high
        EE_IIC_Delay(4);
	EE_IIC_SCL(1);
	EE_IIC_SDA(1); //发送I2C总线结束信号
        EE_IIC_Delay(4);
}


uint8_t EE_IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	EE_SDA_IN(); //SDA设置为输入  （从机给一个低电平做为应答）
	EE_IIC_SDA(1);EE_IIC_Delay(1);
	EE_IIC_SCL(1);EE_IIC_Delay(1);;
	while(EE_READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			EE_IIC_Stop();
			return 1;
		}
	}
	EE_IIC_SCL(0); //时钟输出0
	return 0;
}



void EE_IIC_Ack(void)
{
	EE_IIC_SCL(0);
	EE_SDA_OUT();
	EE_IIC_SDA(0);
	EE_IIC_Delay(1);
	EE_IIC_SCL(1);
	EE_IIC_Delay(2);
	EE_IIC_SCL(0);
}


void EE_IIC_NAck(void)
{
	EE_IIC_SCL(0);
	EE_SDA_OUT();
	EE_IIC_SDA(1);
	EE_IIC_Delay(1);
	EE_IIC_SCL(1);
	EE_IIC_Delay(1);
	EE_IIC_SCL(0);
}


void EE_IIC_SendByte(uint8_t data)
{
    uint8_t t;
    EE_SDA_OUT();
    EE_IIC_SCL(0); //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        EE_IIC_SDA((data&0x80)>>7);
        EE_IIC_Delay(1);
        EE_IIC_SCL(1);
        data<<=1;
        EE_IIC_Delay(1);
        EE_IIC_SCL(0);
    }
    EE_IIC_Delay(1);
}


uint8_t EE_IIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
	EE_SDA_IN(); //SDA设置为输入模式 等待接收从机返回数据
    for(i=0;i<8;i++ )
	{
        EE_IIC_SCL(0);
        EE_IIC_Delay(1);
        EE_IIC_SCL(1);
        receive<<=1;
        if(EE_READ_SDA())receive++; //从机发送的电平
        EE_IIC_Delay(1);
    }
    if(ack)
        EE_IIC_Ack(); //发送ACK
    else
        EE_IIC_NAck(); //发送nACK
    return receive;
}


uint8_t EE_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	EE_IIC_Start();
	EE_IIC_SendByte(I2C_Addr);	 //发送从机地址
	if(EE_IIC_WaitAck()) //如果从机未应答则数据发送失败
	{
		EE_IIC_Stop();
		return 1;
	}
	EE_IIC_SendByte(reg); //发送寄存器地址
	EE_IIC_WaitAck();

	EE_IIC_Start();
	EE_IIC_SendByte(I2C_Addr+1); //进入接收模式
	EE_IIC_WaitAck();
	*buf=EE_IIC_ReadByte(0);
        EE_IIC_Stop(); //产生一个停止条件
	return 0;
}


uint8_t EE_EE_IIC_SendByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	EE_IIC_Start();
	EE_IIC_SendByte(I2C_Addr); //发送从机地址
	if(EE_IIC_WaitAck())
	{
		EE_IIC_Stop();
		return 1; //从机地址写入失败
	}
	EE_IIC_SendByte(reg); //发送寄存器地址
        EE_IIC_WaitAck();
	EE_IIC_SendByte(data);
	if(EE_IIC_WaitAck())
	{
		EE_IIC_Stop();
		return 1; //数据写入失败
	}
	EE_IIC_Stop(); //产生一个停止条件

	return 0;
}


