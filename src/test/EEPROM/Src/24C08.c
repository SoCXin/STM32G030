#include "main.h"

#define	EE_ADDR 0xa0




#define EE_SCL_PIN  GPIO_PIN_10   //ģ��IIC��SCL�ź�  1.�޸����ż����޸�IIC�ӿ�
#define EE_SDA_PIN  GPIO_PIN_11   //ģ��IIC��SDA�ź�

void EE_SDA_IN(void) 	//���ó�����
	{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void EE_SDA_OUT(void)//���ó����
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void EE_SCK_OUT(void) //���ó����
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


#define EE_IIC_SCL(val)         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,val)                   //SCL      2.�޸����ż����޸�IIC�ӿ�
#define EE_IIC_SDA(val)         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,val)                    //SDA






unsigned char EE_READ_SDA(void)
{
return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11);
}



/******************************************************************************
*��  ����void EE_IIC_Delay(void)
*�����ܣ�IIC��ʱ
*��  ������
*����ֵ����
*��  ע: ��ֲʱֻ��Ҫ��EE_IIC_Delay()�����Լ�����ʱ����
*******************************************************************************/
void EE_IIC_Delay(uint8_t us)
{
        for(int i = 0; i < 20; i++)
        {
            __asm("NOP");//core bus 160M  ����´��IIC���� 400K
        }

}
/******************************************************************************
*��  ����void IIC_Init(void)
*�����ܣ�IIC��ʼ��
*��  ������
*����ֵ����
*��  ע����
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
	EE_SDA_OUT(); //sda�����
	EE_IIC_SDA(1);
	EE_IIC_SCL(1);
	EE_IIC_Delay(4);
 	EE_IIC_SDA(0); //START:when CLK is high,DATA change form high to low
	EE_IIC_Delay(4);
	EE_IIC_SCL(0); //ǯסI2C���ߣ�׼�����ͻ��������
}


void EE_IIC_Stop(void)
{
	EE_SDA_OUT(); //sda�����
	EE_IIC_SCL(0);
	EE_IIC_SDA(0); //STOP:when CLK is high DATA change form low to high
        EE_IIC_Delay(4);
	EE_IIC_SCL(1);
	EE_IIC_SDA(1); //����I2C���߽����ź�
        EE_IIC_Delay(4);
}


uint8_t EE_IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	EE_SDA_IN(); //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ��
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
	EE_IIC_SCL(0); //ʱ�����0
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
    EE_IIC_SCL(0); //����ʱ�ӿ�ʼ���ݴ���
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
	EE_SDA_IN(); //SDA����Ϊ����ģʽ �ȴ����մӻ���������
    for(i=0;i<8;i++ )
	{
        EE_IIC_SCL(0);
        EE_IIC_Delay(1);
        EE_IIC_SCL(1);
        receive<<=1;
        if(EE_READ_SDA())receive++; //�ӻ����͵ĵ�ƽ
        EE_IIC_Delay(1);
    }
    if(ack)
        EE_IIC_Ack(); //����ACK
    else
        EE_IIC_NAck(); //����nACK
    return receive;
}


uint8_t EE_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	EE_IIC_Start();
	EE_IIC_SendByte(I2C_Addr);	 //���ʹӻ���ַ
	if(EE_IIC_WaitAck()) //����ӻ�δӦ�������ݷ���ʧ��
	{
		EE_IIC_Stop();
		return 1;
	}
	EE_IIC_SendByte(reg); //���ͼĴ�����ַ
	EE_IIC_WaitAck();

	EE_IIC_Start();
	EE_IIC_SendByte(I2C_Addr+1); //�������ģʽ
	EE_IIC_WaitAck();
	*buf=EE_IIC_ReadByte(0);
        EE_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}


uint8_t EE_EE_IIC_SendByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	EE_IIC_Start();
	EE_IIC_SendByte(I2C_Addr); //���ʹӻ���ַ
	if(EE_IIC_WaitAck())
	{
		EE_IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	EE_IIC_SendByte(reg); //���ͼĴ�����ַ
        EE_IIC_WaitAck();
	EE_IIC_SendByte(data);
	if(EE_IIC_WaitAck())
	{
		EE_IIC_Stop();
		return 1; //����д��ʧ��
	}
	EE_IIC_Stop(); //����һ��ֹͣ����

	return 0;
}


