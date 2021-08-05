/**
  ******************************************************************************
  * @author  https://gzwelink.taobao.com
  * @version V1.0
  * @date    2020.01.01
  * @brief   RT-Thread系统实验
  ******************************************************************************
  * @attention
  * 实验平台:MINI-STM32G030C8开发板 
  ******************************************************************************
  */

#include <rtthread.h>
#include "main.h"

#define RT_LED1_THREAD_PRIORITY       (RT_THREAD_PRIORITY_MAX / 4)
#define RT_LED2_THREAD_PRIORITY       (RT_THREAD_PRIORITY_MAX / 5)
void LED1Main(void *parameter)
{
while(1)
{
		rt_thread_delay(200);//50ms
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET);
}


}

void LED2Main(void *parameter)
{
while(1)
{
	rt_thread_delay(50);//200ms
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET);
}


}

void MX_RT_Thread_Init(void)
{
	    rt_thread_t tid;
	    tid = rt_thread_create("LED1", LED1Main, RT_NULL,
                           128, RT_LED1_THREAD_PRIORITY, 20);
    RT_ASSERT(tid != RT_NULL);
    rt_thread_startup(tid);
	    tid = rt_thread_create("LED2", LED2Main, RT_NULL,
                           128, RT_LED2_THREAD_PRIORITY, 20);
    RT_ASSERT(tid != RT_NULL);
    rt_thread_startup(tid);

}

void MX_RT_Thread_Process(void)
{
rt_thread_delay(1* RT_TICK_PER_SECOND);
}

