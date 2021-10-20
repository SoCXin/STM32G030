/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
uint8_t u8Uart1RxBuf;
uint16_t uwExpectedCRCValue = 0xCB92; //0xCB91;
__IO uint16_t uwCRCValue = 0;
__IO uint16_t testa = 0;
#define BUFFER_SIZE    14
static const uint8_t aDataBuffer[BUFFER_SIZE] =
{
  0x1, 0x2, 0x3, 0x4, 0x5, 0x6,0x7, 0x8, 0x9, 0xa, 0xb, 0xc,0xd, 0xe,
};

__IO uint32_t tmp_index = 0;

void HAL_SYSTICK_Callback(void)
{
  tmp_index++;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Program Size: Code=7136 RO-data=352 RW-data=20 ZI-data=1908

uint8_t  u8YmodeType; //SOH(128) or STX(1024)
uint8_t  u8TranState;
uint16_t u16Wait10ms;
uint16_t u16CntFlashPage;
uint32_t chksum;
#define FLASH_START_BASE    0x08000000
#define USER_APP_ADDRESS    0x08004000
#define APP_START_PAGE      ((USER_APP_ADDRESS - FLASH_START_BASE) / FLASH_PAGE_SIZE)

// void Ymodem_loop(void)
// {
//     uint32_t START_ADDR= USER_APP_ADDRESS;
//     switch(u8TranState)
//     {
//         case 0:
//             u16Uart1RxIndex = 0;
//             u8YmodeType = 0;
//             u8EndState = 0;
//             u16CntFlashPage = 0;
//             u8CntRxFlame = 0;
//             ClrUartRxBuf();
//             Send_CMD(MODEM_C);
//             u16Wait10ms = 25;
//             u8TranState = 1;
//             break;
//         case 1:
//             if(u16Wait10ms)
//             {
//                 if(YmodemReceiveDate(START_ADDR))
//                 {
//                     if(u8UartRxBuf[1] == 0x00)
//                     {
//                         Send_CMD(MODEM_ACK);
//                         delay_ms(2);
//                         Send_CMD(MODEM_C);
//                         u16FirmeareSize = GetFirmwareSize(u8UartRxBuf);
//                         u16FirmeareChksum = GetFirmwareChksum(u8UartRxBuf);
//                         PragamerAddr = START_ADDR;
//                         u8Program1K = 0;
//                         u8TranState = 2;
//                         u16Wait10ms = 25;
//                     }
//                 }
//             }
//             else
//             {
//                 u8TranState = 0;
//             }
//             break;

//         case 2:
//             if(u16Wait10ms)
//             {
//                 if(YmodemReceiveDate(START_ADDR))
//                 {
//                     Send_CMD(MODEM_ACK);
//                     u8TimeOut250ms = 0;
//                 }
//             }
//             else
//             {
//                 if(u8EndState < 2)
//                 {
//                     u16Uart1RxIndex = 0;
//                     u16Wait10ms = 25;
//                     Send_CMD(MODEM_NAK);
//                     TimeOutReset(10);
//                 }
//                 else
//                 {
//                     msg_enter();
//                     u8TranState = 3;
//                 }
//             }
//             break;

//         case 3:
//             if (((*(__IO uint32_t*)USER_APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
//             {
//                 chksum = CalcRomChksum(USER_APP_ADDRESS, u16FirmeareSize);
//                 if(chksum == chksum)    //(chksum == u16FirmeareChksum)
//                 {
//                     //如果和发送的校验和相等则重启完成升级
//                     uint8_t buf[20] ;
//                     sprintf((char *)buf, "\r\nChksum:%d,%d\r\n",chksum,u16FirmeareChksum);
//                     HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
//                     txDownloadSuccess();
//                     //u8TranState = 4; //程序下载完成
//                     if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 1234) HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,1234);
//                     // delay_ms(500);
//                     McuReset();
//                 }
//                 else
//                 {
//                     msg_verifChksumError();
//                     u8TranState = 4;        //0 重新发起接收请求
//                 }
//             }
//             else
//             {
//                 static uint8_t buf[] = "Verify error!\r\n";
//                 HAL_UART_Transmit(&huart1,buf,sizeof(buf)-1,10);
//                 u8TranState = 4; //0 重新发起接收请求
//             }
//             break;

//         case 4:
//         break;

//         default:
//         break;
//     }
// }

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
	LL_USART_EnableIT_RXNE(USART1);
	LL_USART_EnableIT_PE(USART1);
  uint8_t buf[] = "\r\nSTM32G030 UART1(115200) TEST\r\n";
  for(uint8_t i=0;i<sizeof(buf)-1;i++)
  {
    LL_USART_TransmitData9(USART1, buf[i]);
    while(!LL_USART_IsActiveFlag_TXE(USART1));
  }
  for(uint8_t i=0;i<BUFFER_SIZE;i++)
  {
    LL_CRC_FeedData8(CRC, aDataBuffer[i]);
  }
    testa = LL_CRC_ReadData16(CRC);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    LL_mDelay(500);
    memset((char *)buf,0,sizeof(buf));
    // uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)aDataBuffer, BUFFER_SIZE);
    sprintf((char *)buf, "CRC%d:%x,%x\r\n",tmp_index,testa,(uint16_t)((uwCRCValue>>8) | (uwCRCValue<<8)));
    // HAL_UART_Transmit(&huart1,buf,strlen((char *)buf),100);
    for(uint8_t i=0;i<strlen((char *)buf);i++)
    {
      LL_USART_TransmitData8(USART1, buf[i]);
      while(!LL_USART_IsActiveFlag_TXE(USART1));
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* LSI configuration and activation */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }

  LL_PWR_EnableBkUpAccess();
  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  }
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(64000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
  LL_RCC_EnableRTC();
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  LL_CRC_SetInputDataReverseMode(CRC, LL_CRC_INDATA_REVERSE_BYTE);
  LL_CRC_SetOutputDataReverseMode(CRC, LL_CRC_OUTDATA_REVERSE_BIT);
  LL_CRC_SetInitialData(CRC, 0xFFFF);
  LL_CRC_SetPolynomialCoef(CRC, 32773);
  LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_16B);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTC);

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_EnableAutoBaudRate(USART1);
  LL_USART_SetAutoBaudRateMode(USART1, LL_USART_AUTOBAUD_DETECT_ON_STARTBIT);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
