/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32wb0x_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void lowPowerIOSetup(void);
/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* IOs configuration to have the lowest power consumption in DEEPSTOP */
  lowPowerIOSetup();

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief PKA MSP Initialization
* This function configures the hardware resources used in this example
* @param hpka: PKA handle pointer
* @retval None
*/
void HAL_PKA_MspInit(PKA_HandleTypeDef* hpka)
{
  if(hpka->Instance==PKA)
  {
  /* USER CODE BEGIN PKA_MspInit 0 */

  /* USER CODE END PKA_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_PKA_CLK_ENABLE();
    /* PKA interrupt Init */
    HAL_NVIC_SetPriority(PKA_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(PKA_IRQn);
  /* USER CODE BEGIN PKA_MspInit 1 */

  /* USER CODE END PKA_MspInit 1 */

  }

}

/**
* @brief PKA MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hpka: PKA handle pointer
* @retval None
*/
void HAL_PKA_MspDeInit(PKA_HandleTypeDef* hpka)
{
  if(hpka->Instance==PKA)
  {
  /* USER CODE BEGIN PKA_MspDeInit 0 */

  /* USER CODE END PKA_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_PKA_CLK_DISABLE();

    /* PKA interrupt DeInit */
    HAL_NVIC_DisableIRQ(PKA_IRQn);
  /* USER CODE BEGIN PKA_MspDeInit 1 */

  /* USER CODE END PKA_MspDeInit 1 */
  }

}

/**
* @brief RADIO MSP Initialization
* This function configures the hardware resources used in this example
* @param hradio: RADIO handle pointer
* @retval None
*/
void HAL_RADIO_MspInit(RADIO_HandleTypeDef* hradio)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(hradio->Instance==RADIO)
  {
  /* USER CODE BEGIN RADIO_MspInit 0 */

  /* USER CODE END RADIO_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RF;
    PeriphClkInitStruct.RFClockSelection = RCC_RF_CLK_16M;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    if (__HAL_RCC_RADIO_IS_CLK_DISABLED())
    {
      /* Radio reset */
      __HAL_RCC_RADIO_FORCE_RESET();
      __HAL_RCC_RADIO_RELEASE_RESET();

      /* Enable Radio peripheral clock */
      __HAL_RCC_RADIO_CLK_ENABLE();
    }

    /**RADIO GPIO Configuration
    RF1     ------> RADIO_RF1
    */
    RT_DEBUG_GPIO_Init();

    /* RADIO interrupt Init */
    HAL_NVIC_SetPriority(RADIO_TXRX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RADIO_TXRX_IRQn);
    HAL_NVIC_SetPriority(RADIO_TXRX_SEQ_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RADIO_TXRX_SEQ_IRQn);
  /* USER CODE BEGIN RADIO_MspInit 1 */

  /* USER CODE END RADIO_MspInit 1 */

  }

}

/**
* @brief RADIO MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hradio: RADIO handle pointer
* @retval None
*/
void HAL_RADIO_MspDeInit(RADIO_HandleTypeDef* hradio)
{
  if(hradio->Instance==RADIO)
  {
  /* USER CODE BEGIN RADIO_MspDeInit 0 */

  /* USER CODE END RADIO_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RADIO_CLK_DISABLE();
    __HAL_RCC_RADIO_FORCE_RESET();
    __HAL_RCC_RADIO_RELEASE_RESET();

    /* RADIO interrupt DeInit */
    HAL_NVIC_DisableIRQ(RADIO_TXRX_IRQn);
    HAL_NVIC_DisableIRQ(RADIO_TXRX_SEQ_IRQn);
  /* USER CODE BEGIN RADIO_MspDeInit 1 */

  /* USER CODE END RADIO_MspDeInit 1 */
  }

}

/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspInit 0 */

  /* USER CODE END SPI3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI3 GPIO Configuration
    PB3     ------> SPI3_SCK
    PA8     ------> SPI3_MISO
    PA9     ------> SPI3_NSS
    PA11     ------> SPI3_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_SPI3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_SPI3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_PWR_SetNoPullB(LL_PWR_GPIO_BIT_3);

    LL_PWR_SetNoPullA(LL_PWR_GPIO_BIT_8|LL_PWR_GPIO_BIT_9|LL_PWR_GPIO_BIT_11);

  /* USER CODE BEGIN SPI3_MspInit 1 */

  /* USER CODE END SPI3_MspInit 1 */

  }

}

/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI3)
  {
  /* USER CODE BEGIN SPI3_MspDeInit 0 */

  /* USER CODE END SPI3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI3_CLK_DISABLE();

    /**SPI3 GPIO Configuration
    PB3     ------> SPI3_SCK
    PA8     ------> SPI3_MISO
    PA9     ------> SPI3_NSS
    PA11     ------> SPI3_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11);

  /* USER CODE BEGIN SPI3_MspDeInit 1 */

  /* USER CODE END SPI3_MspDeInit 1 */
  }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB0     ------> USART1_RX
    PA1     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = BATT_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(BATT_EN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    LL_PWR_SetNoPullB(LL_PWR_GPIO_BIT_0);

    LL_PWR_SetNoPullA(LL_PWR_GPIO_BIT_1);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */

  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB0     ------> USART1_RX
    PA1     ------> USART1_TX
    */
    HAL_GPIO_DeInit(BATT_EN_GPIO_Port, BATT_EN_Pin);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    /* USART1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/**
  * @brief  Configures the IOs pull resistors to have the optimized power consumption.
  * @param  None
  * @retval None
  */
static void lowPowerIOSetup(void)
{
  
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, 
                       PWR_GPIO_BIT_0|
                       PWR_GPIO_BIT_1|
                       PWR_GPIO_BIT_2|                         
                       PWR_GPIO_BIT_3);
  
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, 
                             PWR_GPIO_BIT_8|
                             PWR_GPIO_BIT_9|
                             PWR_GPIO_BIT_10|
                             PWR_GPIO_BIT_11);
  
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, 
                             PWR_GPIO_BIT_0|
                             PWR_GPIO_BIT_3|
                             PWR_GPIO_BIT_6|
                             PWR_GPIO_BIT_7|
                             PWR_GPIO_BIT_12|
                             PWR_GPIO_BIT_13);
  
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, 
                       PWR_GPIO_BIT_1|
                       PWR_GPIO_BIT_2|
                       PWR_GPIO_BIT_4|
                       PWR_GPIO_BIT_5|  
                       PWR_GPIO_BIT_14|
                       PWR_GPIO_BIT_15);
}

/* USER CODE END 1 */
