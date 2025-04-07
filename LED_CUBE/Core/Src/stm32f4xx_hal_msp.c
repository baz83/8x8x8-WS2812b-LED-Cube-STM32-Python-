/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
/* USER CODE END Includes */
// extern DMA_HandleTypeDef hdma_tim1_up;

// extern DMA_HandleTypeDef hdma_tim1_ch1;

// extern DMA_HandleTypeDef hdma_tim1_ch2;

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

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */
extern void Error_Handler(void);
/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
 void HAL_MspInit(void)
 {
 
   /* USER CODE BEGIN MspInit 0 */
 
   /* USER CODE END MspInit 0 */
   
   HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
 
   /* System interrupt init*/
   /* MemoryManagement_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
   /* BusFault_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
   /* UsageFault_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
   /* SVCall_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
   /* DebugMonitor_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
   /* PendSV_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
   /* SysTick_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
 
   /* System interrupt init*/
 
   /* USER CODE BEGIN MspInit 1 */
 
   /* USER CODE END MspInit 1 */
 }

// /**
// * @brief TIM_PWM MSP Initialization
// * This function configures the hardware resources used in this example
// * @param htim_pwm: TIM_PWM handle pointer
// * @retval None
// */
// void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
// {
//   if(htim_pwm->Instance==TIM1)
//   {
//   /* USER CODE BEGIN TIM1_MspInit 0 */

//   /* USER CODE END TIM1_MspInit 0 */
//     /* Peripheral clock enable */
//     __HAL_RCC_TIM1_CLK_ENABLE();

//     /* TIM1 DMA Init */
//     /* TIM1_UP Init */
//     hdma_tim1_up.Instance = DMA2_Stream5;
//     hdma_tim1_up.Init.Channel = DMA_CHANNEL_6;
//     hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
//     hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_tim1_up.Init.MemInc = DMA_MINC_DISABLE;
//     hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//     hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//     hdma_tim1_up.Init.Mode = DMA_CIRCULAR;
//     hdma_tim1_up.Init.Priority = DMA_PRIORITY_VERY_HIGH;
//     hdma_tim1_up.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//     if (HAL_DMA_Init(&hdma_tim1_up) != HAL_OK)
//     {
//       Error_Handler();
//     }

//     __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_UPDATE],hdma_tim1_up);

//     /* TIM1_CH1 Init */
//     hdma_tim1_ch1.Instance = DMA2_Stream1;
//     hdma_tim1_ch1.Init.Channel = DMA_CHANNEL_6;
//     hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
//     hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
//     hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//     hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//     hdma_tim1_ch1.Init.Mode = DMA_CIRCULAR;
//     hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
//     hdma_tim1_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//     if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
//     {
//       Error_Handler();
//     }

//     __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC1],hdma_tim1_ch1);

//     /* TIM1_CH2 Init */
//     hdma_tim1_ch2.Instance = DMA2_Stream2;
//     hdma_tim1_ch2.Init.Channel = DMA_CHANNEL_6;
//     hdma_tim1_ch2.Init.Direction = DMA_MEMORY_TO_PERIPH;
//     hdma_tim1_ch2.Init.PeriphInc = DMA_PINC_DISABLE;
//     hdma_tim1_ch2.Init.MemInc = DMA_MINC_DISABLE;
//     hdma_tim1_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
//     hdma_tim1_ch2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
//     hdma_tim1_ch2.Init.Mode = DMA_CIRCULAR;
//     hdma_tim1_ch2.Init.Priority = DMA_PRIORITY_VERY_HIGH;
//     hdma_tim1_ch2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//     if (HAL_DMA_Init(&hdma_tim1_ch2) != HAL_OK)
//     {
//       Error_Handler();
//     }

//     __HAL_LINKDMA(htim_pwm,hdma[TIM_DMA_ID_CC2],hdma_tim1_ch2);

//   /* USER CODE BEGIN TIM1_MspInit 1 */

//   /* USER CODE END TIM1_MspInit 1 */

//   }

// }

/**
// * @brief TIM_PWM MSP De-Initialization
// * This function freeze the hardware resources used in this example
// * @param htim_pwm: TIM_PWM handle pointer
// * @retval None
// */
// void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
// {
//   if(htim_pwm->Instance==TIM1)
//   {
//   /* USER CODE BEGIN TIM1_MspDeInit 0 */

//   /* USER CODE END TIM1_MspDeInit 0 */
//     /* Peripheral clock disable */
//     __HAL_RCC_TIM1_CLK_DISABLE();

//     /* TIM1 DMA DeInit */
//     HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_UPDATE]);
//     HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC1]);
//     HAL_DMA_DeInit(htim_pwm->hdma[TIM_DMA_ID_CC2]);
//   /* USER CODE BEGIN TIM1_MspDeInit 1 */

//   /* USER CODE END TIM1_MspDeInit 1 */
//   }

// }

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */

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
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
