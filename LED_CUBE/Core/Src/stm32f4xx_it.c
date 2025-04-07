/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
// extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
// extern DMA_HandleTypeDef hdma_tim1_up;
// extern DMA_HandleTypeDef hdma_tim1_ch1;
// extern DMA_HandleTypeDef hdma_tim1_ch2;

extern UART_HandleTypeDef huart2;  // Add this line!

extern volatile uint32_t lastButton1Press;
extern volatile uint32_t lastButton2Press;
extern volatile uint8_t currentMode;
extern volatile uint8_t currentAnimation; 
extern const uint8_t animationPatterns[][8];
extern void updateLEDs(uint8_t* pattern);

extern volatile uint8_t button1Pressed;  // Mode toggle flag
extern volatile uint8_t button2Pressed;  // Animation change flag

extern volatile uint8_t button1State;
extern volatile uint8_t lastButton1State;
extern volatile uint32_t button1PressStartTime;
extern volatile uint8_t button1LongPressDetected;
/* USER CODE BEGIN EV */

/* Button handling constants */
#define MODE_ANIMATION 0
#define MODE_STL 1
#define DEBOUNCE_TIME 75
#define BUTTON_LONG_PRESS_TIME 1000  // 1 second

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

// /**
//   * @brief This function handles DMA2 stream1 global interrupt.
//   */
// void DMA2_Stream1_IRQHandler(void)
// {
//   /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

//   /* USER CODE END DMA2_Stream1_IRQn 0 */
//   HAL_DMA_IRQHandler(&hdma_tim1_ch1);
//   /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

//   /* USER CODE END DMA2_Stream1_IRQn 1 */
// }

// /**
//   * @brief This function handles DMA2 stream2 global interrupt.
//   */
// void DMA2_Stream2_IRQHandler(void)
// {
//   /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

//   /* USER CODE END DMA2_Stream2_IRQn 0 */
//   HAL_DMA_IRQHandler(&hdma_tim1_ch2);
//   /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

//   /* USER CODE END DMA2_Stream2_IRQn 1 */
// }

// /**
//   * @brief This function handles USB On The Go FS global interrupt.
//   */
// void OTG_FS_IRQHandler(void)
// {
//   /* USER CODE BEGIN OTG_FS_IRQn 0 */

//   /* USER CODE END OTG_FS_IRQn 0 */
//   HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
//   /* USER CODE BEGIN OTG_FS_IRQn 1 */

//   /* USER CODE END OTG_FS_IRQn 1 */
// }

/**
  * @brief This function handles DMA2 stream5 global interrupt.
  */
// void DMA2_Stream5_IRQHandler(void)
// {
//   /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

//   /* USER CODE END DMA2_Stream5_IRQn 0 */
//   HAL_DMA_IRQHandler(&hdma_tim1_up);
//   /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

//   /* USER CODE END DMA2_Stream5_IRQn 1 */
// }

/* USER CODE BEGIN 1 */
/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts (for buttons).
  */
void EXTI15_10_IRQHandler(void)
{
  uint32_t currentTime = HAL_GetTick();
  
  /* PC13 - Button 1 (with long press detection) */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
    
    // Get current button state
    button1State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    
    // Debounce check
    if (currentTime - lastButton1Press > DEBOUNCE_TIME)
    {
      // On button press (transition from released to pressed)
      if (button1State == GPIO_PIN_RESET && lastButton1State == GPIO_PIN_SET)
      {
        button1PressStartTime = currentTime;
        lastButton1Press = currentTime;
      }
      // On button release (transition from pressed to released)
      else if (button1State == GPIO_PIN_SET && lastButton1State == GPIO_PIN_RESET)
      {
        uint32_t pressDuration = currentTime - button1PressStartTime;
        
        if (pressDuration >= BUTTON_LONG_PRESS_TIME)
        {
          button1LongPressDetected = 1;
        }
        else
        {
          button1Pressed = 1;
        }
        
        lastButton1Press = currentTime;
      }
    }
    
    lastButton1State = button1State;
  }
  
  /* PC10 - Button 2 (Animation button) - No changes to this logic */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
    
    // Debounce check
    if (currentTime - lastButton2Press > DEBOUNCE_TIME*2)
    {
      lastButton2Press = currentTime;
      
      // Set flag to be processed in main loop
      button2Pressed = 1;
      
      // Simple feedback (non-blocking)
      // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    }
  }
}
