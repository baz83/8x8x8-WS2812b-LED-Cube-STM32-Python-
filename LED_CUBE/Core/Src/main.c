/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
// REMOVE: #include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "led_cube.h"
#include "cube_patterns.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define READY_SIGNAL 0xAA      // Signal to indicate ready to receive LED pattern
#define MODE_ANIMATION 0       // Animation mode
#define MODE_STL 1             // STL display mode
#define ANIMATION_COUNT 5      // Number of built-in animations

#define CUBE_START_MARKER 0xA5  // Frame start marker
#define CUBE_END_MARKER   0x5A  // Frame end marker
#define CUBE_BUFFER_SIZE  64    // 64 bytes = 512 bits for 8x8x8 cube

// Button timing constants
#define BUTTON_LONG_PRESS_TIME 1000  // 1000ms = 1 second for long press

/* UART Ring Buffer for improved reception reliability */
#define UART_RING_BUFFER_SIZE 128  // Size of the ring buffer (power of 2 is most efficient)

// Rotation types
#define ROTATION_NONE   0
#define ROTATION_90_X   1
#define ROTATION_90_Y   2
#define ROTATION_90_Z   3
#define ROTATION_180_X  4
#define ROTATION_180_Y  5
#define ROTATION_180_Z  6
#define ROTATION_270_X  7
#define ROTATION_270_Y  8
#define ROTATION_270_Z  9
#define ROTATION_COUNT  10
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t ledArray[8];          // Array for individual LED states
uint8_t testPattern[8] = {1, 1, 1, 1, 1, 1, 1, 1};  // Test pattern

// Button flags
volatile uint8_t button1Pressed = 0;
volatile uint8_t button2Pressed = 0;
volatile uint8_t blinkLedFlag = 0;  // Flag to signal LED blink from interrupt

// Button state tracking
volatile uint8_t button1State = GPIO_PIN_SET;  // Current button state (default: not pressed)
volatile uint8_t lastButton1State = GPIO_PIN_SET;  // Previous button state
volatile uint32_t button1PressStartTime = 0;  // When button was pressed
volatile uint8_t button1LongPressDetected = 0;  // Flag for long press

// Mode and animation control
volatile uint8_t currentMode = MODE_ANIMATION;  // Start in animation mode
volatile uint8_t currentAnimation = 0;          // Current animation index

// Button debouncing
volatile uint32_t lastButton1Press = 0;
volatile uint32_t lastButton2Press = 0;

// UART reception for cube data
uint8_t uartRxBuffer[1];                // Single byte receive buffer
uint8_t cubeDataBuffer[CUBE_BUFFER_SIZE]; // Buffer for received cube data
uint16_t cubeDataIndex = 0;             // Current position in the data buffer
uint8_t rxState = 0;                    // 0=waiting for start, 1=receiving data, 2=waiting for end

volatile uint8_t uartRingBuffer[UART_RING_BUFFER_SIZE];
volatile uint16_t ringBufferHead = 0;  // Position to write new data
volatile uint16_t ringBufferTail = 0;  // Position to read data from

// STL rotation variables
volatile uint8_t currentRotation = ROTATION_NONE;
uint8_t originalSTLData[CUBE_BUFFER_SIZE];  // Store original STL data

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MX_Button_Init(void);
float LED_Cube_GetRefreshRate(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief Apply rotation to STL data and update the LED cube
  * @param rotation Rotation type (0-9)
  * @param data Original STL data
  */
 void rotateAndUpdateSTL(uint8_t rotation) 
 {
   // Create temporary buffer for rotated data
   uint8_t tempCube[PLANE_COUNT][CUBE_SIZE][CUBE_SIZE] = {0};
   
   // First unpack original data into a temp array
   uint16_t bitIndex = 0;
   for (uint16_t byteIndex = 0; byteIndex < CUBE_BUFFER_SIZE; byteIndex++) {
     uint8_t currentByte = originalSTLData[byteIndex];
     
     for (uint8_t bit = 0; bit < 8; bit++) {
       uint8_t state = (currentByte >> (7 - bit)) & 0x01;
       
       uint8_t z = bitIndex / 64;
       uint16_t remainder = bitIndex % 64;
       uint8_t y = remainder / 8;
       uint8_t x = remainder % 8;
       
       if (z < PLANE_COUNT && y < CUBE_SIZE && x < CUBE_SIZE) {
         tempCube[z][y][x] = state;
       }
       
       bitIndex++;
       if (bitIndex >= 512) break;
     }
   }
   
   // Clear cube data
   memset(cube_data, 0, sizeof(cube_data));
   
   // Apply rotation
   for (uint8_t z = 0; z < PLANE_COUNT; z++) {
     for (uint8_t y = 0; y < CUBE_SIZE; y++) {
       for (uint8_t x = 0; x < CUBE_SIZE; x++) {
         uint8_t nx = 0, ny = 0, nz = 0;
         
         switch (rotation) {
           case ROTATION_90_X:  // 90° around X
             nx = x; ny = CUBE_SIZE-1-z; nz = y;
             break;
           case ROTATION_90_Y:  // 90° around Y
             nx = z; ny = y; nz = CUBE_SIZE-1-x;
             break;
           case ROTATION_90_Z:  // 90° around Z
             nx = CUBE_SIZE-1-y; ny = x; nz = z;
             break;
           case ROTATION_180_X: // 180° around X
             nx = x; ny = CUBE_SIZE-1-y; nz = CUBE_SIZE-1-z;
             break;
           case ROTATION_180_Y: // 180° around Y
             nx = CUBE_SIZE-1-x; ny = y; nz = CUBE_SIZE-1-z;
             break;
           case ROTATION_180_Z: // 180° around Z
             nx = CUBE_SIZE-1-x; ny = CUBE_SIZE-1-y; nz = z;
             break;
           case ROTATION_270_X: // 270° around X
             nx = x; ny = z; nz = CUBE_SIZE-1-y;
             break;
           case ROTATION_270_Y: // 270° around Y
             nx = CUBE_SIZE-1-z; ny = y; nz = x;
             break;
           case ROTATION_270_Z: // 270° around Z
             nx = y; ny = CUBE_SIZE-1-x; nz = z;
             break;
           default:  // No rotation (0)
             nx = x; ny = y; nz = z;
         }
         
         // Apply rotated data
         if (nx < CUBE_SIZE && ny < CUBE_SIZE && nz < PLANE_COUNT) {
           cube_data[nz][ny][nx] = tempCube[z][y][x];
         }
       }
     }
   }
 }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    /* Store received byte in ring buffer */
    uint8_t receivedByte = uartRxBuffer[0];
    
    /* Calculate next head position (with wrap-around) */
    uint16_t nextHead = (ringBufferHead + 1) % UART_RING_BUFFER_SIZE;
    
    /* Only store if buffer isn't full */
    if (nextHead != ringBufferTail) {
      uartRingBuffer[ringBufferHead] = receivedByte;
      ringBufferHead = nextHead;
    }
    
    /* Restart UART reception for next byte */
    HAL_UART_Receive_IT(&huart2, uartRxBuffer, 1);
  }
}

/**
  * @brief Process data in the UART ring buffer
  * @retval None
  */
 void processUartRingBuffer(void)
 {
   /* Process all available bytes in ring buffer */
   while (ringBufferHead != ringBufferTail) {
     /* Get the next byte from buffer */
     uint8_t receivedByte = uartRingBuffer[ringBufferTail];
     ringBufferTail = (ringBufferTail + 1) % UART_RING_BUFFER_SIZE;
     
     /* Only process data in STL mode */
     if (currentMode == MODE_STL) {
       /* Process based on current state */
       switch (rxState) {
         case 0: // Waiting for start marker
           if (receivedByte == CUBE_START_MARKER) {
             /* Start marker received, prepare to receive data */
             cubeDataIndex = 0;
             rxState = 1;
             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // LED on during reception
           }
           else {
             HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Toggle LED for feedback
           }
           break;
           
         case 1: // Receiving data
           /* Store the received byte */
           if (cubeDataIndex < CUBE_BUFFER_SIZE) {
             cubeDataBuffer[cubeDataIndex++] = receivedByte;
           }
           
           /* If we received all expected data, wait for end marker */
           if (cubeDataIndex >= CUBE_BUFFER_SIZE) {
             rxState = 2;
           }
           break;
           
          case 2: // Waiting for end marker
          if (receivedByte == CUBE_END_MARKER) {
            /* End marker received, process the complete cube data */
            // Store a copy of the original data for rotations
            memcpy(originalSTLData, cubeDataBuffer, CUBE_BUFFER_SIZE);
            currentRotation = ROTATION_NONE;  // Reset rotation state
            
            LED_Cube_SetFullData(cubeDataBuffer, CUBE_BUFFER_SIZE);
            rxState = 0;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            blinkLedFlag = 1;  // Set flag for LED feedback
          }
          break;
       }
     }
   }
 }

/**
  * @brief Initialize GPIO pins for button interrupts
  */
static void MX_Button_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  // Enable GPIOC clock
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  // Configure Button 1 (PC13 - User Button on Nucleo board)
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;  // Interrupt on both edges
  GPIO_InitStruct.Pull = GPIO_PULLUP;                  // Use internal pull-up
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  // Configure Button 2 (PC10 - External button)
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge
  GPIO_InitStruct.Pull = GPIO_PULLUP;           // Use internal pull-up
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  // Set interrupt priorities
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  
  // Enable the EXTI interrupts
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
  // Initialize button state tracking variables
  button1State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  lastButton1State = button1State;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize peripherals */
  MX_USART2_UART_Init();
  MX_Button_Init();  // Initialize button GPIOs with interrupts
  
  /* Initialize debug LED */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_5;  // On-board LED (PA5)
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  static uint32_t lastFpsDisplay = 0;

  /* Double blink to show we've started */
  for (int i = 0; i < 2; i++) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(200);
  }
  
  /* Initialize WS2812B LEDs */
  LED_Cube_Init();
  CUBE_Patterns_Init();
  
  /* Triple blink to show UART receiving is starting */
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_Delay(100);
  }
  
  /* Start UART reception in interrupt mode */
  HAL_UART_Receive_IT(&huart2, uartRxBuffer, 1);
  
  /* Main communication loop */
  uint32_t lastReadySignalTime = 0;
  uint32_t lastAutoAnimationTime = 0;
  uint8_t readySignal[1] = {READY_SIGNAL};
  
  while (1)
  {
    if (HAL_GetTick() - lastFpsDisplay >= 2000) {
      lastFpsDisplay = HAL_GetTick();
      
      // Get current refresh rate
      float fps = LED_Cube_GetRefreshRate();
      
      // Format using integer math instead of float formatting
      int whole = (int)fps;
      int frac = (int)((fps - whole) * 10); // Get first decimal place
      
      char buffer[50];
      sprintf(buffer, "LED Cube refresh rate: %d.%d Hz\r\n", whole, frac);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
    }
    /* Process button flags */
    if (button1Pressed) {
      button1Pressed = 0;  // Clear flag
      
      if (currentMode == MODE_ANIMATION) {
        // In animation mode, cycle through patterns
        CUBE_Patterns_NextPattern();
        
        // Visual feedback (single blink)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      }
      else if (currentMode == MODE_STL) {
        // In STL mode, rotate the model
        currentRotation = (currentRotation + 1) % ROTATION_COUNT;
        rotateAndUpdateSTL(currentRotation);
        
        // Visual feedback (single blink)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      }
    }

    // Handle long press on Button 1
    if (button1LongPressDetected) {
      button1LongPressDetected = 0;  // Clear flag
      
      if (currentMode == MODE_ANIMATION) {
        // Toggle between static and dynamic modes
        uint8_t newMode = (CUBE_Patterns_GetMode() == PATTERN_STATIC) ? 
                          PATTERN_DYNAMIC : PATTERN_STATIC;
        CUBE_Patterns_SetMode(newMode);
        
        // Visual feedback (triple blink for mode switch)
        for (int i = 0; i < 3; i++) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          HAL_Delay(50);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
          HAL_Delay(50);
        }
      }
    }
    
    // Handle Button 2 press
    if (button2Pressed) {
      button2Pressed = 0;  // Clear flag
      
      if (currentMode == MODE_ANIMATION) {
        // Switch from Animation to STL mode
        currentMode = MODE_STL;
        
        // Clear all LEDs when switching to STL mode
        LED_Cube_Clear();
        
        // RESET BUTTON STATE VARIABLES
        lastButton1State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
        button1State = lastButton1State;
        button1LongPressDetected = 0;
        button1Pressed = 0;
        lastButton1Press = HAL_GetTick();
        
        // Visual feedback for mode change (double blink)
        for (int i = 0; i < 2; i++) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          HAL_Delay(100);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
          HAL_Delay(100);
        }
      } 
      else if (currentMode == MODE_STL) {
        // Switch from STL to Animation mode
        currentMode = MODE_ANIMATION;
        
        // RESET BUTTON STATE VARIABLES
        lastButton1State = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
        button1State = lastButton1State;
        button1LongPressDetected = 0;
        button1Pressed = 0; 
        lastButton1Press = HAL_GetTick();

        // Visual feedback for mode change (triple blink)
        for (int i = 0; i < 3; i++) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
          HAL_Delay(100);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
          HAL_Delay(100);
        }
      }
    }

    /* Process any received UART data */
    processUartRingBuffer();
    
    /* Handle mode-specific operations */
    uint32_t currentTime = HAL_GetTick();

    if (currentMode == MODE_ANIMATION) {
        // Process cube patterns
        CUBE_Patterns_ProcessAnimation();
        LED_Cube_Process();  // Always process the cube display
    } 
    else if (currentMode == MODE_STL) {
        /* Process LED cube when in STL mode */
        LED_Cube_Process();
        
        /* Send ready signal every 500ms */
        if (currentTime - lastReadySignalTime >= 500) {
            lastReadySignalTime = currentTime;
            
            /* Send ready signal via UART */
            HAL_UART_Transmit(&huart2, readySignal, 1, 100);
            
            /* Toggle LED to show we're sending */
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        }
    }

    /* Process blink LED flag */
    if (blinkLedFlag) {
        blinkLedFlag = 0;
        
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
  }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
// static void MX_TIM1_Init(void)
// {

//   /* USER CODE BEGIN TIM1_Init 0 */

//   /* USER CODE END TIM1_Init 0 */

//   TIM_MasterConfigTypeDef sMasterConfig = {0};
//   TIM_OC_InitTypeDef sConfigOC = {0};
//   TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

//   /* USER CODE BEGIN TIM1_Init 1 */

//   /* USER CODE END TIM1_Init 1 */
//   htim1.Instance = TIM1;
//   htim1.Init.Prescaler = 0;
//   htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
//   htim1.Init.Period = 65535;
//   htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   htim1.Init.RepetitionCounter = 0;
//   htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//   if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sConfigOC.OCMode = TIM_OCMODE_PWM1;
//   sConfigOC.Pulse = 0;
//   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//   sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//   sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//   sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//   sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//   sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//   sBreakDeadTimeConfig.DeadTime = 0;
//   sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//   sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//   sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//   if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN TIM1_Init 2 */

//   /* USER CODE END TIM1_Init 2 */

// }

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  // Enable USART2 interrupt in the NVIC
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
// static void MX_DMA_Init(void)
// {

//   /* DMA controller clock enable */
//   __HAL_RCC_DMA2_CLK_ENABLE();

//   /* DMA interrupt init */
//   /* DMA2_Stream1_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
//   HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
//   /* DMA2_Stream2_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
//   HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
//   /* DMA2_Stream5_IRQn interrupt configuration */
//   HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
//   HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

// }

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
// static void MX_GPIO_Init(void)
// {
//   GPIO_InitTypeDef GPIO_InitStruct = {0};
// /* USER CODE BEGIN MX_GPIO_Init_1 */
// /* USER CODE END MX_GPIO_Init_1 */

//   /* GPIO Ports Clock Enable */
//   __HAL_RCC_GPIOC_CLK_ENABLE();
//   __HAL_RCC_GPIOH_CLK_ENABLE();
//   __HAL_RCC_GPIOA_CLK_ENABLE();
//   __HAL_RCC_GPIOB_CLK_ENABLE();

//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

//   /*Configure GPIO pin Output Level */
//   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

//   /*Configure GPIO pin : B1_Pin */
//   GPIO_InitStruct.Pin = B1_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

//   /*Configure GPIO pin : PC0 */
//   GPIO_InitStruct.Pin = GPIO_PIN_0;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

//   /*Configure GPIO pin : LD2_Pin */
//   GPIO_InitStruct.Pin = LD2_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_NOPULL;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

// /* USER CODE BEGIN MX_GPIO_Init_2 */
// /* USER CODE END MX_GPIO_Init_2 */
// }

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
