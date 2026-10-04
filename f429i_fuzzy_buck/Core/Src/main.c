/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Buck Converter Fuzzy PID Control
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 64
#define UART_TX_BUFFER_SIZE 64
#define ADC_FILTER_SIZE 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// PWM Control
uint8_t duty_percentage = 0;  // Fixed typo: percentage not percantage
uint32_t pwm_period = 100;    // PWM period (from TIM1 configuration)

// ADC Variables
uint32_t adc_value = 0;
uint32_t adc_values[ADC_FILTER_SIZE] = {0};  // Moving average filter
uint8_t adc_index = 0;
uint32_t adc_filtered = 0;

// UART Communication
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
volatile uint8_t uart_rx_complete = 0;
volatile uint8_t uart_command_ready = 0;
uint8_t rx_char;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Process_UART_Command(void);
void Set_PWM_Duty(uint8_t duty);
uint32_t Read_ADC_Filtered(void);
void Send_ADC_Response(void);
void Initialize_System(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Set PWM duty cycle for buck converter
 * @param duty: Duty cycle percentage (0-100)
 */
void Set_PWM_Duty(uint8_t duty)
{
    if (duty > 90) duty = 90;  // Limit to maximum 90%
    
    duty_percentage = duty;
    uint32_t pulse_value = (duty * pwm_period) / 100;
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pulse_value);
}

/**
 * @brief Read ADC value with moving average filter
 * @retval Filtered ADC value (0-4095)
 */
uint32_t Read_ADC_Filtered(void)
{
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);
    
    // Wait for conversion to complete
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        // Get ADC value
        adc_value = HAL_ADC_GetValue(&hadc1);
        
        // Add to moving average filter
        adc_values[adc_index] = adc_value;
        adc_index = (adc_index + 1) % ADC_FILTER_SIZE;
        
        // Calculate moving average
        uint32_t sum = 0;
        for (int i = 0; i < ADC_FILTER_SIZE; i++)
        {
            sum += adc_values[i];
        }
        adc_filtered = sum / ADC_FILTER_SIZE;
    }
    
    HAL_ADC_Stop(&hadc1);
    return adc_filtered;
}

/**
 * @brief Send ADC response to MATLAB
 */
void Send_ADC_Response(void)
{
    uint32_t adc_reading = Read_ADC_Filtered();
    
    // Format response: "ADC:XXXX\r\n"
    int len = snprintf((char*)uart_tx_buffer, UART_TX_BUFFER_SIZE, 
                      "ADC:%lu\r\n", adc_reading);
    
    HAL_UART_Transmit(&huart1, uart_tx_buffer, len, 100);
}

int duty;

/**
 * @brief Process received UART command from MATLAB
 */
void Process_UART_Command(void)
{
    if (!uart_command_ready) return;
    
    uart_command_ready = 0;
    
    // Null-terminate the received string
    uart_rx_buffer[UART_RX_BUFFER_SIZE-1] = '\0';
    
    // Check for DUTY command: "DUTY:XX"
    if (strncmp((char*)uart_rx_buffer, "DUTY:", 5) == 0)
    {
        // Extract duty cycle value
        duty = atoi((char*)&uart_rx_buffer[5]);
        
        // Validate and set duty cycle
        if (duty >= 0 && duty <= 100)
        {
            Set_PWM_Duty((uint8_t)duty);
            
            // Send acknowledgment (optional)
            // snprintf((char*)uart_tx_buffer, UART_TX_BUFFER_SIZE, "OK:%d\r\n", duty);
            // HAL_UART_Transmit(&huart1, uart_tx_buffer, strlen((char*)uart_tx_buffer), 100);
        }
    }
    // Check for ADC request: "ADC?"
    else if (strncmp((char*)uart_rx_buffer, "ADC?", 4) == 0)
    {
        Send_ADC_Response();
    }
    
    // Clear the buffer
    memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
    
    // Restart UART receive
    HAL_UART_Receive_IT(&huart1, &rx_char, 1);
}

/**
 * @brief Initialize system for buck converter control
 */
void Initialize_System(void)
{
    // Initialize ADC filter array
    for (int i = 0; i < ADC_FILTER_SIZE; i++)
    {
        adc_values[i] = 0;
    }
    
    // Set initial duty cycle to 0% for safety
    Set_PWM_Duty(0);
    
    // Start UART receive interrupt
    HAL_UART_Receive_IT(&huart1, &rx_char, 1);
    
    // Send startup message
    // const char* startup_msg = "STM32 Buck Converter Ready\r\n";
    // HAL_UART_Transmit(&huart1, (uint8_t*)startup_msg, strlen(startup_msg), 100);
}

/**
 * @brief UART Receive Complete Callback
 * @param huart: UART handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        static uint8_t rx_index = 0;
        
        // Store received character
        if (rx_char == '\n' || rx_char == '\r')
        {
            // End of command
            uart_rx_buffer[rx_index] = '\0';
            uart_command_ready = 1;
            rx_index = 0;
        }
        else if (rx_index < UART_RX_BUFFER_SIZE - 1)
        {
            // Store character
            uart_rx_buffer[rx_index++] = rx_char;
        }
        else
        {
            // Buffer overflow, reset
            rx_index = 0;
            memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
        }
        
        // Continue receiving
        HAL_UART_Receive_IT(&huart1, &rx_char, 1);
    }
}

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Start PWM output
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  
  // Initialize buck converter control system
  Initialize_System();
  
  // Small delay for system stabilization
  HAL_Delay(100);
  uint32_t last_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    // Process incoming UART commands from MATLAB
    Process_UART_Command();
    
    // Optional: Add periodic ADC reading for diagnostics
    if (HAL_GetTick() - last_tick >= 50)
    {
        // Read ADC for internal monitoring (not sent unless requested)
        Read_ADC_Filtered();
        last_tick = HAL_GetTick();
    }
    
    // Small delay to prevent excessive CPU usage
    HAL_Delay(1);
    
  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 18 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100 -1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int fd, unsigned char *buf, int len) {
  if (fd == 1 || fd == 2) {                     // stdout or stderr ?
    HAL_UART_Transmit(&huart1, buf, len, 999);  // Print to the UART
  }
  return len;
}

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
