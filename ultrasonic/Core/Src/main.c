/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId UltrasonicBackHandle;
osMutexId LcdHandle;
osStaticMutexDef_t LcdControlBlock;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void UltrasonicBack_Task(void const * argument);

/* USER CODE BEGIN PFP */
uint32_t IC_Front_Val1 = 0;
uint32_t IC_Front_Val2 = 0;
uint32_t Front_Difference = 0;
uint8_t Is_First_Captured_Front = 0;  // is the first value captured ?
uint8_t Front_Distance  = 0;

uint32_t IC_Back_Val1 = 0;
uint32_t IC_Back_Val2 = 0;
uint32_t Back_Difference = 0;
uint8_t Is_First_Captured_Back = 0;  // is the first value captured ?
uint8_t Back_Distance  = 0;

uint8_t Front_Flag=0;
uint8_t Back_Flag=0;
uint8_t Car_direction=0;

///TIM1_ch1
#define TRIG_Front_PIN GPIO_PIN_12
#define TRIG_Front_PORT GPIOB

///TIM1_ch2
#define TRIG_Back_PIN GPIO_PIN_15
#define TRIG_Back_PORT GPIOB

#define FRONT_CHANNEL TIM_CHANNEL_1
#define BACK_CHANNEL TIM_CHANNEL_2

#define IN1_PIN GPIO_PIN_0
#define IN1_PORT GPIOA
#define IN2_PIN GPIO_PIN_1
#define IN2_PORT GPIOA
#define IN3_PIN GPIO_PIN_5
#define IN3_PORT GPIOA
#define IN4_PIN GPIO_PIN_6
#define IN4_PORT GPIOA

#define MOTOR_FORWARD  'F'
#define MOTOR_BACKWARD 'B'
#define MOTOR_STOP     'S'
#define MOTOR_LEFT     'L'
#define MOTOR_RIGHT    'R'

uint8_t received_command[1];
uint8_t Forward_stop=0;
uint8_t Backward_stop=0;

void Stop(void)
{
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
/*	if(htim->Instance== TIM2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
			{
				if (Is_First_Captured_Back==0) // if the first value is not captured
				{
					IC_Back_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
					Is_First_Captured_Back = 1;  // set the first captured as true
					// Now change the polarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
				}

				else if (Is_First_Captured_Back==1)   // if the first is already captured
				{
					IC_Back_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
					__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

					if (IC_Back_Val2 > IC_Back_Val1)
					{
						Back_Difference = IC_Back_Val2-IC_Back_Val1;
					}

					else if (IC_Back_Val1 > IC_Back_Val2)
					{
						Back_Difference = (0xffff - IC_Back_Val1) + IC_Back_Val2;
					}

					Back_Distance = Back_Difference * .034/2;
					Is_First_Captured_Back = 0; // set it back to false

					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
				}
			}
	}*/

	//if(htim->Instance== TIM2)
	//{

	/*	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
			{
				if (Is_First_Captured_Back==0) // if the first value is not captured
				{
					IC_Back_Val1 = HAL_TIM_ReadCapturedValue(htim, BACK_CHANNEL); // read the first value
					Is_First_Captured_Back = 1;  // set the first captured as true
					// Now change the polarity to falling edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, BACK_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
				}

				else if (Is_First_Captured_Back==1)   // if the first is already captured
				{
					IC_Back_Val2 = HAL_TIM_ReadCapturedValue(htim, BACK_CHANNEL);  // read second value
					__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

					if (IC_Back_Val2 > IC_Back_Val1)
					{
						Back_Difference = IC_Back_Val2-IC_Back_Val1;
					}

					else if (IC_Back_Val1 > IC_Back_Val2)
					{
						Back_Difference = (0xffffffff - IC_Back_Val1) + IC_Back_Val2;
					}

					Back_Distance = Back_Difference * .034/2;
					Is_First_Captured_Back = 0; // set it back to false

					// set polarity to rising edge
					__HAL_TIM_SET_CAPTUREPOLARITY(htim, BACK_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
					__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
				}
			}*/
	//}

	    if (htim->Instance == TIM1)
	    {

	        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel 1
	        {
	            if (Is_First_Captured_Front == 0) // if the first value is not captured
	            {
	                IC_Front_Val1 = HAL_TIM_ReadCapturedValue(htim, FRONT_CHANNEL); // read the first value
	                Is_First_Captured_Front = 1;  // set the first captured as true
	                // Now change the polarity to falling edge
	                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FRONT_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
	            }
	            else if (Is_First_Captured_Front == 1)   // if the first is already captured
	            {
	                IC_Front_Val2 = HAL_TIM_ReadCapturedValue(htim, FRONT_CHANNEL);  // read second value
	                __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

	                if (IC_Front_Val2 > IC_Front_Val1)
	                {
	                    Front_Difference = IC_Front_Val2 - IC_Front_Val1;
	                }
	                else if (IC_Front_Val1 > IC_Front_Val2)
	                {
	                    Front_Difference = (0xffff - IC_Front_Val1) + IC_Front_Val2;
	                }

	                Front_Distance = Front_Difference * .034 / 2;
	                Is_First_Captured_Front = 0; // set it back to false

	                // set polarity to rising edge
	                __HAL_TIM_SET_CAPTUREPOLARITY(htim, FRONT_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
	                __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
	            }
	        }

	    }

	    if (htim->Instance == TIM2)
	    {
	        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel 3
	        {
	            if (Is_First_Captured_Back == 0) // if the first value is not captured
	            {
	                IC_Back_Val1 = HAL_TIM_ReadCapturedValue(htim, BACK_CHANNEL); // read the first value
	                Is_First_Captured_Back = 1;  // set the first captured as true
	                // Now change the polarity to falling edge
	                __HAL_TIM_SET_CAPTUREPOLARITY(htim, BACK_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
	            }
	            else if (Is_First_Captured_Back == 1)   // if the first is already captured
	            {
	                IC_Back_Val2 = HAL_TIM_ReadCapturedValue(htim, BACK_CHANNEL);  // read second value
	                __HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

	                if (IC_Back_Val2 > IC_Back_Val1)
	                {
	                    Back_Difference = IC_Back_Val2 - IC_Back_Val1;
	                }
	                else if (IC_Back_Val1 > IC_Back_Val2)
	                {
	                    Back_Difference = (0xffff - IC_Back_Val1) + IC_Back_Val2;
	                }

	                Back_Distance = Back_Difference * .034 / 2;
	                Is_First_Captured_Back = 0; // set it back to false

	                // set polarity to rising edge
	                __HAL_TIM_SET_CAPTUREPOLARITY(htim, BACK_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
	                __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
	            }
	        }
	    }


}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		if (huart->Instance == USART1)
		{
		        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
		        Car_direction=received_command[0];
		        Motor_Control(received_command[0]);
		        HAL_UART_Receive_IT(&huart1, received_command, 1);
		}
}


// Motor control function
void Motor_Control(uint8_t command)
{
    switch (command)
    {
    case MOTOR_FORWARD:
            // Forward
    	if(Forward_stop==0)
    	{
            HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
    	}
            break;
          case MOTOR_BACKWARD:
            // Backward
        	  if(Backward_stop==0)
        	  {
                  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
        	  }

            break;
          case MOTOR_LEFT:
            // Left
            HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
            break;
          case MOTOR_RIGHT:
            // Right
            HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
            break;
          case MOTOR_STOP:
          default:
            // Stop

        	  Stop();
            break;
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);  // Start input capture in interrupt mode for TIM1
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart1, received_command, 1);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of Lcd */
  osMutexStaticDef(Lcd, &LcdControlBlock);
  LcdHandle = osMutexCreate(osMutex(Lcd));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UltrasonicBack */
  osThreadDef(UltrasonicBack, UltrasonicBack_Task, osPriorityAboveNormal, 0, 128);
  UltrasonicBackHandle = osThreadCreate(osThread(UltrasonicBack), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA5 PA6
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB15 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
  //  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UltrasonicBack_Task */
/**
* @brief Function implementing the Ultrasonic_Back thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UltrasonicBack_Task */
void UltrasonicBack_Task(void const * argument)
{
  /* USER CODE BEGIN UltrasonicBack_Task */
	LCD_Init();
	//LCD_Write_String("hello");
  /* Infinite loop */
  for(;;)
  {

	HCSR04_Read(TRIG_Front_PORT, TRIG_Front_PIN, &htim1,TIM_IT_CC1);
//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	HCSR04_Read(TRIG_Back_PORT, TRIG_Back_PIN, &htim2,TIM_IT_CC2);
//	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);

	if(Back_Distance < 15 && Car_direction== MOTOR_BACKWARD )
	{
		Backward_stop=1;
		Stop();
		LCD_Clear();
		LCD_Set_Cursor(0, 0);
		LCD_Write_String("7aseeeeb");
	}

	else if( Front_Distance < 15 && Car_direction==MOTOR_FORWARD)
	{
		Forward_stop=1;
		Stop();
		LCD_Clear();
		LCD_Set_Cursor(0, 0);
		LCD_Write_String("7aseeeeb");
	}
	else if(Back_Distance < 30 && Car_direction== MOTOR_BACKWARD)
			 {


		Backward_stop=0;
				Back_Flag=1;
			   	if(Front_Flag == 1)
			    {
			    	LCD_Clear();
			    }
			    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
				//LCD_Clear();
			   	LCD_Set_Cursor(0, 0);
			    LCD_Write_String("WARNING: BACK");
			 //  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			  }


	 else if(Front_Distance < 30 && Car_direction==MOTOR_FORWARD)
	    {
	    	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

		 Forward_stop=0;
	    	Front_Flag=1;
	    	if(Back_Flag == 1)
	    	{
	    		LCD_Clear();
	    	}

	    	LCD_Set_Cursor(0, 0);
	    	LCD_Write_String("WARNING: FRONT");
	    //	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	    }
	    else
	    {

	    	Forward_stop=0;
	    	Backward_stop=0;
	    	Back_Flag=0;
	    	Front_Flag=0;
	    	///or lidar flag
	    	if(Back_Flag == 1 || Front_Flag==1)
	    	{

	    	}
	    	else
	    	{
	    		LCD_Clear();
	    	}
	    }
//	 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    osDelay(300);
  }
  /* USER CODE END UltrasonicBack_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
