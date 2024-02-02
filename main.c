/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "logger.h"
#include "stdlib.h"
#include "iks01a3_env_sensors_ex.h"
#include "iks01a3_env_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
#include "iks01a3_motion_sensors.h"

#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct DateEnv {
uint32_t tick_value;
float temp;
float press;
float hum;
}date_env;

typedef struct DateAxes {
uint32_t tick_value;
IKS01A3_MOTION_SENSOR_Axes_t axes;
}date_axes;

typedef struct DateTempAnormale {
uint32_t tick_value;
float temp;
}date_temp_a;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* Definitions for StartAcqEnvTask */
osThreadId_t StartAcqEnvTaskHandle;
const osThreadAttr_t StartAcqEnvTask_attributes = {
  .name = "StartAcqEnvTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 192 * 4
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* USER CODE BEGIN PV */

float temp_value = 0;  // Measured temperature value
//char str_tmp[100] = ""; // Formatted message to display the temperature value
uint8_t msg1[] = "****** Temperature values measurement ******\n\n\r";
uint8_t msg2[] = "=====> Initialize Temperature sensor HTS221 \r\n";
uint8_t msg3[] = "=====> Temperature sensor HTS221 initialized \r\n";
//uint8_t msg4[] = "user code beguin 3\n";


//float accel_value = 0;  // Measured accel value

char str_tmp[100] = ""; // Formatted message to display the accel value
uint8_t msg5[] = "****** ACCELERO values measurement ******\n\n\r";
uint8_t msg6[] = "=====> Initialize ACCELERO sensor  \r\n";
uint8_t msg7[] = "=====> ACCELERO sensor IKS0A3 initialized \r\n";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
void StartDefaultTask(void *argument);

//test

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
	//__io_putchar(*ptr++);
	ITM_SendChar(*ptr++);
	}
	return len;
}

void IRQ_TIM16()
{
	//osThreadFlagsSet(acqAccTaskHandle, 0x04);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//test2
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
//  IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_PRESSURE);
//  IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE);
//  IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_HUMIDITY);
//  //IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0,MOTION_ACCELERO);
  log_init(&huart1);

  log_printf("START APP\n");


  //stm32.com
//  HAL_UART_Transmit(&huart1,msg1,sizeof(msg1),1000);
//  HAL_UART_Transmit(&huart1,msg2,sizeof(msg2),1000);
//
//  IKS01A3_ENV_SENSOR_Init(0, ENV_TEMPERATURE);
//  //BSP_ENV_SENSOR_Init(0, ENV_TEMPERATURE);
//  IKS01A3_ENV_SENSOR_Enable(0, ENV_TEMPERATURE);
//  //BSP_ENV_SENSOR_Enable(0, ENV_TEMPERATURE);
//  HAL_UART_Transmit(&huart1,msg3,sizeof(msg3),1000);


  HAL_UART_Transmit(&huart1,msg5,sizeof(msg5),1000);
  HAL_UART_Transmit(&huart1,msg6,sizeof(msg6),1000);
  IKS01A3_MOTION_SENSOR_Init(0, MOTION_ACCELERO);
  IKS01A3_MOTION_SENSOR_Enable(0, MOTION_ACCELERO);
  //BSP_ENV_SENSOR_Enable(0, ENV_TEMPERATURE);
  HAL_UART_Transmit(&huart1,msg7,sizeof(msg7),1000);






  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (10, sizeof(date_env), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of StartAcqEnvTask */
  StartAcqEnvTaskHandle = osThreadNew(StartDefaultTask, NULL, &StartAcqEnvTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the StartAcqEnvTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for(;;)

{
    	date_axes accel_data;

//      	IKS01A3_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &temp_value);
      	int32_t bsp = IKS01A3_MOTION_SENSOR_GetAxes(0, MOTION_ACCELERO, &accel_data.axes);
    	//IKS01A3_MOTION_SENSOR_GetAxesRaw(accel_value, MOTION_ACCELERO, &accel_data);



    	  	 // BSP_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &temp_value);

//    	  	  int tmpInt1 = accel_value;
//    	  	  float tmpFrac = accel_value - tmpInt1;
//    	  	  int tmpInt2 = trunc(tmpFrac * 100);

    	  	  //printf("user code beguin 3\n");
    	  	//  HAL_UART_Transmit(&huart1,msg4,sizeof(msg4),1000);


    	  	  snprintf(str_tmp,100," ACCELERATION = %ld\n\r",accel_data.axes );
    	  	  HAL_UART_Transmit(&huart1,( uint8_t *)str_tmp,sizeof(str_tmp),1000);

    	  	  HAL_Delay(1000);





//
//    	IKS01A3_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &temp_value);
//    	  	 // BSP_ENV_SENSOR_GetValue(0, ENV_TEMPERATURE, &temp_value);
//
//    	  	  int tmpInt1 = temp_value;
//    	  	  float tmpFrac = temp_value - tmpInt1;
//    	  	  int tmpInt2 = trunc(tmpFrac * 100);
//
//    	  	  //printf("user code beguin 3\n");
//    	  	  HAL_UART_Transmit(&huart1,msg4,sizeof(msg4),1000);
//
//
//    	  	  snprintf(str_tmp,100," TEMPERATURE = %d.%02d\n\r", tmpInt1, tmpInt2);
//    	  	  HAL_UART_Transmit(&huart1,( uint8_t *)str_tmp,sizeof(str_tmp),1000);
//    	  	  printf("APRES\n");
//    	  	  HAL_Delay(1000);
//

//        float* acc_data = (float*) malloc(3 * sizeof(float)); // Allocate memory for 3-axis accelerometer data
//
//        int32_t IKS01A3_MOTION_SENSOR_Init();
//        //uint32_t err = IKS01A3_ACC_SENSOR_GetValue(IKS01A3_LSM6DSO_0, ACC_AXIS_ALL, acc_data);
//        int32_t IKS01A3_MOTION_SENSOR_GetAxes(uint32_t Instance, uint32_t Function, IKS01A3_MOTION_SENSOR_Axes_t *Axes)
//        {
//        if (err == osOK) {
//            // Send accelerometer data to the queue
//            osStatus_t status = osMessageQueuePut(accQueueHandle, acc_data, 0, osWaitForever);
//
//            // Display accelerometer data
//            printf("Accelerometer Data: x=%.2f / y=%.2f / z=%.2f\n", acc_data[0], acc_data[1], acc_data[2]);
//        } else {
//            // Handle error, if any
//            printf("Error reading accelerometer data\n");
//            free(acc_data); // Free memory if data could not be read
//        }
//
//        acc_flag = osThreadFlagsWait(0x08, osFlagsWaitAll, osWaitForever);
//
//        // Call the function to send and display accelerometer data
//        sendAndDisplayAccData(accQueueHandle, acc_flag);
//
//        osDelay(1);
//
//}
   }

  /* USER CODE END 5 */
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
