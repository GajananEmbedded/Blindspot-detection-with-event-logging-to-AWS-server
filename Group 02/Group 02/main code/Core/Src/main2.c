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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TRIG_PIN GPIO_PIN_8
#define TRIG_PORT_GPIO GPIOA
#define ECHO_PIN GPIO_PIN_9
#define ECHO_PORT_GPIO GPIOA
/**/
#define TRIG_PIN2 GPIO_PIN_8
#define TRIG_PORT_GPIO2 GPIOC
#define ECHO_PIN2 GPIO_PIN_9
#define ECHO_PORT_GPIO2 GPIOC
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
SPI_HandleTypeDef hspi1;
osThreadId defaultTaskHandle;

/*TIMERS*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* SENSOR 1 */
uint32_t pMillis;
volatile uint32_t sensor1_Triger_Time = 0;
volatile uint32_t sensor1_Echo_Time = 0;
volatile uint32_t sensor2_Triger_Time = 0;
volatile uint32_t sensor2_Echo_Time = 0;
uint32_t distance_by_sensor1 = 0;
uint32_t distance_by_sensor2 = 0;
// uint32_t distance_by_sensor1 = 0;

/*SENSOR 2*/
unsigned int xLastWakeupTime1 = 0;
unsigned int initial_setting = 0;
unsigned int pending_interrupts=0;

SemaphoreHandle_t Mutex_Handle_t = NULL;


/* CAN VARIABLE */
uint8_t ubKeyNumber = 0x0;
CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;

/*VARIABLES ACCELAROMETER*/
uint8_t ReadFlag=0x80;
uint8_t Return[2];
uint16_t Rx_x,Rx_y,Rx_z;
uint8_t displacement=0;
uint8_t  distance_travel=0;

uint8_t  TxBuf[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void vTask1_sensor1( void *pvParameters );
void vTask2_sensor2( void *pvParameters );
void vTask3_CAN_send( void *pvParameters );
void vTask4_displacement( void *pvParameters );

void CAN_filterConfig(void);
void LED_Display(uint8_t LedStatus);
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
	BaseType_t ret = 0;
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
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */


 //  HAL_TIM_Base_Start(&htim2);
 //  HAL_GPIO_WritePin(TRIG_PORT_GPIO2, TRIG_PIN2, GPIO_PIN_RESET);
   HAL_TIM_Base_Start(&htim4);
   HAL_GPIO_WritePin(TRIG_PORT_GPIO2, TRIG_PIN2, GPIO_PIN_RESET);
   HAL_TIM_Base_Start(&htim6);
  HAL_GPIO_WritePin(TRIG_PORT_GPIO, TRIG_PIN, GPIO_PIN_RESET);
 //  HAL_TIM_Base_Start(&htim7);
 //  HAL_GPIO_WritePin(TRIG_PORT_GPIO2, TRIG_PIN7, GPIO_PIN_RESET);

   /*##-Step1:Filter Configuration ###########################################*/
       CAN_filterConfig();

   /*##-Step2:Start the CAN peripheral ###########################################*/

 	/*##-Step4:Configure Transmission process #####################################*/

  /* USER CODE END 2 */

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  	ret = xTaskCreate(vTask1_sensor1, "Ultrasonic_sensor 1 data", 240, NULL,1, NULL);
    configASSERT(ret);
    ret = xTaskCreate(vTask2_sensor2, "Ultrasonic_sensor 1 data", 240, NULL, 1, NULL);
    configASSERT(ret);
    ret = xTaskCreate(vTask3_CAN_send, "CAN_data_send", 240, NULL,1, NULL);
    configASSERT(ret);
    ret = xTaskCreate(vTask4_displacement, "Accellerometer data ", 240, NULL,1, NULL);
        	configASSERT(ret);
  /* Start scheduler */
    vTaskStartScheduler();

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
void vTask1_sensor1( void *pvParameters ){
		while(1){


		HAL_GPIO_WritePin(TRIG_PORT_GPIO2, TRIG_PIN2, GPIO_PIN_SET);
		    __HAL_TIM_SET_COUNTER(&htim4, 0);

		    while (__HAL_TIM_GET_COUNTER (&htim4) < 10);
		    HAL_GPIO_WritePin(TRIG_PORT_GPIO2, TRIG_PIN2, GPIO_PIN_RESET);

		    pMillis = HAL_GetTick();

		    while (!(HAL_GPIO_ReadPin (ECHO_PORT_GPIO2, ECHO_PIN2)) && pMillis + 5 >  HAL_GetTick());
		    sensor2_Triger_Time = __HAL_TIM_GET_COUNTER (&htim4);
if(sensor2_Triger_Time<100){sensor2_Triger_Time=0;sensor2_Echo_Time=0;}
		    pMillis = HAL_GetTick();

		    while ((HAL_GPIO_ReadPin (ECHO_PORT_GPIO2, ECHO_PIN2)) && pMillis + 10 > HAL_GetTick());
		    sensor2_Echo_Time = __HAL_TIM_GET_COUNTER (&htim4);

		    distance_by_sensor2 =((((((sensor2_Echo_Time-sensor2_Triger_Time))/10)*(343/2))*0.0245)/2)/100;

		    if(distance_by_sensor2 >=70 )
		   		{
		   			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, RESET);
		   			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, RESET);
		   			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, RESET);
		   			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, RESET);
		   		}

		   		else if ((distance_by_sensor2 >= 50)&&(distance_by_sensor2 < 70))
		   		{
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
		   			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		   		}

		   		else if ((distance_by_sensor2 >= 25)&&(distance_by_sensor2 < 50))
		   		{
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);

		   		}
		   		else if ((distance_by_sensor2 >= 10)&&(distance_by_sensor2 < 25))
		   		{
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
		   		}

		   		else if (distance_by_sensor2 < 10)
		   		{
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
		   			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
		   		}


		}
		HAL_Delay(500);
		}
void vTask2_sensor2( void *pvParameters ){
	while(1){


	HAL_GPIO_WritePin(TRIG_PORT_GPIO, TRIG_PIN, GPIO_PIN_SET);
	    __HAL_TIM_SET_COUNTER(&htim6, 0);

	    while (__HAL_TIM_GET_COUNTER (&htim6) < 10);
	    HAL_GPIO_WritePin(TRIG_PORT_GPIO, TRIG_PIN, GPIO_PIN_RESET);

	    pMillis = HAL_GetTick();

	    while (!(HAL_GPIO_ReadPin (ECHO_PORT_GPIO, ECHO_PIN)) && pMillis + 5 >  HAL_GetTick());
	    sensor1_Triger_Time = __HAL_TIM_GET_COUNTER (&htim6);

	    pMillis = HAL_GetTick();

	    while ((HAL_GPIO_ReadPin (ECHO_PORT_GPIO, ECHO_PIN)) && pMillis + 10 > HAL_GetTick());
	    sensor1_Echo_Time = __HAL_TIM_GET_COUNTER (&htim6);

	    distance_by_sensor1 =((((((sensor1_Echo_Time-sensor1_Triger_Time))/10)*(343/2))*0.0245)/2)/100;

	    if(distance_by_sensor1 >=70 )
	   		{
	   			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
	   			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET);
	   			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, RESET);
	   			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, RESET);
	   		}

	   		else if ((distance_by_sensor1 >= 50)&&(distance_by_sensor1 < 70))
	   		{
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
	   			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
	   		}

	   		else if ((distance_by_sensor1 >= 25)&&(distance_by_sensor1 < 50))
	   		{
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);

	   		}
	   		else if ((distance_by_sensor1 >= 10)&&(distance_by_sensor1 < 25))
	   		{
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
	   		}

	   		else if (distance_by_sensor1 < 10)
	   		{
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_13);
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_11);
	   			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
	   		}


	}
	HAL_Delay(500);
	}
void vTask3_CAN_send( void *pvParameters ){

	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	     	  {
	     		/* Start Error */
	     		Error_Handler();
	     	  }
	   /*##-Step3:Activate CAN RX notification #######################################*/
	 	 if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	 	{
	 	  // Notification Error
	 	  Error_Handler();
	 	}
		  TxHeader.StdId = 0x123;
		  TxHeader.RTR = CAN_RTR_DATA;
		  TxHeader.IDE = CAN_ID_STD;
		  TxHeader.DLC = 2;
		  TxHeader.TransmitGlobalTime = DISABLE;
	while(1){
	  {
		    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
			      {
		    	HAL_Delay(50);//Debouncing Delay
		    	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
		    	{
						if (ubKeyNumber == 0x4)
							{
							  ubKeyNumber = 0x00;
							}
						else
							{
							/*time-bound sending/specifica value */
							  LED_Display(++ubKeyNumber);
							  /* Set the data to be transmitted */
							  TxData[0] = ubKeyNumber;
							  TxData[1] = 0xAD;///value to be send

							  /* Start the Transmission process */
							  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
								  {
									/* Transmission request Error */
									Error_Handler();
								  }
					HAL_Delay(100);//Delay just for better Tuning
							}
						}
			      }
	    /* USER CODE END WHILE */

	    /* USER CODE end */
	  }
	}
	HAL_Delay(500);
}
void vTask4_displacement( void *pvParameters ){

	while(1){
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
		  TxBuf[0]=0x20;//Address of Register
		  TxBuf[1]=0x37;//Data for Register
		  HAL_SPI_Transmit(&hspi1, TxBuf, 2, 50);
		  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High
		  /* USER CODE END 2 */

		  /* Infinite loop */
		  /* USER CODE BEGIN WHILE */
		  while (1)
		  {
			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
			TxBuf[0]=0x28|ReadFlag;//Read Multi Byte Address of Register for X
			HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
			HAL_SPI_Receive(&hspi1, Return, 2, 50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High

			Rx_x= ((Return[1]<<8)|Return[0])/100;
			if((Rx_x>=50) && (Rx_x<=5000))
					    {
					    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,SET);
					    }
					else
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,RESET);//LED



			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
			TxBuf[0]=0x2A|ReadFlag;//Read Multi Byte Address of Register for Y
			HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
			HAL_SPI_Receive(&hspi1, Return, 2, 50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High*/

			Rx_y= ((Return[1]<<8)|Return[0])/100;
			if((Rx_y<=700) && (Rx_y<=5000))
				    {
				    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,SET);
				    }
				else
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,RESET);//led2



			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
			TxBuf[0]=0x2C|ReadFlag;//Read Multi Byte Address of Register for Z
			HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
			HAL_SPI_Receive(&hspi1, Return, 2, 50);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High

			Rx_z= ((Return[1]<<8)|Return[0])/100;
			if((Rx_z>=180) && (Rx_z<=5000))
					    {
					    	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,SET);//led5
					    }
					else
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,RESET);//led5*/
		/* only for Rx_y*/


		    /* USER CODE BEGIN 3 */
		  }
		  /* USER CODE END 3 */

		}
	displacement+=Rx_y;
	HAL_Delay(500);
	}
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
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 56;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE11 PE12 PE13 PE14
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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

void CAN_filterConfig(void)
{
	/*##- Setup the Filter to receive ANY MSG ID #####################################*/
	CAN_FilterTypeDef sFilterConfig;

	  sFilterConfig.FilterBank = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

/* Get RX message from Fifo0 as message is Pending in Fifo0 to be Read */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	  /* Get RX message from Fifo0 as message is Pending in Fifo to be Read */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }

  /* Display LEDx */
  if ((RxHeader.StdId == 0x123) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 2))
  {
    LED_Display(RxData[0]);
    ubKeyNumber = RxData[0];
  }
}

/**
  * @brief  Turns ON/OFF the dedicated LED.
  * @param  LedStatus: LED number from 0 to 3
  * @retval None
  */
void LED_Display(uint8_t LedStatus)
{
  /* Turn OFF all LEDs */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  switch(LedStatus)
  {
    case (1):
      /* Turn ON LED1 */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
      break;

    case (2):
      /* Turn ON LED2 */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
      break;

    case (3):
      /* Turn ON LED3 */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      break;

    case (4):
      /* Turn ON LED4 */
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
      break;
    default:
      break;
  }
}
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
    osDelay(1);
  }
  /* USER CODE END 5 */
}
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
