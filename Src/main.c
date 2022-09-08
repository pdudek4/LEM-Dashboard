/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nextion.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


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
CAN_HandleTypeDef hcan1;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
static CAN_RxHeaderTypeDef CANRxh;
CAN_TxHeaderTypeDef txCAN;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void CAN_filterConfig(void);
void CAN_filterConfig1(void);

//void Heartbeat_err(uint8_t device);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t txmailbox;
	
bool nx_dowysylki_p;
bool nx_dowysylki_r;
bool nx_dowysylki_sd;

uint8_t CAN_ramka[CAN_FRAME_COUNT][8];
uint8_t CAN_data[8] = {0, 1, 2, 3, 4, 5, 6, 7};

char nx_endline[3] = {0xff, 0xff, 0xff};
char buf_nxt_p[90];
char buf_nxt_r[140];

char buf_sd1[2500];
char buf_sd2[2500];

nextion_uart_t nx_val;
sd_card_t sd_card;

dash_state_t dash_state = IDLE;
dash_page_t dash_page = PAGE0;

uint8_t Uart2_buf_rx[10]; //odbiór z nextiona
uint8_t Uart2_zn;
volatile uint8_t Uart2_i;
volatile uint8_t Uart2_ff;
bool Uart2_free = true;
uint8_t can_ok = 15;		//dzialanie can, dekrementacja w add to bufor 5Hz, odnowienie w can zapi[2]

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	txCAN.DLC = 8;
	txCAN.StdId = 0x608;
	txCAN.RTR = CAN_RTR_DATA;
	txCAN.IDE = CAN_ID_STD;
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_SDIO_SD_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
HAL_GPIO_WritePin(LED_Pin_GPIO_Port, LED_Pin_Pin, 0);	
//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);	

	CAN_filterConfig();
	CAN_filterConfig1();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // musi byc!!
	HAL_TIM_Base_Start_IT(&htim1); //5 Hz docelowo, wazniejsze onformacje na dash
	HAL_TIM_Base_Start_IT(&htim2); //10 Hz przeliczanie
	HAL_TIM_Base_Start_IT(&htim3); //0,5 Hz docelowo, mniej wazne info na dash
	HAL_UART_Receive_IT(&huart2, &Uart2_zn, 1);	//odbior z nextiona
	
  sd_card.init = false;
	sd_card.flaga = 1;
	sd_card.sd_add_buf = false;
	strcpy(sd_card.SD_nazwapliku, "1.TXT\0");
	dash_state = IDLE;
	dash_page = PAGE0;
			nx_val.DC_curr=0;
			nx_val.bat_percent=0;
			nx_val.bat_voltage=0;
			nx_val.controller_temp=0;
			nx_val.engine_temp=0;
			nx_val.rpm=0;
			nx_val.speed=0;
			nx_val.SDO_req=0;
			nx_val.SDO_ans=0;

	nx_val.contactor=false;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
		switch(dash_page){
			case PAGE2:
				IdleRun();
				break;
			case PAGE3:
			case PAGE0:
				Nextion_SendValue(buf_nxt_p, &nx_dowysylki_p, &Uart2_free);
				break;
			case PAGE1:
				Nextion_SendValue(buf_nxt_r, &nx_dowysylki_r, &Uart2_free);
				break;
			default:
				IdleRun();
				break;
		}
	if(dash_state == NEXTION_SD){
		if(sd_card.sd_add_buf){ 																		//co 5 Hz ustawiana ta flaga w timerze
			if(sd_card.flaga) { 																			//rozpoznanie buf_sd1 lub buf_sd2
				AddToBuffor_SD(buf_sd1, &nx_val, &nx_dowysylki_sd);			//dodawanie 1 linii do bufora sd1
				SDZapis(&sd_card, buf_sd1, buf_sd2, &nx_dowysylki_sd);	//zapis na sd (25 linii), co 5 wywolanie
			}
			else {
				AddToBuffor_SD(buf_sd2, &nx_val, &nx_dowysylki_sd);			//dodawanie 1 linii do bufora sd2
				SDZapis(&sd_card, buf_sd2, buf_sd1, &nx_dowysylki_sd);	//zapis na sd (25 linii), co 5 wywolanie
			}
			sd_card.sd_add_buf = false;
		}
	}
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
			//10 = 250 kb/s    5 = 500 kb/s
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 3;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void){

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
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
  htim2.Init.Prescaler = 9999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

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

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Pin_GPIO_Port, LED_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin_Pin */
  GPIO_InitStruct.Pin = LED_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int counter=1;
	static int counter2=1;
	
	txCAN.DLC = 8;
	txCAN.StdId = 0x608;
	txCAN.RTR = CAN_RTR_DATA;
	txCAN.IDE = CAN_ID_STD;
	uint32_t mailbox;
	
	uint8_t frame[8] = {0x40, 0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00};
	//>>>>>>>>>>>>>>>TIM1----- 5 Hz <<<<<<<<<<<<<<<
	if(htim->Instance == TIM1)
	{
		//odpytywanie SDO
		switch(counter){
			case 1:
				//frame[1] = ZAPI_ID_CURR;
				frame[1] = ZAPI_DC_CURR;
				break;
		/*	case 2:
				frame[1] = ZAPI_IQ_CURR;
				break;
			case 3:
				frame[1] = ZAPI_DC_CURR;
				break;
			case 4:
				frame[1] = ZAPI_TORQUE;
				break;*/
		}
		
		HAL_CAN_AddTxMessage(&hcan1, &txCAN, frame, &mailbox);
		//counter++;
		//if(counter >= 5) counter = 1;
		
			nx_val.SDO_req++;		//debug test
	}
	//>>>>>>>>>>>>>>>TIM2----- 5 Hz<<<<<<<<<<<<<<<<<
	if(htim->Instance == TIM2)
	{
		switch(dash_page){
			case PAGE3:
			case PAGE0:
				AddToBuffor_P(buf_nxt_p, &nx_val, &nx_dowysylki_p);
				break;
			case PAGE1:
				AddToBuffor_R(buf_nxt_r, &nx_val, &nx_dowysylki_r);
				break;
			default:
				break;
		}
		
		//sprawdzenie komunikacji CAN i wyzerowanie wartosci
//		if(can_ok) can_ok--;
		
//		if(!can_ok)
//		{
//			memset(&nx_val, 0, sizeof(nx_val));
//		}
		
		//dodawanie do bufora sd 1 linii
		sd_card.sd_add_buf = true;
	}
	//>>>>>>>>>>>>>>>TIM3-----odpytywanie rzadkie 1 HZ<<<<<<<<<<<<<<
	if(htim->Instance == TIM3)
	{
		//odpytywanie SDO
		switch(counter2){
			case 1:
				frame[1] = ZAPI_BAT_VOLTAGE;
			  break;
			case 2:
				frame[1] = ZAPI_CONTR_TEMP;
			  break;
			case 3:
				frame[1] = ZAPI_MOT_TEMP;
			  break;
		}
		
		HAL_CAN_AddTxMessage(&hcan1, &txCAN, frame, &mailbox);
		counter2++;
		if(counter2 >= 4) counter2 = 1;
		
			nx_val.SDO_req++;		//debug test
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint16_t tmp;
	float f_tmp;
	
	uint32_t ID = (CAN_RI0R_STID & hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0].RIR) >> CAN_TI0R_STID_Pos;
	
	switch(ID){
		case CAN_ADR_ZAPI0:
			CanGetMsgData(CAN_ramka[0]);
			nx_val.can_count = 0;					//w AddToBuffor_P inkrementacja, informacja o HV, komunikacji z zapim
			CAN_ramka[0][0] &= 1;
			nx_val.contactor = !(CAN_ramka[0][0]);
		
			tmp = (CAN_ramka[0][2] << 8) + CAN_ramka[0][3];
			nx_val.rpm = (uint16_t) tmp * 0.15f;
			
//			tmp = (uint16_t) nx_val.rpm * 0.02358912f*0.8f;
//			tmp = (uint16_t) nx_val.rpm * 0.02409101617f;
			tmp = (uint16_t) nx_val.rpm *	0.024894050043f;
			nx_val.speed = (uint8_t) tmp;
		
		break;
		
		case CAN_ADR_ZAPI2:
			can_ok = 15;
			CanGetMsgData(CAN_ramka[1]);
			switch (CAN_ramka[1][1]){
				case ZAPI_BAT_VOLTAGE:
					tmp = ((CAN_ramka[1][5] << 8) + CAN_ramka[1][4]) /100;		//scale *100
					nx_val.bat_voltage = (uint8_t) tmp;
				
					f_tmp = (nx_val.bat_voltage-84)*3.05;
					nx_val.bat_percent = (uint8_t) f_tmp;
					break;
				
				case ZAPI_CONTR_TEMP:
					nx_val.controller_temp = CAN_ramka[1][4];
					break;
				
				case ZAPI_MOT_TEMP:
					nx_val.engine_temp = CAN_ramka[1][4];
					break;	
				
				case ZAPI_ID_CURR:
					nx_val.id_curr = ((CAN_ramka[1][5] << 8) + CAN_ramka[1][4]);
					break;
				
				case ZAPI_IQ_CURR:
					nx_val.iq_curr = ((CAN_ramka[1][5] << 8) + CAN_ramka[1][4]);
					break;
				
				case ZAPI_DC_CURR:
					nx_val.DC_curr = ((CAN_ramka[1][5] << 8) + CAN_ramka[1][4]);
					break;
				
				case ZAPI_TORQUE:
					nx_val.torque = ((CAN_ramka[1][5] << 8) + CAN_ramka[1][4]);
					break;
			}
			
				nx_val.SDO_ans++;		//debug test
			break;
		
		case CAN_ADR_SENS0:
			CanGetMsgData(CAN_ramka[2]);	//pot1 - rear
			nx_val.susp_rear.min = (CAN_ramka[2][1] << 8) + CAN_ramka[2][0];
			nx_val.susp_rear.max = (CAN_ramka[2][3] << 8) + CAN_ramka[2][2];
			nx_val.susp_rear.avg = (CAN_ramka[2][5] << 8) + CAN_ramka[2][4];
			break;
		
		case CAN_ADR_SENS1:
			CanGetMsgData(CAN_ramka[3]);	//pot2 - front
			nx_val.susp_front.min = (CAN_ramka[3][1] << 8) + CAN_ramka[3][0];
			nx_val.susp_front.max = (CAN_ramka[3][3] << 8) + CAN_ramka[3][2];
			nx_val.susp_front.avg = (CAN_ramka[3][5] << 8) + CAN_ramka[3][4];
			break;
		
		case CAN_ADR_SENS2:
			CanGetMsgData(CAN_ramka[4]);	//yaw pich roll
			nx_val.imu_data.yaw 	= (CAN_ramka[4][1] << 8) + CAN_ramka[4][0];
			nx_val.imu_data.pitch = (CAN_ramka[4][3] << 8) + CAN_ramka[4][2];
			nx_val.imu_data.roll 	= (CAN_ramka[4][5] << 8) + CAN_ramka[4][4];
			break;
		
		case CAN_ADR_SENS3:
			CanGetMsgData(CAN_ramka[5]);	//yaw pich roll
			nx_val.imu_data.acc_x = (CAN_ramka[5][1] << 8) + CAN_ramka[5][0];
			nx_val.imu_data.acc_y = (CAN_ramka[5][3] << 8) + CAN_ramka[5][2];
			nx_val.imu_data.acc_z	= (CAN_ramka[5][5] << 8) + CAN_ramka[5][4];
			break;
		
		case CAN_ADR_PDM:
			CanGetMsgData(CAN_ramka[6]);	//pdm->temps
//			nx_val.pdm_val.status_field = CAN_ramka[6][0];	//nie uzywane obecnie
			nx_val.pdm_val.temps[0] 		= CAN_ramka[6][1];
			nx_val.pdm_val.temps[1] 		= CAN_ramka[6][2];
			nx_val.pdm_val.temps[2] 		= CAN_ramka[6][3];
			nx_val.pdm_val.temps[3] 		= CAN_ramka[6][4];
//			nx_val.pdm_val.temps[4] 		= CAN_ramka[6][5];	//nie uzywane obecnie
//			nx_val.pdm_val.imd_resistance = (CAN_ramka[6][7] << 8) + CAN_ramka[6][6];
			break;
		
	}
}

void CAN_filterConfig(void)
{
	CAN_FilterTypeDef filterConfig;
	//skala 32 bit jest spoko do Ext ID, do Std ID najlepiej 16 bit, wtedy mamy 2x wiecej ID 
	//w skali 16 bit high i low oznacza 2 osobne ID
	//w skali 32 bit high i low oznacza MSB i LSB danego 32 bitowego rejestru
	//w trybie ID LIST oraz skali 16 bit w kazdym banku mozna ustawic 4 filtry Std ID
	//w trybie ID LIST FilterMask oraz FilterId dotyczy roznych ID
	//w trybie ID MASK FilterMask dotyczy maski a FilterId dotyczy ID
	filterConfig.FilterBank = 1;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.FilterFIFOAssignment = 0;
	filterConfig.FilterIdHigh = (0x0388 << 5);
	filterConfig.FilterIdLow = (0x0588 << 5);
	filterConfig.FilterMaskIdHigh = (0x0700 << 5);
	filterConfig.FilterMaskIdLow = (0x0600 << 5);
	filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

	HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
}

void CAN_filterConfig1(void)
{
	CAN_FilterTypeDef filterConfig;
	//skala 32 bit jest spoko do Ext ID, do Std ID najlepiej 16 bit, wtedy mamy 2x wiecej ID 
	//w skali 16 bit high i low oznacza 2 osobne ID
	//w skali 32 bit high i low oznacza MSB i LSB danego 32 bitowego rejestru
	//w trybie ID LIST oraz skali 16 bit w kazdym banku mozna ustawic 4 filtry Std ID
	//w trybie ID LIST FilterMask oraz FilterId dotyczy roznych ID
	//w trybie ID MASK FilterMask dotyczy maski a FilterId dotyczy ID
	filterConfig.FilterBank = 2;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.FilterFIFOAssignment = 0;
	filterConfig.FilterIdHigh = (0x0610 << 5);
	filterConfig.FilterIdLow = (0x0620 << 5);
	filterConfig.FilterMaskIdHigh = (0x0630 << 5);
	filterConfig.FilterMaskIdLow = (0x0640 << 5);
	filterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

	HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
	if(Uart2_i == 0 && Uart2_zn == 0){
	}
	else{
		Uart2_buf_rx[Uart2_i] = Uart2_zn;
		Uart2_i++;
	}
	if(Uart2_zn == 255){
		Uart2_ff++;
	}
	if(Uart2_ff == 3){
		Process_uart(&dash_state, &dash_page, Uart2_buf_rx, &sd_card);
		Uart2_ff = 0;
		Uart2_i = 0;
	}
	Uart2_free = true;
	HAL_UART_Receive_IT(&huart2, &Uart2_zn, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Uart2_free = true;
}
//diody na nextionie informujace o obecnosci kominukacji
//z danych urzadzeniem
/*void Heartbeat_err(uint8_t device)
{
	char buf[20];
	sprintf(buf, "vis p%,1", device);
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf, 8);
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)k, 3);	
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
