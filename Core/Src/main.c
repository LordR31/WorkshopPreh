/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "lcd_st7565_pinconf.h"
#include "lcd_st7565.h"
#include "font.h"
#include "function_display.h"
#include "function_servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_BUFFER_SIZE 10 // size of buffer for storing ADC measurements

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch4_up;

UART_HandleTypeDef huart2;

/* Definitions for Servo */
osThreadId_t ServoHandle;
const osThreadAttr_t Servo_attributes = {
	.name = "Servo",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Joystick */
osThreadId_t JoystickHandle;
const osThreadAttr_t Joystick_attributes = {
	.name = "Joystick",
	.stack_size =
		128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for LCD */
osThreadId_t LCDHandle;
const osThreadAttr_t LCD_attributes = {
	.name = "LCD",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for Counter */
osThreadId_t CounterHandle;
const osThreadAttr_t Counter_attributes = {
	.name = "Counter",
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
void Task_Servo(void *argument);
void Task_Joystick(void *argument);
void Task_LCD(void *argument);
void Task_CounterUp(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t treapta_curenta = 0; // Numar treapta viteza curenta
uint32_t schimba_treapta = 1; // Semnal pentru incrementarea treptelor de viteza

uint32_t instant_ADC_Values[3]; // Stocheaza valorile instante citite de la ADC

uint32_t mod_lucru_sistem = 0; // Variabila este setata in Task-ul de Read_Joystick si folosita atat in Task-ul de Servo cat si cel de LCD pentru a dicta functionarea sistemului

/////////////// Codificare mod_lucru_sistem ///////////////
/// 			0 - idle/stop/reset					    ///
/// 			1 - wipe x1							    ///
/// 			2 - lucru in trepte, treapta++          ///
/// 			3 - lucru in trepte, treapta--          ///
/// 			4 - parbriz                             ///
/// 			5 - luneta								///
///////////////////////////////////////////////////////////

int sistem_initializat = 0; // Opreste orice operatie a servomotorului si a displayului pana cand sistemul nu a fost initializat
int timer = 0;				// Timer incrementat in Task-ul TimerUp pentru diferitele moduri de lucru cu servomotorul
int timer_trepte = 0;		// Timer incrementat in Task-ul TimerUp pentru blocarea/stoparea incrementarii/decrementarii accidentale a treptelor de viteza
int abort_signal = 0;		// Semanl de abort folosit pentru oprirea fortata a modurilor de lucru

void is_system_initialized() // Functie care verifica daca sistemul s-a initializat (prin verificarea daca adc-ul a citit sau nu valori)
{
	if (instant_ADC_Values[0] > 1900 && instant_ADC_Values[1] > 1900 && instant_ADC_Values[2] > 1900) // Daca ADC a putut citii valori de la Joystick
		sistem_initializat = 1;																		  // sistemul se considera initializat si se poate incepe functionarea normala
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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_ADC_Init();
	MX_TIM3_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

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

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of Servo */
	ServoHandle = osThreadNew(Task_Servo, NULL, &Servo_attributes);

	/* creation of Joystick */
	JoystickHandle = osThreadNew(Task_Joystick, NULL, &Joystick_attributes);

	/* creation of LCD */
	LCDHandle = osThreadNew(Task_LCD, NULL, &LCD_attributes);

	/* creation of Counter */
	CounterHandle = osThreadNew(Task_CounterUp, NULL, &Counter_attributes);

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14 | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_14;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */
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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */
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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 48 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000 - 1;
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
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);
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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
	/* DMA1_Ch1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);
	/* DMA1_Ch2_3_DMA2_Ch1_2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPICD_GPIO_Port, SPICD_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, BL_Pin | SPIRST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPICS_GPIO_Port, SPICS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SPICD_Pin */
	GPIO_InitStruct.Pin = SPICD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPICD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BL_Pin SPIRST_Pin */
	GPIO_InitStruct.Pin = BL_Pin | SPIRST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SPICS_Pin */
	GPIO_InitStruct.Pin = SPICS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPICS_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task_Servo */
/**
 * @brief  Function implementing the Servo thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_Servo */
void Task_Servo(void *argument)
{
	/* USER CODE BEGIN 5 */

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////   Implementare Task Servomotor   //////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); // Porneste PWM pe timer 3 canal 4 (pin PB1)

	// Functie auxiliara pentru modul de lucru in trepte de viteza
	// Este apelata din interiorul for(;;) pe case-urile 2 si 3 ale switch-ului
	void lucru_in_trepte()
	{
		switch (treapta_curenta)
		{
		case 0:																  // Case 0: se intra atunci cand treapta de viteza a ajuns la 0 si se doreste oprirea stergatorului
			mod_lucru_sistem = Servo_master_function(0, &timer, &abort_signal); // Se paseaza la functia master Servo_master_function() modul de lucru (0, oprire), pointerul catre timer si pointerul catre semnalul de abort
			break;
		case 1:											   // Case 1: se intra atunci cand se ajunge la treata 1 de viteza
			Servo_master_function(2, &timer, &abort_signal); // Se paseaza la functia master Servo_master_function() modul de lucru (2, treapta 1), pointerul catre timer si pointerul catre semnalul de abort
			break;
		case 2:											   // Case 2: se intra atunci cand se ajunge la treapta 2 de viteza
			Servo_master_function(3, &timer, &abort_signal); // Se paseaza la functia master Servo_master_function() modul de lucru (3, treapta 2), pointerul catre timer si pointerul catre semnalul de abort
			break;
		case 3:											   // Case 3: se intra atunci cand se ajunge la treapta 3 de viteza
			Servo_master_function(4, &timer, &abort_signal); // Se paseaza la functia master Servo_master_function() modul de lucru (4, treapta 3), pointerul catre timer si pointerul catre semnalul de abort
			break;
		}
	}

	for (;;)
	{
		timer_trepte = 0; // reseteaza de fiecare data timer_trepte

		switch (mod_lucru_sistem) // Primeste de la Task-ul de Read_Joystick modul de lucru (mod_lucru_sistem)
		{
		case 0:											   // Case 0: modul idle sau reset. Servo motorul este adus si sta in pozitia 0
			Servo_master_function(0, &timer, &abort_signal); // Se paseaza la functia master Servo_master_function() modul de lucru (0, oprire), pointerul catre timer si pointerul catre semnalul de abort
			abort_signal = 0;							   // In cazul in care a fost folosit semnalul abort, acesta se readuce la 0 pentru a se putea continua functionarea normala a sistemului
			break;
		case 1:																  // Case 1: wipe x1
			mod_lucru_sistem = Servo_master_function(1, &timer, &abort_signal); // Se paseaza la functia master Servo_master_function() modul de lucru (1, wipe x1), pointerul catre timer si pointerul catre semnalul de abort
			break;
		case 2:												 // Case 2: lucru pe trepte de viteza, modul de incrementare a treptei de viteza
			if (treapta_curenta < 3 && schimba_treapta == 1) // Se verifica daca este necesara incrementarea treptei de viteza (pana la treapta_curenta = 3) folosind semnalul schimba_treapta primit de la task-ul Read_Joystick
			{
				treapta_curenta++;	 // Se incrementeaza treapta
				schimba_treapta = 0; // Se reseteaza semnalul de schimba_treapta
			}
			lucru_in_trepte(); // Se foloseste functia de lucru_in_trepte()

			break;
		case 3:												 // Case 3: lucru in trepte de viteza, modul de decrementare a treptei de viteza
			if (treapta_curenta > 0 && schimba_treapta == 1) // Se verifica daca este necesara decrementarea treptei de viteza (pana la treapta_curenta = 0) folosind semnalul schimba_treapta primit de la task-ul de Read_Joystick
			{
				treapta_curenta--;	 // Se decrementeaza treapta
				schimba_treapta = 0; // Se reseteaza semnalul de schimba_treapta
			}
			lucru_in_trepte(); // Se foloseste functia de lucru_in_trepte()
			break;
		case 4:																  // Case 4: modul spalare parbriz
			mod_lucru_sistem = Servo_master_function(5, &timer, &abort_signal); // Se paseaza la functia master Servo_master_function() moodul de lucru(5, spalare parbriz), pointerul catre timer si pointerul catre semnalul de abort
			break;
		case 5:																  // Case 5: modul spalare luneta
			mod_lucru_sistem = Servo_master_function(6, &timer, &abort_signal); // Se paseaza la functia master Servo_master_function() moodul de lucru(6, spalare luneta), pointerul catre timer si pointerul catre semnalul de abort
			break;
		}

		osDelay(1); // Task-ul se reapeleaza automat la 1ms
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_Joystick */
/**
 * @brief Function implementing the Joystick thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_Joystick */
void Task_Joystick(void *argument)
{
	/* USER CODE BEGIN Task_Joystick */

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////   Implementare Task Joystick   ////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	HAL_ADC_Start_DMA(&hadc, instant_ADC_Values, 3); // Initializeaza ADC-ul si ii stocheaza valorile citite instante in

	for (;;)
	{
		if (sistem_initializat == 0) // Daca sistemul NU este initializat apeleaza functia is_system_initialized() care verifica daca ADC-ul citeste sau nu valori
		{
			is_system_initialized();
		}

		if (sistem_initializat) // Dupa ce sistemul a fost initializat se incepe functionarea normala
		{
			if (instant_ADC_Values[2] <= 1000) // Cazul Buton Apasat
			{
				mod_lucru_sistem = 0; // Modul de lucru se trece in 0 (oprire/reset)
				abort_signal = 1;	  // Semnalul de abort se face 1 pentru a oprii orice operatie
				treapta_curenta = 0;  // Se reseteaza treapta de viteza
			}

			if (instant_ADC_Values[0] <= 1000 && treapta_curenta > 0) // Cazul Joystick Jos in timpul functionarii pe trepte de viteza
			{
				mod_lucru_sistem = 3;	 // Modul de lucru se trece in 3 (lucru in trepte, cazul decrementare treapta de viteza)
				if (timer_trepte > 150)	 // Se verifica daca a trecut timpul minim de 150ms pentru a nu se decrementa accidental de mai multe ori treapta de viteza
					schimba_treapta = 1; // In cazul in care timpul minim a expirat, semnalul schimba_treapta se trece in 1 pentru a permite decrementarea treptei de viteza
			}
			else
			{
				if (instant_ADC_Values[0] <= 1000 && treapta_curenta == 0) // Cazul: Joystick Jos in afara functionarii pe trepte de viteza
				{
					mod_lucru_sistem = 1; // Modul de lucru se trece in 1 (wipe x1)
				}
			}

			if (instant_ADC_Values[0] >= 3500) // Cazul Joystick Sus
			{
				mod_lucru_sistem = 2;	 // Modul de lucru se trece in 2 (lucru pe trepte de viteza, cazul incrementare treapta de viteza)
				if (timer_trepte > 150)	 // Se verifica daca a trecut timpul minim de 150ms pentru a nu se incrementa accidental de mai multe ori treapta de viteza
					schimba_treapta = 1; // In cazul in care timpul minim a expirat, semnalul schimba_treapta se trece in 1 pentru a permite incrementarea treptei de viteza
			}

			if (instant_ADC_Values[1] <= 1000) // Cazul Joystick Stanga
			{
				mod_lucru_sistem = 4; // Modul de lucru se trece in 4 (modul spalare parbriz)
			}

			if (instant_ADC_Values[1] >= 3500) // Cazul Joystick Dreapta
			{
				mod_lucru_sistem = 5; // Modul de lucru se trece in 4 (modul spalare luneta)
			}
		}
		osDelay(1); // Task-ul se reapeleaza automat la 1ms
	}
	/* USER CODE END Task_Joystick */
}

/* USER CODE BEGIN Header_Task_LCD */
/**
 * @brief Function implementing the LCD thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_LCD */
void Task_LCD(void *argument)
{
	/* USER CODE BEGIN Task_LCD */

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////   Implementare Task Display   ///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// Se initializeaza display-ul si se activeaza backlight-ul
	st7565_init();
	st7565_backlight_enable();

	// Functie auxiliara pentru modul de lucru in trepte de viteza
	// Este apelata din interiorul for(;;) pe case-urile 2 si 3 ale switch-ului
	void lucru_in_trepte_LCD()
	{
		switch (treapta_curenta)
		{
		case 0:							// Case 0: se intra atunci cand treapta de viteza a ajuns la 0 si se doreste oprirea stergatorului
			Display_master_function(0); // Se apeleaza functia master Display_mod_lucru_Display() cu parametrul 0 (reset/oprire)
			break;
		case 1:							// Case 1: se intra atunci cand se ajunge la treapta 1 de viteza
			Display_master_function(2); // Se apeleaza functia master Display_mod_lucru_Display() cu parametrul 2 (treapta 1)
			break;
		case 2:							// Case 2: se intra atunci cand se ajunge la treapta 2 de viteza
			Display_master_function(3); // Se apeleaza functia master Display_mod_lucru_Display() cu parametrul 3 (treapta 2)
			break;
		case 3:							// Case 3: se intra atunci cand se ajunge la treapta 3 de viteza
			Display_master_function(4); // Se apeleaza functia master Display_mod_lucru_display() cu paramentrul 4 (treapta 4)
			break;
		}
	}

	for (;;)
	{

		switch (mod_lucru_sistem)
		{
		case 0:							// Case 0: modul idle/reset
			Display_master_function(0); // Se apeleaza functia master Display_mod_lucru_display() cu parametrul 0 (idle/reset)
			break;
		case 1:							// Case 1: modul wipe x1
			Display_master_function(1); // Se apeleaza functia master Display_mod_lucru_display() cu parametrul 1 (wipe x1)
			break;
		case 2:					   // Case 2: modul de lucru in trepte, cazul incrementare treapta de viteza
			lucru_in_trepte_LCD(); // Se apeleaza functia lucru_in_trepte_LCD()
			break;
		case 3:					   // Case 3: modul de lucru in trepte, cazul decrementare treapta de viteza
			lucru_in_trepte_LCD(); // Se apeleaza functia lucru_in_trepte_LCD()
			break;
		case 4:							// Case 4: modul spalare parbriz
			Display_master_function(5); // Se apeleaza functia master Display_mod_lucru_display() cu parametrul 4 (spalare parbriz)
			break;
		case 5:							// Case 5: modul spalare luneta
			Display_master_function(6); // Se apeleaza functia master Display_mod_lucru_display() cu parametrul 5 (spalare luneta)
			break;
		}
		osDelay(1); // Task-ul se reapeleaza automat la 1ms
	}
	/* USER CODE END Task_LCD */
}

/* USER CODE BEGIN Header_Task_CounterUp */
/**
 * @brief Function implementing the Counter thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task_CounterUp */
void Task_CounterUp(void *argument)
{
	/* USER CODE BEGIN Task_CounterUp */
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////   Implementare Task TimerUp   ///////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	for (;;)
	{
		timer++;		// Incrementeaza timer pentru lucrul cu servomotorul
		timer_trepte++; // Incrementeaza timer_trepte pentru evitarea incrementarii/decrementarii de mai multe ori a treptei de viteza 

		osDelay(1);		// Task-ul se reapeleaza la 1ms astfel ca ambele timere masoara in ms
	}
	/* USER CODE END Task_CounterUp */
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

#ifdef USE_FULL_ASSERT
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
