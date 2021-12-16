/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*
 *https://ccrma.stanford.edu/courses/422-winter-2014/projects/WaveFormat/
Offset  Size  Name             Description

The canonical WAVE format starts with the RIFF header:

0         4   ChunkID          Contains the letters "RIFF" in ASCII form
                               (0x52494646 big-endian form).
4         4   ChunkSize        36 + SubChunk2Size, or more precisely:
                               4 + (8 + SubChunk1Size) + (8 + SubChunk2Size)
                               This is the size of the rest of the chunk
                               following this number.  This is the size of the
                               entire file in bytes minus 8 bytes for the
                               two fields not included in this count:
                               ChunkID and ChunkSize.
8         4   Format           Contains the letters "WAVE"
                               (0x57415645 big-endian form).

The "WAVE" format consists of two subchunks: "fmt " and "data":
The "fmt " subchunk describes the sound data's format:

12        4   Subchunk1ID      Contains the letters "fmt "
                               (0x666d7420 big-endian form).
16        4   Subchunk1Size    16 for PCM.  This is the size of the
                               rest of the Subchunk which follows this number.
20        2   AudioFormat      PCM = 1 (i.e. Linear quantization)
                               Values other than 1 indicate some
                               form of compression.
22        2   NumChannels      Mono = 1, Stereo = 2, etc.
24        4   SampleRate       8000, 44100, etc.
28        4   ByteRate         == SampleRate * NumChannels * BitsPerSample/8
32        2   BlockAlign       == NumChannels * BitsPerSample/8
                               The number of bytes for one sample including
                               all channels. I wonder what happens when
                               this number isn't an integer?
34        2   BitsPerSample    8 bits = 8, 16 bits = 16, etc.
          2   ExtraParamSize   if PCM, then doesn't exist
          X   ExtraParams      space for extra parameters

The "data" subchunk contains the size of the data and the actual sound:

36        4   Subchunk2ID      Contains the letters "data"
                               (0x64617461 big-endian form).
40        4   Subchunk2Size    == NumSamples * NumChannels * BitsPerSample/8
                               This is the number of bytes in the data.
                               You can also think of this as the size
                               of the read of the subchunk following this
                               number.
44        *   Data             The actual sound data.
 * */

typedef struct{
	uint8_t RIFF_ID [4]; // "riff"
	int SIZE; //
	uint8_t WAV_ID [4]; // "WAVE"
	uint8_t FMT_ID [4]; // "fmt id"
	int FMT_SZ; 		// fmt
	int FORMAT; 		//
	short CHANNELS; 	// Channels
	int SampleRate; 	// Sample per second
	int ByteRate; 		// Average Byte per second
	short BLOCK_SZ; 	// CHANNELS * (BIT>>3)
	short BitsPerSample; // BITS
	uint8_t DATA_ID [4]; // "data"
	int DATA_SZ; //

}WavHeader_typedef;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLOCK 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_ch1;
DMA_HandleTypeDef hdma_tim4_ch2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
volatile uint8_t fp = 0;
volatile int NumOfBLOCK = 0;
volatile uint8_t btn_flag = 1;
uint8_t Left[2][BLOCK/2];
uint8_t Right[2][BLOCK/2];
uint8_t Wave[2][BLOCK];
uint8_t WaveHeaderbuff[46];
WavHeader_typedef headerInfo;
volatile uint8_t EndOffle_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Stop(void);
void play(void);
void pause(void);
void Next(void);
void Preview(void);
void clearBuff(void);
void GetHeaderInfo(WavHeader_typedef *Out_WavHeaderInfo ,uint8_t *pInput);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MyEvent1(DMA_HandleTypeDef *_hdma){

	NumOfBLOCK++;
	if((int)(headerInfo.DATA_SZ/(BLOCK)) <= (int)(NumOfBLOCK))
	{
		EndOffle_flag = 1;
	}
	else
	{
		fp = 1-fp;

		// Convert data to PWM For Audio Signal
		HAL_DMA_Abort(&hdma_tim4_ch1);
		HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)Right[1-fp], (uint32_t) &(TIM2->CCR1), BLOCK/2);

		HAL_DMA_Abort(&hdma_tim4_ch2);
		HAL_DMA_Start_IT(&hdma_tim4_ch2, (uint32_t)Left[1-fp], (uint32_t) &(TIM2->CCR2), BLOCK/2);


		// Get Next BLOCK Data From UART
		HAL_UART_Receive_DMA(&huart1, Wave[fp], BLOCK);
		HAL_UART_Transmit(&huart1, (uint8_t *)"A", 1, 100);


		// Extract Left/Right Channel Data
		for(int i=0 ,j=0 ;i<BLOCK ;i+=2 ,j++){
			Right[fp][j] = Wave[fp][i];//right channel
			Left[fp][j] = Wave[fp][i+1];//left channel
		}
	}
}

void MyEvent2(DMA_HandleTypeDef *_hdma){


}

void MyEvent3(DMA_HandleTypeDef *_hdma){
	/* 4096 2048*/
	if(NumOfBLOCK==0){
		for(int i=0 ,j=0 ;i<BLOCK ;i+=2 ,j++){
			Right[fp][j] = Wave[fp][i];//right channel
			Left[fp][j] = Wave[fp][i+1];//left channel
		}
	}
}

void Stop(void){
	btn_flag = 1;
	HAL_UART_Transmit(&huart1, (uint8_t *)"C", 1, 100);

	HAL_Delay(100);

	clearBuff();

	HAL_UART_DMAStop(&huart1);
	NumOfBLOCK=0;
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_DISABLE);
	HAL_DMA_Abort(&hdma_tim4_ch1);
	HAL_DMA_Abort(&hdma_tim4_ch2);

}

void play(void){


	/**************************get WavHeader Info*******************/
	if(NumOfBLOCK == 0)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)"B", 1, 10);
		HAL_UART_Receive_DMA(&huart1, WaveHeaderbuff, 45 );

		HAL_Delay(100);

		GetHeaderInfo(&headerInfo, WaveHeaderbuff);

		if(headerInfo.SampleRate == 8000) {
			TIM4->PSC = 72-1;
			TIM4->ARR = 125-1;
		}
		else if(headerInfo.SampleRate == 11025) {
			TIM4->PSC = 72-1;
			TIM4->ARR = 91-1;
		}
		else if(headerInfo.SampleRate == 16000) {
			TIM4->PSC = 36-1;
			TIM4->ARR = 125-1;
		}
		else if(headerInfo.SampleRate == 22050) {
			TIM4->PSC = 9-1;
			TIM4->ARR = 363-1;
		}
		else if(headerInfo.SampleRate == 32000) {
			TIM4->PSC = 9-1;
			TIM4->ARR = 250-1;
		}
		if(headerInfo.SampleRate == 44100){
			TIM4->PSC = 9-1;
			TIM4->ARR = 181-1;
		}
		else if(headerInfo.SampleRate == 48000) {
			TIM4->PSC = 9-1;
			TIM4->ARR = 166-1;
		}
	}



	/*****************************get data***************************/

	if(NumOfBLOCK == 0)
	{
		HAL_UART_Receive_DMA(&huart1, Wave[fp], BLOCK);
		HAL_UART_Transmit(&huart1, (uint8_t *)"A", 1, 10);

		HAL_Delay(100);
	}

	fp = 1-fp;

	HAL_UART_Receive_DMA(&huart1, Wave[fp], BLOCK);
	HAL_UART_Transmit(&huart1, (uint8_t *)"A", 1, 10);


	/**************************convert data to PWM*******************/

	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_ENABLE);
	HAL_DMA_Start_IT(&hdma_tim4_ch1, (uint32_t)Right[1-fp], (uint32_t) &(TIM2->CCR1), BLOCK/2);
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_ENABLE);
	HAL_DMA_Start_IT(&hdma_tim4_ch2, (uint32_t)Left[1-fp], (uint32_t) &(TIM2->CCR2), BLOCK/2);

}

void pause(void){

	//HAL_UART_DMAStop(&huart1);
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_DISABLE);
	HAL_DMA_Abort(&hdma_tim4_ch1);
	HAL_DMA_Abort(&hdma_tim4_ch2);
}

void Next(void){
	btn_flag=1;
	HAL_UART_Transmit(&huart1, (uint8_t *)"N", 1, 100);


	HAL_Delay(100);

	for(int x=0 ;x<BLOCK ;x++){
		Wave[0][x] = Wave[1][x] = 128; // bitRate/2
	}

	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_DISABLE);
	HAL_DMA_Abort(&hdma_tim4_ch1);
	HAL_DMA_Abort(&hdma_tim4_ch2);

	NumOfBLOCK=0;
	play();
}

void Preview(void){
	btn_flag=1;
	HAL_UART_Transmit(&huart1, (uint8_t *)"P", 1, 100);


	HAL_Delay(100);

	for(int x=0 ;x<BLOCK ;x++){
		Wave[0][x] = Wave[1][x] = 128; // bitRate/2
	}

	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM2, TIM_CHANNEL_2, TIM_CCx_DISABLE);
	HAL_DMA_Abort(&hdma_tim4_ch1);
	HAL_DMA_Abort(&hdma_tim4_ch2);

	NumOfBLOCK=0;
	play();
}

void clearBuff(void){
	for(int x=0 ;x<BLOCK ;x++){
		Wave[0][x] = Wave[1][x] = 128;
	}
}

void GetHeaderInfo(WavHeader_typedef *Out_WavHeaderInfo ,uint8_t *pInput){

	Out_WavHeaderInfo->RIFF_ID[0] = WaveHeaderbuff[0];
	Out_WavHeaderInfo->RIFF_ID[1] = WaveHeaderbuff[1];
	Out_WavHeaderInfo->RIFF_ID[2] = WaveHeaderbuff[2];
	Out_WavHeaderInfo->RIFF_ID[3] = WaveHeaderbuff[3];

	//...

	Out_WavHeaderInfo->SampleRate = (WaveHeaderbuff[24])+
			((WaveHeaderbuff[25]) << 8) +
			((WaveHeaderbuff[26]) << 16) +
			((WaveHeaderbuff[27]) << 24) ;

	//...

	Out_WavHeaderInfo->DATA_SZ = (WaveHeaderbuff[40])+
			((WaveHeaderbuff[41]) << 8) +
			((WaveHeaderbuff[42]) << 16) +
			((WaveHeaderbuff[43]) << 24) ;

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
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	//stereo
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

	//just set before HAL_DMA_Start_IT function
	HAL_DMA_RegisterCallback(&hdma_tim4_ch1, HAL_DMA_XFER_CPLT_CB_ID, &MyEvent1);
	HAL_DMA_RegisterCallback(&hdma_tim4_ch2, HAL_DMA_XFER_CPLT_CB_ID, &MyEvent2);

	HAL_DMA_RegisterCallback(&hdma_usart1_rx, HAL_DMA_XFER_CPLT_CB_ID, &MyEvent3);


	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2);
	__HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if(EndOffle_flag==1){
			EndOffle_flag=0;
			Stop();
			btn_flag = 3;
			play();
		}


		//press Preview button => Preview
		if (HAL_GPIO_ReadPin(PREVIEW_GPIO_Port, PREVIEW_Pin) == GPIO_PIN_RESET) {

			Preview();

			HAL_Delay(200);
		}


		//press Next button => Next
		if (HAL_GPIO_ReadPin(Next_GPIO_Port, Next_Pin) == GPIO_PIN_RESET) {

			Next();

			HAL_Delay(200);
		}



		//press STOP button => STOP
		if (HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin) == GPIO_PIN_RESET) {

			Stop();

			HAL_Delay(200);
		}



		//press Play/Pause button => Play
		if (HAL_GPIO_ReadPin(PLAY_GPIO_Port, PLAY_Pin) == GPIO_PIN_RESET && btn_flag == 1) {

			btn_flag = 2;
			play();

			HAL_Delay(200);
		}


		//press Play/Pause button => Pause
		if(HAL_GPIO_ReadPin(PLAY_GPIO_Port, PLAY_Pin) == GPIO_PIN_RESET && btn_flag == 3){

			btn_flag = 4;
			pause();

			HAL_Delay(200);
		}


		//release Play/Pause button
		if (HAL_GPIO_ReadPin(PLAY_GPIO_Port, PLAY_Pin) == GPIO_PIN_SET && (btn_flag == 2 || btn_flag == 4)){

			//now play
			if(btn_flag == 2)
				btn_flag = 3;
			//now pause
			else if(btn_flag == 4)
				btn_flag = 1;

			HAL_Delay(200);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 256;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 128;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

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
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 72-1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 181-1;
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
	if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* DMA1_Channel5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PLAY_Pin */
	GPIO_InitStruct.Pin = PLAY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PLAY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PREVIEW_Pin Next_Pin STOP_Pin */
	GPIO_InitStruct.Pin = PREVIEW_Pin|Next_Pin|STOP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

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
