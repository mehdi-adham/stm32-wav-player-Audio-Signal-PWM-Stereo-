/**
 ******************************************************************************
 * @file    task.h
 * @created 11/02/1398 9:01:22 AM
 * @author  Mehdi Adham
 * @brief   This file contains all the functions Event
 ******************************************************************************
 */

#include "main.h"
#include "task.h"


extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim4_ch1;



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart -> Instance == USART1){

	}
	UNUSED(huart);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart -> Instance == USART2){
	}

	if(huart ->Instance == USART1){


	}


	UNUSED(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart ->Instance == USART1){

	}
	UNUSED(huart);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim -> Instance == TIM4){


	}

	UNUSED(htim);
}



/*
void DMA_IRQHandler(DMA_HandleTypeDef *hdma){

}*/
