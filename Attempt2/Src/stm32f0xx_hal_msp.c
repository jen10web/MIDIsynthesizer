/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f0xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief TSC MSP Initialization
* This function configures the hardware resources used in this example
* @param htsc: TSC handle pointer
* @retval None
*/
void HAL_TSC_MspInit(TSC_HandleTypeDef* htsc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htsc->Instance==TSC)
  {
  /* USER CODE BEGIN TSC_MspInit 0 */

  /* USER CODE END TSC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TSC_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TSC GPIO Configuration    
    PA2     ------> TSC_G1_IO3
    PA3     ------> TSC_G1_IO4
    PA6     ------> TSC_G2_IO3
    PA7     ------> TSC_G2_IO4
    PB0     ------> TSC_G3_IO2
    PB1     ------> TSC_G3_IO3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TSC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TSC interrupt Init */
    HAL_NVIC_SetPriority(TSC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TSC_IRQn);
  /* USER CODE BEGIN TSC_MspInit 1 */

  /* USER CODE END TSC_MspInit 1 */
  }

}

/**
* @brief TSC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htsc: TSC handle pointer
* @retval None
*/
void HAL_TSC_MspDeInit(TSC_HandleTypeDef* htsc)
{
  if(htsc->Instance==TSC)
  {
  /* USER CODE BEGIN TSC_MspDeInit 0 */

  /* USER CODE END TSC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TSC_CLK_DISABLE();
  
    /**TSC GPIO Configuration    
    PA2     ------> TSC_G1_IO3
    PA3     ------> TSC_G1_IO4
    PA6     ------> TSC_G2_IO3
    PA7     ------> TSC_G2_IO4
    PB0     ------> TSC_G3_IO2
    PB1     ------> TSC_G3_IO3 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1);

    /* TSC interrupt DeInit */
    HAL_NVIC_DisableIRQ(TSC_IRQn);
  /* USER CODE BEGIN TSC_MspDeInit 1 */

  /* USER CODE END TSC_MspDeInit 1 */
  }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration    
    PC4     ------> USART3_TX
    PC5     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_4_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PC4     ------> USART3_TX
    PC5     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    /* USART3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART3_4_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
