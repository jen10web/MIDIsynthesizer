/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program MIDI Synthesis 
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include <stdlib.h>
#include <stdio.h> /* for printf */

void SystemClock_Config(void);
void Error_Handler(void);
void setupTSC(void);
volatile int rupt = 0;
volatile unsigned int AV;
void Transmit(char input);
void TransmitString(char input[]);
volatile int globalReadReg = 0;
volatile int globalNewData = 0;

int main(void)
{
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
	/*Enable GPIOS*/
		__HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
   	__HAL_RCC_TSC_CLK_ENABLE(); 
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	/*Initalize LEDS*/
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6/7/8/9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
  
	/*UART*/
	GPIOC->MODER |= 0x00000000;
	GPIOC->MODER |= 0x00055A00;
	GPIOC->AFR[0] |= 0x00110000;
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200; 
	USART3->CR1 |= 0x2c;
	USART3->CR1 |= 0x1;
	//Initialize NVIC
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);

	/*TSC*/
	setupTSC();

  while (1)
  {

		//Manually Discharge capacitor by clearing IODEF in CR REG - 1ms delay
		TSC->CR &= ~(0x10); 
		HAL_Delay(1);
		//SET IODEF high if interlaced or low if push button?
		//TSC->CR &= ~(0x10); 
		TSC->CR |= 0x10; 
		//Select channel want to sample in IO_CCR: IO_GIO3
		TSC->CR |= 0x02; //Start New Acquistion
		
		while(!rupt){
		}
		rupt = 0;
    TSC->CR |= 0x02; //restart acquisition
		
		//Read value from TSC and light up LED: MAX 16383
		if (AV <= 4000)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		else if (AV <= 8000 )
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		else if (AV <= 12000)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		else if (AV <= 16383)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		
  }

}
	
void setupTSC(void) {	
	
	/*TSC Pins Initialization*/
	GPIO_InitTypeDef initStr2 = {GPIO_PIN_1, GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_LOW, GPIO_PULLUP};
	GPIO_InitTypeDef initStr3 = {GPIO_PIN_2, GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_LOW, GPIO_PULLUP};
	HAL_GPIO_Init(GPIOA, &initStr2); // Initialize pins PA1
	HAL_GPIO_Init(GPIOA, &initStr3); // Initialize pins PA2
	
	TSC->CR = TSC_CR_PGPSC_2 | TSC_CR_PGPSC_0 | TSC_CR_CTPH_0 | TSC_CR_CTPL_0
  | TSC_CR_MCV_2 | TSC_CR_MCV_1 | TSC_CR_TSCE; 
	
	GPIOA->AFR[0] &= ~((uint32_t)(15 << 4)); //Zero pin 1
	GPIOA->AFR[0] |= (uint32_t)(3 << 4); //Set AF3:0011
	GPIOA->AFR[0] &= ~((uint32_t)(15 << 8)); //Zero pin 2
	GPIOA->AFR[0] |= (uint32_t)(3 << 8); //Set AF3:0011

	TSC->IER= 0x01; //enable end of acquisition interrupt
	
	TSC->IOSCR |= 0x04; //enable G1_IO3 (PA2) as sampling capacitor
	TSC->IOCCR |= 0x02; //enable G1_IO2 (PA1) as channel/sense pin
	TSC->IOGCSR |= 0x01; //enable G1 analog group
	TSC->IOHCR &= ~(0x06); //disable hysteresis on PA1 and PA2 
	
	TSC->IOASCR |= 0x04; //Set GI_I03 analog switch closed 
	NVIC_EnableIRQ(TSC_IRQn); //Enable TSC Interrupt
	//TSC->CR |= 0x00C0; //Define Maximum number of count pulses
	TSC->CR |= 0x01; //Enable TSC
		
}

void TSC_IRQHandler(void) {

		AV = TSC->IOGXCR[0]; /* Get G1 counter value */
		char str[12];
    sprintf(str, "%d\t", AV);
		TransmitString(str);
		rupt = 1;
		TSC->ICR |= 0x03; //clear interrupts
}
void Transmit(char input)
{
	while( !(USART3->ISR & 0x80)  )//flag is not set
	{
		// do nothing
	}	
	
	USART3->TDR = input;
}


void TransmitString(char input[])
{
	int i = 0;
	while (input[i] != '\0')
	{
		Transmit(input[i]);
		i++;
	}
}

void USART3_4_IRQHandler(void)
{
	globalReadReg = USART3->RDR;
	globalNewData = 1;
}

/** System Clock Configuration*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
