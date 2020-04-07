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

void SystemClock_Config(void);
void Error_Handler(void);
void setupTSC();

int main(void)
{
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	/*Enable GPIOS*/
		__HAL_RCC_GPIOA_CLK_ENABLE();
	 //__HAL_RCC_TSEN();
	 //RCC->AHBENR |= RCC_AHBENR_TSEN;
	//__HAL_RCC_GPIOC_CLK_ENABLE();
	//__HAL_RCC_GPIOB_CLK_ENABLE();
	/*Enable I2C2*/
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	
	/*--------------------------------------------------------------*/
	/*Initalize LEDS*/
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6/7/8/9
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	
	/*Alt Function modes for I2C2*/
	GPIO_InitTypeDef initStr2 = {GPIO_PIN_11 | GPIO_PIN_13, GPIO_MODE_AF_OD, GPIO_SPEED_FREQ_LOW, GPIO_PULLUP};
	HAL_GPIO_Init(GPIOB, &initStr2); // Initialize pins PB11/13

	GPIOB->AFR[1] &= ~((uint32_t)(15 << 12)); //Zero pin 11
	GPIOB->AFR[1] |= (uint32_t)(1 << 12); //Set AF1:0001
	GPIOB->AFR[1] &= ~((uint32_t)(15 << 20)); //Zero pin 13
	GPIOB->AFR[1] |= (uint32_t)(5 << 20); //Set AF5:0101
	
	/* Set pins HIGH*/
	GPIO_InitTypeDef initStr3 = {GPIO_PIN_14, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOB, &initStr3); // Initialize pin PB14
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Start PB14 high

	GPIO_InitTypeDef initStr4 = {GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr4); // Initialize pin PC0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Start PC0 high

	
	/* Set params in TIMINGR*/
	I2C2 -> TIMINGR &= ~(((uint32_t)15 << 28)); //Set PRESC to 0
	I2C2 -> TIMINGR |= ((uint32_t)( 1 << 28 )); //Set 28 - 1
	I2C2 -> TIMINGR &= ~(((uint32_t)15 << 20)); //Set SCLDEL 0
	I2C2 -> TIMINGR |= ((uint32_t)(4 << 20)); // Set 20 - 4
	I2C2 -> TIMINGR &= ~(((uint32_t)15 << 16)); //Set SDADEL 0
	I2C2 -> TIMINGR |= ((uint32_t)(2 << 16)); // Set 16 - 2
	I2C2 -> TIMINGR &= ~(((uint32_t)255 << 8)); //Set SCLH to 0
	I2C2 -> TIMINGR |= ((uint32_t)(15 << 8)); //Set 8 - 16
	I2C2 -> TIMINGR &= ~(((uint32_t)255 << 0)); //Set SCLL to 0
	I2C2 -> TIMINGR |= ((uint32_t)(19 << 0)); //Set 0 - 19

	/*Set PE bit*/
	I2C2 -> CR1 |= 1;
	
	/*Writing the register*/
	I2C2 -> CR2 = 0;
	I2C2 -> CR2 |= ((uint32_t)(0x6B << 1)); //Set to Address of L
	I2C2 -> CR2 &= ~((uint32_t)(255 << 16));
	I2C2 -> CR2 |= ((uint32_t)(2 << 16)); //Set Transmit 1
	I2C2 -> CR2 &= ~((uint32_t)(1 << 10)); //Set RD_WRN
	I2C2 -> CR2 |= ((uint32_t)(1 << 13)); //Set START
	
	while(((I2C2 -> ISR & 2) == 0) && ((I2C2 -> ISR & 8) == 0)){ //Wait in TXIS and NAFLACK Flag
	}
	
	I2C2 -> TXDR = 0x20; //Write adress of gryroscope value

	while(((I2C2 -> ISR & 2) == 0) && ((I2C2 -> ISR & 8) == 0)){ //Wait in TXIS and NAFLACK Flag
	}
	
	I2C2 -> TXDR = 11; //Write gyro crtl reg

	while((I2C2 -> ISR & 64) == 0){ //Wait for TC Flag
	}	
	
	I2C2 -> CR2 |= ((uint32_t)(1 << 14));//Set Stop Bit
	
  while (1)
  {
		HAL_Delay(100);
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(500);
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		
		short x = -11;
		short y = 11;
		
		char x_hi = 0;
		char x_lo = 0;
		char y_hi = 0;
		char y_lo = 0;
		
	
		
		/*Writing the register*/
		I2C2 -> CR2 &= ~((uint32_t)(254));
		I2C2 -> CR2 = 0;
		I2C2 -> CR2 |= ((uint32_t)(0x6B << 1)); //Set to Address of L
		I2C2 -> CR2 &= ~((uint32_t)(255 << 16));
		I2C2 -> CR2 |= ((uint32_t)(1 << 16)); //Set Transmit 1
		I2C2 -> CR2 &= ~((uint32_t)(1 << 10)); //Set RD_WRN
		I2C2 -> CR2 |= ((uint32_t)(1 << 13)); //Set START
		
		
		//while(((I2C2 -> ISR & 2) == 0) && ((I2C2 -> ISR & 8) == 0)){ //Wait in TXIS and NAFLACK Flag
		//}
		
		
		//I2C2 -> TXDR = 0x20; //Write adress of gryroscope value

		while(((I2C2 -> ISR & 2) == 0) && ((I2C2 -> ISR & 8) == 0)){ //Wait in TXIS and NAFLACK Flag
		}
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		
		I2C2 -> TXDR = 0xA8; //x hi
		
		
		while((I2C2 -> ISR & 64) == 0){ //Wait for TC Flag
		}	
			
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		/*Reading the register*/
		I2C2 -> CR2 = 0;
		I2C2 -> CR2 |= ((uint32_t)(0x6B << 1));
		I2C2 -> CR2 &= ~((uint32_t)(255 << 16));
		I2C2 -> CR2 |= ((uint32_t)(2 << 16)); //Set Transmit 1
		I2C2 -> CR2 |= ((uint32_t)(1 << 10)); //Set RD_WRN
		I2C2 -> CR2 |= ((uint32_t)(1 << 13)); //Set START
		
		//Wait until RXNE or NACKF flag
		while(((I2C2 -> ISR & 4) == 0) && ((I2C2 -> ISR & 8) == 0)){
		}
		
		//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		
		x_lo = I2C2 -> RXDR;
		
		//Wait until RXNE or NACKF flag
		while(((I2C2 -> ISR & 4) == 0) && ((I2C2 -> ISR & 8) == 0)){
		}
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		x_hi = I2C2 -> RXDR;
		
		
		while((I2C2 -> ISR & 64) == 0){ //Wait for TC Flag
		}	
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		I2C2 -> CR2 |= ((uint32_t)(1 << 14));//Set Stop Bit

		x = x_hi;
		x = x << 8;
		x = x | x_lo;
		//x = (x_hi << 8) | x_lo;
		
		
		
		/*Writing the register*/
		I2C2 -> CR2 &= ~((uint32_t)(254));
		I2C2 -> CR2 = 0;
		I2C2 -> CR2 |= ((uint32_t)(0x6B << 1)); //Set to Address of L
		I2C2 -> CR2 &= ~((uint32_t)(255 << 16));
		I2C2 -> CR2 |= ((uint32_t)(1 << 16)); //Set Transmit 1
		I2C2 -> CR2 &= ~((uint32_t)(1 << 10)); //Set RD_WRN
		I2C2 -> CR2 |= ((uint32_t)(1 << 13)); //Set START
		
		
		//while(((I2C2 -> ISR & 2) == 0) && ((I2C2 -> ISR & 8) == 0)){ //Wait in TXIS and NAFLACK Flag
		//}
		
		
		//I2C2 -> TXDR = 0x20; //Write adress of gryroscope value

		while(((I2C2 -> ISR & 2) == 0) && ((I2C2 -> ISR & 8) == 0)){ //Wait in TXIS and NAFLACK Flag
		}
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		
		I2C2 -> TXDR = 0xAA; //x hi
		
		
		while((I2C2 -> ISR & 64) == 0){ //Wait for TC Flag
		}	
			
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		/*Reading the register*/
		I2C2 -> CR2 = 0;
		I2C2 -> CR2 |= ((uint32_t)(0x6B << 1));
		I2C2 -> CR2 &= ~((uint32_t)(255 << 16));
		I2C2 -> CR2 |= ((uint32_t)(2 << 16)); //Set Transmit 1
		I2C2 -> CR2 |= ((uint32_t)(1 << 10)); //Set RD_WRN
		I2C2 -> CR2 |= ((uint32_t)(1 << 13)); //Set START
		
		//Wait until RXNE or NACKF flag
		while(((I2C2 -> ISR & 4) == 0) && ((I2C2 -> ISR & 8) == 0)){
		}
		
		//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		
		y_lo = I2C2 -> RXDR;
		
		//Wait until RXNE or NACKF flag
		while(((I2C2 -> ISR & 4) == 0) && ((I2C2 -> ISR & 8) == 0)){
		}
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		y_hi = I2C2 -> RXDR;
		
		
		while((I2C2 -> ISR & 64) == 0){ //Wait for TC Flag
		}	
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6); // Toggle PC8
		//HAL_Delay(100);
		I2C2 -> CR2 |= ((uint32_t)(1 << 14));//Set Stop Bit

		y = y_hi;
		y = y << 8;
		y = y | y_lo;
		
		
		// read x and y
		
		int threshold = 10000;
		
		if (x > threshold)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		if (x < -threshold)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		if (y > threshold)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		if (y < -threshold)
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			
		
		

  }

}
	
void setupTSC() {
    GPIOA->MODER |= (0x2 << (2*1)) + (0x2 << (2*2)); //enable AF for PA1 and PA2
    GPIOA->AFR[0] |= (0x3 << (4*1)) + (0x3 << (4*2)); //enable AF3 for PA1 and PA2

    TSC->IER |= 0x01; //enable end of acquisition interrupt
    TSC->IOSCR |= 0x04; //enable G1_IO3 (PA2) as sampling capacitor
    TSC->IOCCR |= 0x02; //enable G1_IO2 (PA1) as channel
    TSC->IOGCSR |= 0x01; //enable G1 analog group
    TSC->IOHCR &= ~(0x06); //disable hysteresis on PA1 and PA2 
    //TSC->IOASCR |= 0x04; //do I need this?
    NVIC->ISER[0] |= 1 << TS_IRQn; //enable TSC interrupt
    TSC->CR |= 0x01 + 0x00C0 + 0x02; //enable TSC and start acquisition
}

void GetReg() {
	
}

/** System Clock Configuration
*/
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
