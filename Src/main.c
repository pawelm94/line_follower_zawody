/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t czujniki1[4], czujniki2[4];
int treshold=800;
int Kp=60;
int Kd=1500;
int W[10]={-200,-80,-40,-10,-1,1,10,40,80,200};
float P=0, D=0,error=0, previous_error=0, uchyb=0, poprzedni_uchyb=0, PD_value=0, motor_l=0, motor_p=0;
int e=0;
int i=0;
int k=0;
int motor_initial=600;
int predkosc=500;
int dyf=1;
int rdyf=20;
uint8_t Received;
uint8_t data[50];// Tablica przechowujaca wysylana wiadomosc.
uint16_t size = 0; // Rozmiar wysylanej wiadomosci ++cnt; // Zwiekszenie licznika wyslanych wiadomosci.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) czujniki1, 4);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *) czujniki2, 4);
//
  HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  HAL_UART_Receive_IT(&huart1, &Received, 1);
	  switch(atoi(&Received)){
	  	  case 0:
	  		  k=0;
		  break;
	  	  case 1:
	  		  k=1;
	  		  break;

	  	  case 2:

	  		  predkosc=predkosc+10;
	  		  motor_initial=motor_initial+10;
	  		  size = sprintf(data, "%d. \n\r",predkosc);
	  		  HAL_UART_Transmit_IT(&huart1, data, size);
	  		  if(k==0)
	  		  {
	  			  Received='0';
	  		  }
	  		  if(k==1)
	  		  {
	  			  Received='1';
	  		  }

		  break;

	  	  case 3:
	  		  predkosc=predkosc-10;
	  		  motor_initial=motor_initial-10;
	  		  size = sprintf(data, "%d. \n\r",predkosc);
	  		  HAL_UART_Transmit_IT(&huart1, data, size);
	  		  if(k==0)
				  {
					  Received='0';
				  }
				  if(k==1)
				  {
				  	  Received='1';
				  }
		  break;

	  	  case 4:
	  		  Kp=Kp+1;
	  		  size = sprintf(data, "%d. \n\r",Kp);
	  		  HAL_UART_Transmit_IT(&huart1, data, size);
	  		  if(k==0)
	  			  {
	  				  Received='0';
	  			  }
	  			  if(k==1)
	  			  {
	  			  	  Received='1';
	  			  }
	  	  break;

	  	  case 5:
	  		  Kp=Kp-1;
	  		  size = sprintf(data, "%d. \n\r",Kp);
	  		  HAL_UART_Transmit_IT(&huart1, data, size);
	  		  if(k==0)
	 			  {
	 				  Received='0';
	 			  }
	 			  if(k==1)
	 			  {
	 			  	  Received='1';
	 			  }
	 	 break;

	  	 case 6:
	  		  Kd=Kd+50;
	  		  size = sprintf(data, "%d. \n\r",Kd);
	  		  HAL_UART_Transmit_IT(&huart1, data, size);
	  		  if(k==0)
				  {
					  Received='0';
				  }
				  if(k==1)
				  {
				  	  Received='1';
				  }
		 break;

	  	 case 7:
	  		 Kd=Kd-50;
	  		 size = sprintf(data, "%d. \n\r",Kd);
	  		 HAL_UART_Transmit_IT(&huart1, data, size);
	  		 if(k==0)
				  {
					  Received='0';
				  }
				  if(k==1)
				  {
				  	  Received='1';
				  }
		 break;
	  }

	  if(k==0)
	  {
		  TIM1->CCR2 = 0;
		  TIM1->CCR3 = 0;
	  }

	  if(k==1)
	  {
		  i=0;
	      error=0;
	      if(czujniki1[2]>treshold)
	  	  {
	    	  error+=W[1];
	  		  i++;
	  		  e=W[0];
	  	  }

	  	  if(czujniki2[3]>treshold)
	  	  {
	  		  error+=W[2];
	  		  i++;
	  	  }

	  	  if(czujniki2[2]>treshold)
	  	  {
	  		  error+=W[3];
	  		  i++;
	  	  }

	  	  if(czujniki2[1]>treshold)
	  	  {
	  		  error+=W[4];
	  		  i++;
	  	  }

	  	  if(czujniki2[0]>treshold)
	  	  {
	  		  error+=W[5];
	  		  i++;
	  	  }

	  	  if(czujniki1[3]>treshold)
	  	  {
	  		  error+=W[6];
	  		  i++;
	  	  }

	  	  if(czujniki1[1]>treshold)
	  	  {
	  		  error+=W[7];
	  		  i++;
	  	  }

	  	  if(czujniki1[0]>treshold)
	  	  {
	  		  error+=W[8];
	  		  i++;
	  		  e=W[9];
	  	  }

	  	  if(i!=0) error=error/i;
	  	  else
	  	  {
	  		  error=e;
	  	  }

	  	  uchyb=error;
	  	  previous_error=error;
	  	  P = uchyb;
	  	  D = uchyb - poprzedni_uchyb;
	  	  PD_value = (Kp*P) + (Kd*D);
	  	  poprzedni_uchyb=uchyb;
	  	  motor_l = motor_initial-PD_value;
	  	  motor_p = motor_initial+PD_value;

	  	  if (motor_l>predkosc) motor_l=predkosc;
	  	  if (motor_l>(dyf*motor_initial+rdyf)) motor_l=dyf*motor_initial+rdyf;
	  	  if (motor_l<0) motor_l=0;
	  	  if (motor_p>predkosc) motor_p=predkosc;
	  	  if (motor_p>(dyf*motor_initial+rdyf)) motor_p=dyf*motor_initial+rdyf;
	  	  if (motor_p<0) motor_p=0;

	  	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);


	 }

	  HAL_Delay(10);
}
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
