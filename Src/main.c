
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "uart_osc.h"
#include "flash_memory.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t ADC_Value[60000];
uint32_t i;
uint16_t ad1,da1;
uint16_t AD_Value;
uint32_t Address,ADDR_READ;
uint32_t *TEST_ADDR;
GPIO_PinState K_R,K_P;
uint32_t  startAddress , endAddress ;;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		//DMA方式
//		HAL_DMA_Start
//		HAL_DMA_Abort
//		HAL_ADC_Start_DMA
//		HAL_ADC_Stop_DMA
//		HAL_DAC_Start_DMA
//		HAL_DAC_Stop_DMA
		
		
		
		//TEST_LED
		/*HAL_GPIO_WritePin(GPIOA,LED_RECORD_Pin,1);
		HAL_GPIO_WritePin(GPIOA,LED_PLAYBACK_Pin,0);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOA,LED_RECORD_Pin,0);
		HAL_GPIO_WritePin(GPIOA,LED_PLAYBACK_Pin,1);
		HAL_Delay(1000);*/
		
		//TEST
		
//		//TEST_OK
//		//HAL_GPIO_WritePin(GPIOA,LED_RECORD_Pin,HAL_GPIO_ReadPin(GPIOC,KEY_RECORD_Pin));
//		if(HAL_GPIO_ReadPin(GPIOC,KEY_RECORD_Pin)==GPIO_PIN_SET)
//		{
//			HAL_GPIO_WritePin(GPIOA,LED_RECORD_Pin,GPIO_PIN_RESET);
//			printf("RECOEDING \r\n");
//			HAL_Delay(500);
//			startAddress=0x08020000;endAddress=0x0802F000;
//			Address=startAddress;
//			FlashInit();
//			i=0;
//			while(Address<endAddress)
//			{
//			HAL_ADC_Start(&hadc1);
//			ad1=HAL_ADC_GetValue(&hadc1);
//			WriteFlash(Address,ad1);
//			//AD_Value=*(__IO uint16_t*)(Address);
//			//printf("addr:0x%x, data:0x%x\r\n", Address, AD_Value);
//			Address=Address+2;	
//			printf("AD value= %1.3f V  \r\n",ad1*3.3f/4096);
//			//printf("0x%x \r\n",AD_Value);
//				
//				
//				//指针方法
//				/*
//				*TEST_ADDR=ad1;
//					printf("data:0x%x \r\n",*TEST_ADDR);
//				TEST_ADDR++;
//				i++;
//				*/
//			Uart_OSC_ShowWave(ad1,0,0,0);
//		
//			//HAL_Delay(1000);
//			}
//			//PrintFlash();
//			HAL_ADC_Stop(&hadc1);
//			printf("**************************************************************************************************************************");
//			//HAL_Delay(20000);
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOA,LED_RECORD_Pin,GPIO_PIN_SET);
//			//HAL_Delay(1000);
//		}
//		if(HAL_GPIO_ReadPin(GPIOC,KEY_PLAYBACK_Pin)==GPIO_PIN_SET)
//		{
//			HAL_GPIO_WritePin(GPIOA,LED_PLAYBACK_Pin,GPIO_PIN_RESET);
//			printf("PLAYBACKING...");
//			printf("\r\n DAC EXAMPLE !!!\r\n");
//			
//			startAddress=0x08020000;
//			endAddress=0x0802F000;
//			Address=startAddress;
//			//MemoryProgramStatus=0x0;
//			//uint32_t temp ;
//			while (Address < endAddress)
//			{
//					da1=ReadFlash(Address);
//					printf("addr:0x%x, data:0x%x\r\n", Address, da1);
//					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, da1);
//					HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
//					Address = Address + 2;
//					//printf("DATA:0x%x \r\n",DATA_16);
//			}
//			printf("**************************************************************************************************************************");
//		}
//		else
//		{
//			HAL_GPIO_WritePin(GPIOA,LED_PLAYBACK_Pin,GPIO_PIN_SET);
//			HAL_Delay(1000);
//		}
		
		
		
		//CONTROL
		/*if(HAL_GPIO_ReadPin(GPIOC,KEY_RECORD_Pin)==1)
		{
			HAL_GPIO_WritePin(GPIOA,LED_RECORD_Pin,0);
			printf("RECORDING...");
			HAL_Delay(1000);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA,LED_RECORD_Pin,1);
			HAL_Delay(1000);
		}
		if(HAL_GPIO_ReadPin(GPIOC,KEY_PLAYBACK_Pin)==1)
		{
			HAL_GPIO_WritePin(GPIOA,LED_PLAYBACK_Pin,0);
			printf("PLAYBACKING...");
			HAL_Delay(1000);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA,LED_PLAYBACK_Pin,1);
			HAL_Delay(1000);
		}*/
		
		
		//ADC
		/*HAL_Delay(500);
		HAL_ADC_Start(&hadc1);
		ad1=HAL_ADC_GetValue(&hadc1);
		printf("AD value= %1.3f V  \r\n",ad1*3.3f/4096);
		Uart_OSC_ShowWave(ad1,0,0,0);*/
		//MEMORY
		/*WriteFlash(ad1);
		PrintFlash();*/
  }
	
	//DAC
	/*printf("\r\n DAC EXAMPLE !!!\r\n");
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, ad1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);*/
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
		case GPIO_PIN_14:
			printf("START RECORD..................... \r\n");
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_Value,60000);
		//HAL_DAC_Start_DMA(&hdac,DAC_CHANNEL_2,(uint32_t*)&ADC_Value,10000,DAC_ALIGN_8B_R);
		//i=0;
		//HAL_TIM_Base_Start_IT(&htim2);
		
//		i=0;
//		while(i<10000)
//		{
//			Uart_OSC_ShowWave(ADC_Value[i],0,0,0);
//			i++;
//		}
		break;
		case GPIO_PIN_15:
			printf("PLAYBACK ....................... \r\n");
		i=0;
		HAL_TIM_Base_Start_IT(&htim2);
		break;
		case GPIO_PIN_2:
			printf("STOPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP \r\n");
		HAL_ADC_Stop_DMA(&hadc1);
		break;
		
		default:printf("NOTHING");break;
	}
}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//printf("TIM2... \r\n");
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, ADC_Value[i]);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	i++;
	if(i==60000) HAL_TIM_Base_Stop(&htim2);
}

//			startAddress=0x08020000;endAddress=0x0803FFFF;
//		if(Address<endAddress)
//		{
//			Address=startAddress;
//			FlashInit();
//		}
//			
//			//FlashInit();
//			i=0;
//			while(Address<endAddress-1)
//			{
//			HAL_ADC_Start(&hadc1);
//			ad1=HAL_ADC_GetValue(&hadc1);
//			WriteFlash(Address,ad1);
//			Address=Address+2;	
//			ADDR_READ=Address;
//			printf("AD value= %1.3f V  \r\n",ad1*3.3f/4096);
//				//printf("data: 0x%x \r\n",ad1);
//			Uart_OSC_ShowWave(ad1,0,0,0);
//			}
//			
//			//PrintFlash();
//		break;
//		case GPIO_PIN_15:		
//		printf("PLAYBACK..................... \r\n");
//		//printf("ADDR_READ: 0x%x",ADDR_READ);
//			startAddress=0x08020000;
//			endAddress=ADDR_READ;
//			Address=startAddress;
//		while (Address < endAddress)
//			{
//					da1=ReadFlash(Address);
//					printf("addr:0x%x, data:0x%x\r\n", Address, da1);
//					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, da1);
//					HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
//					Address = Address + 2;
//			HAL_Delay(500);
//					//printf("DATA:0x%x \r\n",DATA_16);
//			}
//			printf("*********************************");
//		break;	
//		case GPIO_PIN_2:
////			printf("PLAYBACK..................... \r\n");
////			startAddress=0x08020000;
////			endAddress=ADDR_READ;
////			Address=startAddress;
////		while (Address < endAddress)
////			{
////					da1=ReadFlash(Address);
////					printf("addr:0x%x, data:0x%x\r\n", Address, da1);
////					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, da1);
////					HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
////					Address = Address + 2;
////					//printf("DATA:0x%x \r\n",DATA_16);
////			}
////			printf("*********************************");
////		break;	
//		
//		printf("STOP RECORD..................... \r\n");
//		HAL_ADC_Stop(&hadc1);
//		Address=0x0803FFFF;
//		break;
//		default:printf("NOTHING");break;


  /*if(GPIO_Pin == GPIO_PIN_14)
  {
		//AD_Value=HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14);
		printf("START RECORD..................... \r\n");
		//printf("%d \r\n",AD_Value);
  }  
	if(GPIO_Pin == GPIO_PIN_15)
  {
		printf("STOP RECORD..................... \r\n");
  }  
	*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
