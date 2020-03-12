/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "brailleMatrix.h"
#include "DCMotors.h"
#include "stm32f1xx_it.h"
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

/* USER CODE BEGIN PV */
#define MAX_CARACTERES 30
#define MAX_LINHAS 27
#define POS_INI 28000
#define POS_FIM 4000

char pressedEnter = 1;
char isEnd = 0;

MotorControl_t motorX;
MotorControl_t motorY;

MotorControl_Simple_t motorZ;

unsigned char buffer_char[MAX_CARACTERES];
unsigned char buffer_braille[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  motorBegin(&motorX, &htim2, MOTOR_X_A_GPIO_Port, MOTOR_X_A_Pin, MOTOR_X_B_GPIO_Port, MOTOR_X_B_Pin);
  motorBegin(&motorY, &htim1, MOTOR_Y_A_GPIO_Port, MOTOR_Y_A_Pin, MOTOR_Y_B_GPIO_Port, MOTOR_Y_B_Pin);

  //motorSimpleBegin(&motorZ, Motor_Z_A_GPIO_Port, Motor_Z_A_Pin, Motor_Z_B_GPIO_Port, Motor_Z_B_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Faz a leitura do teclado e envia ao buffer_char
	  		while(1){
	  			if(keyboardAvailable(&keyboard)){
	  				HAL_GPIO_WritePin(OUTPUT_LED_GPIO_Port, OUTPUT_LED_Pin, GPIO_PIN_SET);
	  				HAL_Delay(100);
	  				HAL_GPIO_WritePin(OUTPUT_LED_GPIO_Port, OUTPUT_LED_Pin, GPIO_PIN_RESET);
	  				uint8_t c = keyboardRead(&keyboard);
	  				if(c == ENTER){
	  					pressedEnter = 1;
	  					HAL_GPIO_WritePin(OUTPUT_LED_GPIO_Port, OUTPUT_LED_Pin, GPIO_PIN_SET);
	  					break;
	  				}else if(c == DOWNARROW){
	  					updateAxis(&motorY, motorY.setPoint - 50);
	  				}else if(c == UPARROW){
	  					updateAxis(&motorY, motorY.setPoint + 50);
	  				}else if(c == BACKSPACE){
	  					clearBuffer(buffer_char);
	  				}else{



	  					feedBuffer(buffer_char, MAX_CARACTERES, c);
	  				}
	  			}
	  		}
	  		//Programa leitura do teclado
	  		if(pressedEnter){
	  			//reverse(buffer_char);
	  			uint32_t length = strlen((const char*)buffer_char);

	  			for(int j=0;j<3;j++){

	  				//Imprime as linhas em braille
	  				if(j==1){
	  					//Linhas das matrizes
	  					for(int8_t i=length-1; i>=0; --i){

	  						// Recebe os pontos da linha para o caractere atual
	  						fillLineWithBraille(buffer_braille, j,buffer_char[i]);

	  						// Percorre os 4 bits, no máximo, para cada caractere
	  						for(int8_t x = 0; x < 2; ++x){

	  							// Passa para a próxima iteração se não tem ponto para furar
	  							if(buffer_braille[x] == '1'){
	  								pierce(&motorZ, PIERCE_TIME);
	  							}

	  							if(x != 1){
	  								updateAxis(&motorX, motorX.setPoint - DELTA_COL_LIN);
	  							}

	  							// Incrementa posição do eixo x, espaçamento entre colunas

	  						}

	  						// Incrementa posição do eixo x, espaçamento entre char na horizontal
	  						if(i != 0){
	  							updateAxis(&motorX, motorX.setPoint - DELTA_CHAR_H);
	  						}


	  					}

	  					// Decrementa posição do eixo y, espaçamento entre linhas
	  					updateAxis(&motorY, motorY.setPoint - DELTA_COL_LIN);
	  					//updateAxis_Simple(&motorY, NEXT_LINE, UP);
	  				}else{

	  					//Linhas das matrizes
	  					for(int8_t i=0; i<length; i++){

	  						//Recebe os pontos da linha para o caractere atual
	  						fillLineWithBraille(buffer_braille, j,buffer_char[i]);

	  						//Percorre os 4 bits, no máximo, para cada caractere
	  						for(int8_t x = 1; x >= 0; --x){

	  							if(buffer_braille[x] == '1'){
	  								pierce(&motorZ, PIERCE_TIME);
	  							}

	  							// Incrementa posição do eixo x, espaçamento entre colunas
	  							if(x != 0){
	  								updateAxis(&motorX, motorX.setPoint + DELTA_COL_LIN);
	  							}


	  						}

	  						// Incrementa posição do eixo x, espaçamento entre char na horizontal
	  						if(i != (length -1)){
	  							updateAxis(&motorX, motorX.setPoint + DELTA_CHAR_H);
	  						}


	  					}

	  					// Incrementa posição do eixo y, espaçamento entre linhas
	  					updateAxis(&motorY, motorY.setPoint + DELTA_COL_LIN);
	  					//updateAxis_Simple(&motorY, NEXT_LINE, UP);
	  				}
	  			}

	  			//Incrementa posição do eixo y, espaçamento entre char na vertical
	  			updateAxis(&motorY, motorY.setPoint + DELTA_CHAR_V - DELTA_COL_LIN);
	  			//updateAxis_Simple(&motorY, NEXT_CHAR, UP);
	  			// Seta posição eixo x para inicial
	  			updateAxis(&motorX, POS_INI);

	  			pressedEnter = 0;
	  		}
	  		HAL_GPIO_WritePin(OUTPUT_LED_GPIO_Port, OUTPUT_LED_Pin, GPIO_PIN_RESET);
	  		keyboardClear(&keyboard);
	  		clearBuffer(buffer_char);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
