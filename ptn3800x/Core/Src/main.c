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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ptn3800x.h"
#define MUX
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
ptn3800x_HandlerTypedef ptn38007;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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


int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 10);
  return ch;
}


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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
//  setvbuf(stdout, NULL, _IONBF, 0);



  /* USER CODE BEGIN 2 */


  uint8_t data;

  HAL_Delay(1000);

  printf("Inicializando Placa \r\n");
  if(!ptn3800x_init(&ptn38007, &hi2c1))
	  printf("Erro Na inicizalizacao do PTN \r\n\n");
  printf("Fim de Inicializacao \r\n\n");

  HAL_Delay(1);

  printf("Config Orientacao\r\n");
  ptn3800x_dev_ctrl_reset(&ptn38007);

  ptn3800x_orientation_config(&ptn38007, REVERSE);
  printf("Orientation %d\r\n", ptn38007.orient_handler.plug_orientation_ctrl);

  ptn3800x_read_data(&ptn38007, MODE_CTRL, &data);
  printf("REG[0x%02x] >> [0x%02x]\r\n", MODE_CTRL, data);



  HAL_Delay(1);

  ptn3800x_operation_mode(&ptn38007, MD_SAFE_STATE);
  ptn3800x_read_data(&ptn38007, MODE_CTRL, &data);
  printf("REG[0x%02x] >> [0x%02x]\r\n", MODE_CTRL, data);


  ptn3800x_orientation_config(&ptn38007, NORMAL);
  ptn3800x_read_data(&ptn38007, MODE_CTRL, &data);
  printf("REG[0x%02x] >> [0x%02x]\r\n", MODE_CTRL, data);


  ptn3800x_AUX_pin_polarity(&ptn38007,NORMAL);
  ptn3800x_read_data(&ptn38007, MODE_CTRL, &data);
  printf("AUX PIn POlarity is %d\r\n", ptn38007.orient_handler.aux_snooping_polarity_bit);
  printf("REG[0x%02x] >> [0x%02x]\r\n", MODE_CTRL, data);

  ptn3800x_AUX_pin_polarity(&ptn38007,REVERSE);
  ptn3800x_read_data(&ptn38007, MODE_CTRL, &data);
  printf("AUX PIn POlarity is %d\r\n", ptn38007.orient_handler.aux_snooping_polarity_bit);
  printf("REG[0x%02x] >> [0x%02x]\r\n", MODE_CTRL, data);

  ptn3800x_AUX_pin_polarity(&ptn38007,NORMAL);


  ptn3800x_operation_mode(&ptn38007, MD_SINGLE_USB3);
  ptn3800x_read_data(&ptn38007, MODE_CTRL, &data);
//  printf("AUX PIn POlarity is %d\r\n", ptn38007.orient_handler.aux_snooping_polarity_bit);
  printf("REG[0x%02x] >> [0x%02x]\r\n", MODE_CTRL, data);

  dev_ctrl_t dev_ctrl_values = {
		  .LT_bypass_control = 1,
		  .programeble_values_disconect_tbt_usb4 = TIME_1ms,
		  .aux_sb_snooping_mux = 1
  };

  ptn3800x_dev_ctrl_set(&ptn38007, &dev_ctrl_values);
  ptn3800x_read_data(&ptn38007, DEV_CTRL, &data);
  printf("REG[0x%02x] >> [0x%02x]\r\n", MODE_CTRL, data);

  uint8_t buffer;
  buffer = 0x00;
  ptn3800x_write_data(&ptn38007, DEV_CTRL, &buffer);

  ptn3800x_dev_ctrl_get(&ptn38007, &dev_ctrl_values);
  printf("bit[5] %d | bit[4:2] %d | bit[1] %d\r\n", dev_ctrl_values.LT_bypass_control,
		  	  	  	  	  	  	  	  	  	  dev_ctrl_values.programeble_values_disconect_tbt_usb4,
											  dev_ctrl_values.aux_sb_snooping_mux);










  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
