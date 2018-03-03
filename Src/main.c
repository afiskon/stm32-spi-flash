/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
/* vim: set ai et ts=4 sw=4: */
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void init() {
    uint16_t pageAddr = 0x123;
    const char wmsg[] = "This is a test message";
    char rmsg[sizeof(wmsg)] = {0};
    HAL_StatusTypeDef res1, res2;

    // read the device id
    {
        uint8_t devid_cmd[1] = { 0x9F };
        uint8_t devid_res[5];

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        res1 = HAL_SPI_Transmit(&hspi1, devid_cmd, sizeof(devid_cmd), HAL_MAX_DELAY);
        res2 = HAL_SPI_Receive(&hspi1, devid_res, sizeof(devid_res), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

        if((res1 != HAL_OK) || (res2 != HAL_OK)) {
            char msg[256];
            snprintf(msg, sizeof(msg), "Error during getting the device id, res1 = %d, res2 = %d\r\n", res1, res2);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }

        {
            char msg[256];
            snprintf(msg, sizeof(msg), 
                "Manufacturer ID: 0x%02X\r\n"
                "Device ID (byte 1): 0x%02X\r\n"
                "Device ID (byte 2): 0x%02X\r\n"
                "Extended device information (EDI) string length: 0x%02X\r\n"
                "EDI byte 1: 0x%02X\r\n"
                "--------\r\n",
                devid_res[0], devid_res[1], devid_res[2], devid_res[3], devid_res[4]);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }

    // write test
    /* if(0) */ {
        uint8_t wcmd[4]; 
        // opcode
        wcmd[0] = 0x82; // 0x82 for buffer 1, 0x85 for buffer 2
        // for 512 bytes/page chip address is transfered in form:
        // 000AAAAA AAAAAAAa aaaaaaaa
        // wcmd[1] = (pageAddr >> 7) & 0x1F;
        // wcmd[2] = (pageAddr << 1) & 0xFE;
        // wcmd[3] = 0x00;

        // 00PPPPPP PPPPPPBB BBBBBBBB
        wcmd[1] = (pageAddr >> 6) & 0x3F;
        wcmd[2] = (pageAddr << 2) & 0xFC;
        wcmd[3] = 0x00;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        res1 = HAL_SPI_Transmit(&hspi1, wcmd, sizeof(wcmd), HAL_MAX_DELAY);
        res2 = HAL_SPI_Transmit(&hspi1, (uint8_t*)wmsg, sizeof(wmsg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

        if((res1 != HAL_OK) || (res2 != HAL_OK)) {
            char msg[256];
            snprintf(msg, sizeof(msg), "Error during writing the data, res1 = %d, res2 = %d\r\n", res1, res2);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }
    }

    // wait until device is ready (using HAL_Delay is error-prone!)
    {
        uint32_t delta = HAL_GetTick();
        uint32_t cnt = 0;

        uint8_t status_cmd[1] = { 0xD7 };
        uint8_t status_res[2];
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, status_cmd, sizeof(status_cmd), HAL_MAX_DELAY);
        do {
            cnt++;
            res1 = HAL_SPI_Receive(&hspi1, status_res, sizeof(status_res), HAL_MAX_DELAY);
            if(res1 != HAL_OK)
                break;
        } while (! (status_res[0] & 0x80)); // check RDY flag

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

        delta = HAL_GetTick() - delta;
        uint8_t protect = (status_res[0] >> 1) & 0x01;
        uint8_t page_size = (status_res[0]) & 0x01;
        uint8_t epe = (status_res[1] >> 5) & 0x01;
        uint8_t sle = (status_res[1] >> 3) & 0x01;
        char msg[256];
        snprintf(msg, sizeof(msg),
            "Await loop took %ld ms, %ld iterations\r\n"
            "Sector protection status: %s\r\n"
            "Page size: %d bytes\r\n"
            "Erase/program error: %s\r\n"
            "Sector lockdown command: %s\r\n"
            "--------\r\n",
            delta, cnt,
            protect ? "enabled" : "disabled",
            page_size ? 512 : 528,
            epe ? "ERROR!" : "no error",
            sle ? "enabled" : "disabled");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    // read test
    {
        uint8_t rcmd[5];
        // opcode
        rcmd[0] = 0x0B; // Note: 0x1B is supported by Adesto chips, but not Atmel chips, so use 0x0B

        // for 512 bytes/page chip address is transfered in form:
        // rcmd[1] = (pageAddr >> 7) & 0x1F;
        // rcmd[2] = (pageAddr << 1) & 0xFE;
        // rcmd[3] = 0x00;

        // 00PPPPPP PPPPPPBB BBBBBBBB
        rcmd[1] = (pageAddr >> 6) & 0x3F;
        rcmd[2] = (pageAddr << 2) & 0xFC;
        rcmd[3] = 0x00;

        // one dummy byte
        rcmd[4] = 0x00;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        res1 = HAL_SPI_Transmit(&hspi1, rcmd, sizeof(rcmd), HAL_MAX_DELAY);
        res2 = HAL_SPI_Receive(&hspi1, (uint8_t*)rmsg, sizeof(rmsg), HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

        if((res1 != HAL_OK) || (res2 != HAL_OK)) {
            char msg[256];
            snprintf(msg, sizeof(msg), "Error during reading the data, res1 = %d, res2 = %d\r\n", res1, res2);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return;
        }
    }

    if(memcmp(rmsg, wmsg, sizeof(rmsg)) == 0) {
        const char result[] = "Test passed!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)result, sizeof(result)-1,
                          HAL_MAX_DELAY);
    } else {
        char msg[256];
        snprintf(msg, sizeof(msg), "Test failed: wmsg = '%s', rmsg = '%s'\r\n", wmsg, rmsg);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

void loop() {
    HAL_Delay(100);
}

/* USER CODE END 0 */

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  init();
  while (1)
  {
    loop();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
