/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAW_I2C_BUFFER_SIZE 7000 // size in bytes (1 byte = 1 received I2C bit or START/STOP condition)
#define USB_TX_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define USB_TX(A) strcpy(szUSBTxBuf, A); CDC_Transmit_FS((uint8_t*)szUSBTxBuf, (uint16_t) strlen(szUSBTxBuf)); HAL_Delay(1);
#define USB_TX_V(A,B) sprintf(szUSBTxBuf, A, B); CDC_Transmit_FS((uint8_t*)szUSBTxBuf, (uint16_t) strlen(szUSBTxBuf)); HAL_Delay(1);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char szUSBTxBuf[USB_TX_BUFFER_SIZE];
uint8_t rawI2CBuf[RAW_I2C_BUFFER_SIZE];
uint16_t rawI2CBufIdx = 0;   // the current writing position inside the buffer
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void processAndTxData();
void setLED(uint8_t bOn);
/* Private function prototypes -----------------------------------------------*/
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */


  for(int i=0;i<3;i++)
  {
	  HAL_Delay(250); setLED(1);
	  HAL_Delay(250); setLED(0);
  }

  USB_TX("I2C Monitor Started\r\n")

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(CDC_GetRxBufferBytesAvailable_FS())
	  {
			uint8_t rxBuf[10];
			uint8_t ret = CDC_ReadRxBuffer_FS(rxBuf, 1);

			if(ret==USB_CDC_RX_BUFFER_OK)
			{
				setLED(1);
				switch(rxBuf[0])
				{
					case '?':
						USB_TX("Commands: H=Hello, F=Fetch, C=Clear\r\n")
						break;
					case 'h':
					case 'H':
						USB_TX("HelloWorld\r\n")
						for(int i=0;i<3;i++)
						  {
							  HAL_Delay(250); setLED(1);
							  HAL_Delay(250); setLED(0);
						  }
						break;
					case 'f':
					case 'F':
						USB_TX("\r\nTransfer Begin\r\n")
						processAndTxData();
						rawI2CBufIdx = 0;
						USB_TX("\r\nTransfer End\r\n")
						break;
					case 'c':
					case 'C':
						rawI2CBufIdx = 0;
						break;
				}
				setLED(0);
			}

			CDC_FlushRxBuffer_FS();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

enum StateType
{
	stStart=0,
	stRptStart,
	stAddress,
	stRW,
	stAckNack,
	stData,
	stStop,
	stError,
};

void processAndTxData()
{
	uint8_t curDatum;
	uint8_t curBufIdx=0;
	enum StateType nState = stStart;
	uint8_t nDataToReadLeft = 0;
	uint8_t bAbort = 0;
	uint8_t bTempByte;
	uint8_t bShift;

	while ((curBufIdx <= rawI2CBufIdx) && !bAbort)
	{
		setLED(curBufIdx&0x1); //Toggle LED

		curDatum = rawI2CBuf[curBufIdx];

		bShift = 1;

		switch(nState)
		{
		case stStart:
		case stRptStart:
			if(curDatum=='B')
			{
				if(nState==stStart)
				{
					USB_TX("s ")
				}
				else // stRptStart
				{
					USB_TX("z ")
				}

				bTempByte = 0x0;
				nDataToReadLeft = 7;
				nState = stAddress;
			}
			break;
		case stAddress:
			if(curDatum=='H')
			{
				bTempByte |= 0x1<<(nDataToReadLeft-1);
			}
			else if(curDatum=='L')
			{
				// Do nothing
			}
			else
			{
				bShift = 0;
				nState = stError;
				break;
			}

			nDataToReadLeft -= 1;

			if(nDataToReadLeft==0)
			{
				USB_TX_V("0x%02X", bTempByte)
				nState = stRW;
			}
			break;
		case stRW:
			if(curDatum=='H')
			{
				USB_TX("r ")
			}
			else if(curDatum=='L')
			{
				USB_TX("w ")
			}
			else
			{
				bShift = 0;
				nState = stError;
				break;
			}

			nState = stAckNack;
			break;
		case stAckNack:
			if(curDatum=='H')
			{
				USB_TX("n ")
				nState = stStop;
			}
			else if(curDatum=='L')
			{
				USB_TX("a ")

				bTempByte = 0x0;
				nDataToReadLeft = 8;
				nState = stData;
			}
			else
			{
				bShift = 0;
				nState = stError;
				break;
			}
			break;
		case stData:
			if(curDatum=='H')
			{
				bTempByte |= 0x1<<(nDataToReadLeft-1);
			}
			else if(curDatum=='L')
			{
				// Do nothing
			}
			else if(curDatum=='B')
			{
				bShift = 0;
				nState = stRptStart;
			}
			else if(curDatum=='E')
			{
				bShift = 0;
				nState = stStop;
			}
			else
			{
				bShift = 0;
				nState = stError;
				break;
			}

			nDataToReadLeft -= 1;

			if(nDataToReadLeft==0)
			{
				USB_TX_V("0x%02X ", bTempByte)
				nState = stAckNack;
			}
			break;
		case stStop:
			if(curDatum=='E')
			{
				USB_TX("p\r\n")

				nState = stStart;
			}
			else
			{
				// Do nothing until stop received
			}
			break;
		case stError:
			USB_TX_V("X:%c\r\n", curDatum)
			nState = stStart;
			break;
		}

		if(bShift)
		{
			curBufIdx += 1; // Shift Index
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (rawI2CBufIdx < RAW_I2C_BUFFER_SIZE)
	{
		if(GPIO_Pin==SDA_SENS_Pin)
		{
			if(HAL_GPIO_ReadPin(SCL_SENS_GPIO_Port, SCL_SENS_Pin)) // Start & Stop can only happen during SCL is HIGH
			{
				rawI2CBuf[rawI2CBufIdx] = HAL_GPIO_ReadPin(SDA_SENS_GPIO_Port, SDA_SENS_Pin)?'E':'B'; // B=Start, E=Stop
			}
			else
			{
				return;
			}
		}
		else if(GPIO_Pin==SCL_SENS_Pin)
		{
			rawI2CBuf[rawI2CBufIdx] = HAL_GPIO_ReadPin(SDA_SENS_GPIO_Port, SDA_SENS_Pin)?'H':'L'; // H=High, L=Low
		}
		else
		{
			return;
		}

		rawI2CBufIdx += 1;
	}
}

void setLED(uint8_t bOn)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, bOn? GPIO_PIN_RESET:GPIO_PIN_SET);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
