/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "eth.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SPI_DOUBLE_DATA_NUM 		10
#define SPI_FLOAT_DATA_NUM 			20
#define SPI_INT32_DATA_NUM 			10
#define SPI_UINT32_DATA_NUM 		10
#define SPI_INT16_DATA_NUM 			10
#define SPI_UINT16_DATA_NUM 		10
#define SPI_INT8_DATA_NUM 			10
#define SPI_UINT8_DATA_NUM 			10

#define DATA_TOTAL_BYTE 			100		// Should be Larger than NEEDED
#define SPI_TOTAL_DATA_NUM			(SPI_DOUBLE_DATA_NUM + SPI_FLOAT_DATA_NUM + SPI_INT32_DATA_NUM + SPI_UINT32_DATA_NUM + SPI_INT16_DATA_NUM + SPI_UINT16_DATA_NUM + SPI_INT8_DATA_NUM + SPI_UINT8_DATA_NUM)


typedef enum _IOIF_SPIDatatType_t {
	SPI_DOUBLE = 1,
	SPI_FLOAT,
	SPI_INT32,
	SPI_UINT32,
	SPI_INT16,
	SPI_UINT16,
	SPI_INT8,
	SPI_UINT8
} IOIF_SPIDataType_t;

typedef struct _IOIF_SPIObj_t {
	double doubleSPIData[SPI_DOUBLE_DATA_NUM];
	float floatSPIData[SPI_FLOAT_DATA_NUM];
	int32_t int32SPIData[SPI_INT32_DATA_NUM];
	uint32_t uint32SPIData[SPI_UINT32_DATA_NUM];
	int16_t int16SPIData[SPI_INT16_DATA_NUM];
	uint16_t uint16SPIData[SPI_UINT16_DATA_NUM];
	int8_t int8SPIData[SPI_INT8_DATA_NUM];
	uint8_t uint8SPIData[SPI_UINT8_DATA_NUM];

	uint8_t doubleIndex;
	uint8_t floatIndex;
	uint8_t int32Index;
	uint8_t uint32Index;
	uint8_t int16Index;
	uint8_t uint16Index;
	uint8_t int8Index;
	uint8_t uint8Index;

	uint8_t totalDataNum;
	uint16_t totalDataByte;

	uint8_t dataTypeArray[SPI_TOTAL_DATA_NUM];

	uint8_t ParsedSPIByteData[DATA_TOTAL_BYTE];
} IOIF_SPIObj_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* For DMA SPI Communication (with DMA) */
static uint8_t spi1CommDmaRxBuff[DATA_TOTAL_BYTE] __attribute__((section(".spi1RxBuff"))) = {0};
static uint8_t spi2CommDmaTxBuff[DATA_TOTAL_BYTE] __attribute__((section(".spi2TxBuff"))) = {0};

uint8_t rxByteDataArray[DATA_TOTAL_BYTE] = {0};

IOIF_SPIObj_t* spiPtr = NULL;
IOIF_SPIObj_t spiTxObj = {0};


/* Transmission */
uint8_t AppendSPIData(IOIF_SPIDataType_t dataType, void* dataAddr)
{
	spiPtr = &spiTxObj;

	if (dataType == SPI_DOUBLE){
		spiPtr->doubleSPIData[spiPtr->doubleIndex] = *((double*)dataAddr);
		spiPtr->dataTypeArray[spiPtr->totalDataNum] = SPI_DOUBLE;

		spiPtr->doubleIndex++;
		spiPtr->totalDataNum++;
		spiPtr->totalDataByte += sizeof(double);
	}
	else if (dataType == SPI_FLOAT){
		spiPtr->floatSPIData[spiPtr->floatIndex] = *((float*)dataAddr);
		spiPtr->dataTypeArray[spiPtr->totalDataNum] = SPI_FLOAT;

		spiPtr->floatIndex++;
		spiPtr->totalDataNum++;
		spiPtr->totalDataByte += sizeof(float);
	}
	else if (dataType == SPI_INT32){
		spiPtr->int32SPIData[spiPtr->int32Index] = *((int32_t*)dataAddr);
		spiPtr->dataTypeArray[spiPtr->totalDataNum] = SPI_INT32;

		spiPtr->int32Index++;
		spiPtr->totalDataNum++;
		spiPtr->totalDataByte += sizeof(int32_t);
	}
	else if (dataType == SPI_UINT32){
		spiPtr->uint32SPIData[spiPtr->uint32Index] = *((uint32_t*)dataAddr);
		spiPtr->dataTypeArray[spiPtr->totalDataNum] = SPI_UINT32;

		spiPtr->uint32Index++;
		spiPtr->totalDataNum++;
		spiPtr->totalDataByte += sizeof(uint32_t);
	}
	else if (dataType == SPI_INT16){
		spiPtr->int16SPIData[spiPtr->int16Index] = *((int16_t*)dataAddr);
		spiPtr->dataTypeArray[spiPtr->totalDataNum] = SPI_INT16;

		spiPtr->int16Index++;
		spiPtr->totalDataNum++;
		spiPtr->totalDataByte += sizeof(int16_t);
	}
	else if (dataType == SPI_UINT16){
		spiPtr->uint16SPIData[spiPtr->uint16Index] = *((uint16_t*)dataAddr);
		spiPtr->dataTypeArray[spiPtr->totalDataNum] = SPI_UINT16;

		spiPtr->uint16Index++;
		spiPtr->totalDataNum++;
		spiPtr->totalDataByte += sizeof(uint16_t);
	}
	else if (dataType == SPI_INT8){
		spiPtr->int8SPIData[spiPtr->int8Index] = *((int8_t*)dataAddr);
		spiPtr->dataTypeArray[spiPtr->totalDataNum] = SPI_INT8;

		spiPtr->int8Index++;
		spiPtr->totalDataNum++;
		spiPtr->totalDataByte += sizeof(int8_t);
	}
	else if (dataType == SPI_UINT8){
		spiPtr->uint8SPIData[spiPtr->uint8Index] = *((uint8_t*)dataAddr);
		spiPtr->dataTypeArray[spiPtr->totalDataNum] = SPI_UINT8;

		spiPtr->uint8Index++;
		spiPtr->totalDataNum++;
		spiPtr->totalDataByte += sizeof(uint8_t);
	}
	else {
		return 1;
		// Error Check
	}

	return 0;
}


uint8_t ParseSPIData(void)
{
	spiPtr = &spiTxObj;

	static uint8_t doubleInd = 0;
	static uint8_t floatInd = 0;
	static uint8_t int32Ind = 0;
	static uint8_t uint32Ind = 0;
	static uint8_t int16Ind = 0;
	static uint8_t uint16Ind = 0;
	static uint8_t int8Ind = 0;
	static uint8_t uint8Ind = 0;

	static uint16_t ind = 0;

	/* 1st data is "totalDataNum" */
	memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->totalDataNum, 1);
	ind++;

	/* Remained data has the form of "dataType, parsedData" */
	for (uint8_t i = 0; i < spiPtr->totalDataNum; i++){
		if (spiPtr->dataTypeArray[i] == SPI_DOUBLE){
			memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->dataTypeArray[i], 1);
			memcpy(&spiPtr->ParsedSPIByteData[ind + 1], &spiPtr->doubleSPIData[doubleInd], sizeof(double));
			doubleInd++;
			ind = ind + sizeof(double) + 1;
		}
		else if (spiPtr->dataTypeArray[i] == SPI_FLOAT){
			memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->dataTypeArray[i], 1);
			memcpy(&spiPtr->ParsedSPIByteData[ind + 1], &spiPtr->floatSPIData[floatInd], sizeof(float));
			floatInd++;
			ind = ind + sizeof(float) + 1;
		}
		else if (spiPtr->dataTypeArray[i] == SPI_INT32){
			memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->dataTypeArray[i], 1);
			memcpy(&spiPtr->ParsedSPIByteData[ind + 1], &spiPtr->floatSPIData[int32Ind], sizeof(int32_t));
			int32Ind++;
			ind = ind + sizeof(int32_t) + 1;
		}
		else if (spiPtr->dataTypeArray[i] == SPI_UINT32){
			memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->dataTypeArray[i], 1);
			memcpy(&spiPtr->ParsedSPIByteData[ind + 1], &spiPtr->uint32SPIData[uint32Ind], sizeof(uint32_t));
			uint32Ind++;
			ind = ind + sizeof(uint32_t) + 1;
		}
		else if (spiPtr->dataTypeArray[i] == SPI_INT16){
			memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->dataTypeArray[i], 1);
			memcpy(&spiPtr->ParsedSPIByteData[ind + 1], &spiPtr->int16SPIData[int16Ind], sizeof(int16_t));
			int16Ind++;
			ind = ind + sizeof(int16_t) + 1;
		}
		else if (spiPtr->dataTypeArray[i] == SPI_UINT16){
			memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->dataTypeArray[i], 1);
			memcpy(&spiPtr->ParsedSPIByteData[ind + 1], &spiPtr->uint16SPIData[uint16Ind], sizeof(uint16_t));
			uint16Ind++;
			ind = ind + sizeof(uint16_t) + 1;
		}
		else if (spiPtr->dataTypeArray[i] == SPI_INT8){
			memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->dataTypeArray[i], 1);
			memcpy(&spiPtr->ParsedSPIByteData[ind + 1], &spiPtr->int8SPIData[int8Ind], sizeof(int8_t));
			int8Ind++;
			ind = ind + sizeof(int8_t) + 1;
		}
		else if (spiPtr->dataTypeArray[i] == SPI_UINT8){
			memcpy(&spiPtr->ParsedSPIByteData[ind], &spiPtr->dataTypeArray[i], 1);
			memcpy(&spiPtr->ParsedSPIByteData[ind + 1], &spiPtr->uint8SPIData[uint8Ind], sizeof(uint8_t));
			uint8Ind++;
			ind = ind + sizeof(uint8_t) + 1;
		}
	}

	doubleInd = 0;
	floatInd = 0;
	int32Ind = 0;
	uint32Ind = 0;
	int16Ind = 0;
	uint16Ind = 0;
	int8Ind = 0;
	uint8Ind = 0;

	ind = 0;

	return 0;
}



/* Receive */
void byteValueToRealValue(void* realData, uint8_t* byteData)
{
	static uint16_t rxDataIndex = 0;
	static uint8_t rxDataNum = 0;
	static uint8_t firstRead = 0;
	static uint8_t totalDataNum = 0;

	/* Check Total Data Number */
	if (firstRead == 0){
		totalDataNum = byteData[rxDataIndex];
		rxDataIndex++;
		firstRead = 1;
	}

	/* Parsing */
	if (byteData[rxDataIndex] == SPI_DOUBLE){
		memcpy((double*)realData, &byteData[rxDataIndex+1], sizeof(double));
		rxDataIndex += 9;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == SPI_FLOAT){
		memcpy((float*)realData, &byteData[rxDataIndex+1], sizeof(float));
		rxDataIndex += 5;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == SPI_INT32){
		memcpy((int32_t*)realData, &byteData[rxDataIndex+1], sizeof(int32_t));
		rxDataIndex += 5;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == SPI_UINT32){
		memcpy((uint32_t*)realData, &byteData[rxDataIndex+1], sizeof(uint32_t));
		rxDataIndex += 5;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == SPI_INT16){
		memcpy((int16_t*)realData, &byteData[rxDataIndex+1], sizeof(int16_t));
		rxDataIndex += 3;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == SPI_UINT16){
		memcpy((uint16_t*)realData, &byteData[rxDataIndex+1], sizeof(uint16_t));
		rxDataIndex += 3;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == SPI_INT8){
		memcpy((int8_t*)realData, &byteData[rxDataIndex+1], sizeof(int8_t));
		rxDataIndex += 2;
		rxDataNum++;
	}
	else if (byteData[rxDataIndex] == SPI_UINT8){
		memcpy((uint8_t*)realData, &byteData[rxDataIndex+1], sizeof(uint8_t));
		rxDataIndex += 2;
		rxDataNum++;
	}
	else {
		rxDataIndex = 0;
		rxDataNum = 0;
		firstRead = 0;
		return;
	}

	/* Optimization case */
	if (rxDataIndex == DATA_TOTAL_BYTE){
		rxDataIndex = 0;
		rxDataNum = 0;
		firstRead = 0;
		return;
	}

	if (rxDataNum == totalDataNum){
		rxDataIndex = 0;
		rxDataNum = 0;
		firstRead = 0;
		return;
	}
}


/* For Transmitted Values */
float txData1 = 3.14;
float txData2 = 4.14;
float txData3 = 5.14;
float txData4 = 6.14;
float txData5 = 7.14;
float txData6 = 8.14;


/* For Received Values */
float data1;
float data2;
float data3;
float data4;
float data5;
float data6;
float data7;
float data8;
float data9;
float data10;
float data11;
float data12;
float data13;



void settingSPIReceivedData(uint8_t* byteData)
{
	byteValueToRealValue(&data1, byteData);
	byteValueToRealValue(&data2, byteData);
	byteValueToRealValue(&data3, byteData);
	byteValueToRealValue(&data4, byteData);
	byteValueToRealValue(&data5, byteData);
	byteValueToRealValue(&data6, byteData);
//	byteValueToRealValue(&data7, byteData);
//	byteValueToRealValue(&data8, byteData);
//	byteValueToRealValue(&data9, byteData);
//	byteValueToRealValue(&data10, byteData);
//	byteValueToRealValue(&data11, byteData);
//	byteValueToRealValue(&data12, byteData);
//	byteValueToRealValue(&data13, byteData);
}



/* Transmit */
uint8_t IOIF_TransmitSPIData(void)
{
	spiPtr = &spiTxObj;

	memset(spi2CommDmaTxBuff, 0, DATA_TOTAL_BYTE);
	memcpy(spi2CommDmaTxBuff, spiPtr->ParsedSPIByteData, DATA_TOTAL_BYTE);
	uint8_t txDebug = HAL_SPI_Transmit_DMA(&hspi2, spi2CommDmaTxBuff, DATA_TOTAL_BYTE);

	spiTxObj = (IOIF_SPIObj_t){0};

	return txDebug;
}

/* Receive */
uint8_t IOIF_ReceiveSPIData(void)
{
	memset(rxByteDataArray, 0, DATA_TOTAL_BYTE);
	uint8_t rxDebug = HAL_SPI_Receive_DMA(&hspi1, spi1CommDmaRxBuff, DATA_TOTAL_BYTE);
	memcpy(rxByteDataArray, spi1CommDmaRxBuff, DATA_TOTAL_BYTE);
	settingSPIReceivedData(rxByteDataArray);

	return rxDebug;
}

uint32_t cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* TIM1 : SPI1(Master) - Receive */
	if (htim == &htim1){
		IOIF_ReceiveSPIData();
	}

	/* TIM2 : SPI2(Slave) - Transmit */
	if (htim == &htim2){
		/* Append data to transmit */
		AppendSPIData(SPI_FLOAT, &txData1);
		AppendSPIData(SPI_FLOAT, &txData2);
		AppendSPIData(SPI_FLOAT, &txData3);
		AppendSPIData(SPI_FLOAT, &txData4);
		AppendSPIData(SPI_FLOAT, &txData5);
		AppendSPIData(SPI_FLOAT, &txData6);

		/* Parse data to byte */
		ParseSPIData();

		/* Transmit with SPI */
		IOIF_TransmitSPIData();

		/* Test Data Update */
		cnt++;
		if (cnt == 1000){
			cnt = 0;
			txData1++;
			txData2++;
			txData3++;
			txData4++;
			txData5++;
			txData6++;
		}
	}

}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x00000000;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
