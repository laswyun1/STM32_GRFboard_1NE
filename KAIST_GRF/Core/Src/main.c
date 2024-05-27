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
#include "adc.h"
#include "dcache.h"
#include "fdcan.h"
#include "flash.h"
#include "gpdma.h"
#include "i2c.h"
#include "icache.h"
#include "memorymap.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gait_ctrl.h"
#include "grf_ctrl.h"
#include "msg_hdlr.h"
#include "module.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define READ_FLASH_ARRAY_SIZE  32 	// 128 Byte Aligned
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void SaveProperties();
void DownloadProperties();
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_DCACHE1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_ICACHE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_FLASH_Init();
  /* USER CODE BEGIN 2 */

  DOP_CreateSDOTable();
  DOP_CreatePDOTable();

  // Init Indicator - BLUE LED //
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

  // Bring the Flash Memory //
  DownloadProperties();

  MS_enum = IDLE;

  InitGrfCtrl();
//  InitGaitCtrl();
  InitMsgHdlr();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    if(MS_enum == IDLE)                         continue;
//	    else if(MS_enum == UPLOAD_PROPERTIES)       UploadProperties();
	    else if(MS_enum == SAVE_PROPERTIES)         SaveProperties();
	    else if(MS_enum == DOWNLOAD_PROPERTIES)     DownloadProperties();
//	    else if(MS_enum == ELECTRICAL_SYSTEM_ID)    Cal_Elec_System_ID_Batch();
//	    else if(MS_enum == BEMF_ID)                 Send_Elec_BEMF_Value();
//	    else if(MS_enum == CURRENT_BANDWIDTH_CHECK) Send_Elec_Bandwidth_Data();
//	    else if(MS_enum == AUTO_TUNING)             Tune_Gain();
//	    else if(MS_enum == ADV_FRICTION_ID)         Send_Advanced_Friction_ID_Data();
//	    else if(MS_enum == CAL_FRICTION_LUT)        Cal_Friction_LUT();
	    else                                        continue;
	    MS_enum = IDLE;
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV2;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_0;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void SaveProperties()
{
	uint32_t writeAddr = 0;
	int32_t memArr1[4] = {0};
	int32_t memArr2[4] = {0};
	int32_t memArr3[4] = {0};

	IOIF_EraseFlash(IOIF_FLASH_START_USER_ADDR, IOIF_ERASE_ONE_SECTOR);
	writeAddr = IOIF_FLASH_START_USER_ADDR;

	/* Set the offset */
#ifdef GRF_LEFT
	GrfDataObj.rawS2offset[0] = 200;
	GrfDataObj.rawS2offset[1] = -20;
	GrfDataObj.rawS2offset[2] = 40;

	GrfDataObj.rawS3offset[0] = 50;
	GrfDataObj.rawS3offset[1] = 80;
	GrfDataObj.rawS3offset[2] = 1230;

	GrfDataObj.rawS4offset[0] = 1234;
	GrfDataObj.rawS4offset[1] = -2392;
	GrfDataObj.rawS4offset[2] = -4321;
#endif
#ifdef GRF_RIGHT
	GrfDataObj.rawS2offset[0] = 0;
	GrfDataObj.rawS2offset[1] = 0;
	GrfDataObj.rawS2offset[2] = 0;

	GrfDataObj.rawS3offset[0] = 0;
	GrfDataObj.rawS3offset[1] = 0;
	GrfDataObj.rawS3offset[2] = 0;

	GrfDataObj.rawS3offset[0] = 0;
	GrfDataObj.rawS3offset[1] = 0;
	GrfDataObj.rawS3offset[2] = 0;
#endif

	/* Save Flash memory  */
	memcpy(&memArr1[0], &GrfDataObj.rawS2offset[0],     	   sizeof(GrfDataObj.rawS2offset[0]));
	memcpy(&memArr1[1], &GrfDataObj.rawS2offset[1],     	   sizeof(GrfDataObj.rawS2offset[1]));
	memcpy(&memArr1[2], &GrfDataObj.rawS2offset[2],     	   sizeof(GrfDataObj.rawS2offset[2]));
	memcpy(&memArr1[3], &GrfDataObj.rawS3offset[0],     	   sizeof(GrfDataObj.rawS3offset[0]));

	IOIF_WriteFlash(writeAddr, memArr1);
	writeAddr += 16;


	memcpy(&memArr2[0], &GrfDataObj.rawS3offset[1],     	   sizeof(GrfDataObj.rawS3offset[1]));
	memcpy(&memArr2[1], &GrfDataObj.rawS3offset[2],     	   sizeof(GrfDataObj.rawS3offset[2]));
	memcpy(&memArr2[2], &GrfDataObj.rawS4offset[0],     	   sizeof(GrfDataObj.rawS4offset[0]));
	memcpy(&memArr2[3], &GrfDataObj.rawS4offset[1],     	   sizeof(GrfDataObj.rawS4offset[1]));

	IOIF_WriteFlash(writeAddr, memArr2);
	writeAddr += 16;


	memcpy(&memArr3[0], &GrfDataObj.rawS4offset[2],     	   sizeof(GrfDataObj.rawS2offset[0]));

	IOIF_WriteFlash(writeAddr, memArr3);
	writeAddr += 16;


	return;
}


void DownloadProperties()
{
  uint32_t readAddr = IOIF_FLASH_START_USER_ADDR;

  /* Download Flash Memory */
  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS2offset[0], 	  IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS2offset[1],    IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS2offset[2], 	  IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS3offset[0], 	  IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS3offset[1],    IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS3offset[2], 	  IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS4offset[0], 	  IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS4offset[1],    IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;

  for(int i = 0; i < 500; ++i) {}

  IOIF_ReadFlash(readAddr, &GrfDataObj.rawS4offset[2],    IOIF_FLASH_READ_SIZE_4B);  readAddr += IOIF_FLASH_READ_ADDR_SIZE_4B;
}


/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x28000000;
  MPU_InitStruct.LimitAddress = 0x28002fff;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER1;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RW;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  MPU_AttributesInit.Number = MPU_REGION_NUMBER0;
  MPU_AttributesInit.Attributes = MPU_DEVICE_nGnRnE | MPU_NOT_CACHEABLE
                              | MPU_TRANSIENT | MPU_NO_ALLOCATE;

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
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
