/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
volatile static uint16_t Adc_Data[12]={0};

struct{
	volatile float FSR1_NormalPressure;										//FSR1法向压力数据		对应ADC10
	volatile float FSR2_NormalPressure;										//FSR2法向压力数据		对应ADC11
	volatile float FSR3_NormalPressure;										//FSR3法向压力数据		对应ADC0
	volatile float FSR4_NormalPressure;										//FSR4法向压力数据		对应ADC1
	volatile float FSR5_NormalPressure;										//FSR5法向压力数据		对应ADC2
	volatile float Thumb_Bending;												  //拇指弯曲度数据			对应ADC3
	volatile float IndexFinger_Bending;									  //食指弯曲度数据			对应ADC4
	volatile float MiddleFinger_Bending;								  //中指弯曲度数据			对应ADC5
	volatile float RingFinger_Bending;									  //无名指弯曲度数据		对应ADC6
	volatile float LittleFinger_Bending;								  //小指弯曲度数据			对应ADC7
	volatile float ADC_Backup1;								  					//备用ADC数据					对应ADC8
	volatile float ADC_Backup2;								  					//备用ADC数据					对应ADC9
}ProstheticHand_SensorData;//假肢手外接传感器数据

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;


osThreadId ADC_CollectionAndProcessTaskHandle;

void ADC_CollectionAndProcessTask(void const * argument);

static void ADC_FREERTOS_Init(void) 
{
	/* definition and creation of ADC_CollectionAndProcessTask */
	osThreadDef(ADC_CollectionAndProcessTask, ADC_CollectionAndProcessTask, osPriorityNormal, 0, 128);
	ADC_CollectionAndProcessTaskHandle = osThreadCreate(osThread(ADC_CollectionAndProcessTask), NULL);
}


/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 12;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	ADC_FREERTOS_Init();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    GPIO_InitStruct.Pin = FSR_AD1_Pin|FSR_AD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = FSR_AD3_Pin|FSR_AD4_Pin|FSR_AD5_Pin|Bending_AD1_Pin 
                          |Bending_AD2_Pin|Bending_AD3_Pin|Bending_AD4_Pin|Bending_AD5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Backup_AD1_Pin|Backup_AD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PC0     ------> ADC1_IN10
    PC1     ------> ADC1_IN11
    PA0-WKUP     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA4     ------> ADC1_IN4
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    PA7     ------> ADC1_IN7
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    HAL_GPIO_DeInit(GPIOC, FSR_AD1_Pin|FSR_AD2_Pin);

    HAL_GPIO_DeInit(GPIOA, FSR_AD3_Pin|FSR_AD4_Pin|FSR_AD5_Pin|Bending_AD1_Pin 
                          |Bending_AD2_Pin|Bending_AD3_Pin|Bending_AD4_Pin|Bending_AD5_Pin);

    HAL_GPIO_DeInit(GPIOB, Backup_AD1_Pin|Backup_AD2_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/*
*****************************************************************************
*@brief		FSR数据处理函数，插值计算公式在Controller中写
*@param		uint16_t *adc_data：存放ADC数据的数组
*@param		uint8_t FSR_index：待处理的FSR数据编号
*@retval	None
*@par
*****************************************************************************
*/
static void FSR_DataProcess(uint16_t *adc_data,uint8_t FSR_index)
{
	switch(FSR_index)
	{
		case 0:ProstheticHand_SensorData.FSR1_NormalPressure=adc_data[10]/4096.0*20;//FSR1，插值得出计算公式
			break;
		case 1:ProstheticHand_SensorData.FSR2_NormalPressure=adc_data[11]/4096.0*20;//FSR2，插值得出计算公式
			break;
		case 2:ProstheticHand_SensorData.FSR3_NormalPressure=adc_data[0]/4096.0*20;//FSR3，插值得出计算公式
			break;
		case 3:ProstheticHand_SensorData.FSR4_NormalPressure=adc_data[1]/4096.0*20;//FSR4，插值得出计算公式
			break;
		case 4:ProstheticHand_SensorData.FSR5_NormalPressure=adc_data[2]/4096.0*20;//FSR5，插值得出计算公式
			break;
		default:
			break;
	}
}

/*
*****************************************************************************
*@brief		Bending数据处理函数，插值计算公式在Controller中写
*@param		uint16_t *adc_data：存放ADC数据的数组
*@param		uint8_t Bending_index：待处理的Bending数据编号
*@retval	None
*@par
*****************************************************************************
*/
static void Bending_DataProcess(uint16_t *adc_data,uint8_t Bending_index)
{
	switch(Bending_index)
	{
		case 0:ProstheticHand_SensorData.Thumb_Bending=adc_data[3]/4096.0*20;//Thumb_Bending，插值得出计算公式
			break;
		case 1:ProstheticHand_SensorData.IndexFinger_Bending=adc_data[4]/4096.0*20;//IndexFinger_Bending，插值得出计算公式
			break;
		case 2:ProstheticHand_SensorData.MiddleFinger_Bending=adc_data[5]/4096.0*20;//MiddleFinger_Bending，插值得出计算公式
			break;
		case 3:ProstheticHand_SensorData.RingFinger_Bending=adc_data[6]/4096.0*20;//RingFinger_Bending，插值得出计算公式
			break;
		case 4:ProstheticHand_SensorData.LittleFinger_Bending=adc_data[7]/4096.0*20;//LittleFinger_Bending，插值得出计算公式
			break;
		default:
			break;
	}
}

/*
*****************************************************************************
*@brief		ADCBackup数据处理函数，插值计算公式在Controller中写
*@param		uint16_t *adc_data：存放ADC数据的数组
*@param		uint8_t ADCBackup_index：待处理的ADCBackup数据编号
*@retval	None
*@par
*****************************************************************************
*/
static void ADCBackup_DataProcess(uint16_t *adc_data,uint8_t ADCBackup_index)
{
	switch(ADCBackup_index)
	{
		case 0:ProstheticHand_SensorData.ADC_Backup1=adc_data[8]/4096.0*3.3;//ADCBackup1，插值得出计算公式
			break;
		case 1:ProstheticHand_SensorData.ADC_Backup2=adc_data[9]/4096.0*3.3;//ADCBackup2，插值得出计算公式
			break;
		default:
			break;
	}
}

/*
*****************************************************************************
*@brief		ADC数据处理函数
*@param		uint16_t *adc_data：存放ADC数据的数组
*@retval	None
*@par
*****************************************************************************
*/
static void ADC_DataProcess(uint16_t *adc_data)
{
	uint8_t i=0;
	for(i=0;i<5;i++)
	{
		FSR_DataProcess(adc_data,i);
		Bending_DataProcess(adc_data,i);
		if(i<2){
			ADCBackup_DataProcess(adc_data,i);
		}
	}	
}

/* USER CODE BEGIN 1 */
void ADC_CollectionAndProcessTask(void const * argument)
{
  /* USER CODE BEGIN ADC_CollectionAndProcessTask */
  /* Infinite loop */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) Adc_Data, 12);
	for(;;)
  {
		ADC_DataProcess(Adc_Data);
		osDelay(5);
  }
  /* USER CODE END ADC_CollectionAndProcessTask */
}


/*
*****************************************************************************
*@brief		取值函数，搬运假肢手外接传感器数据
*@param		ProstheticHand_SensorData_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_ProstheticHand_SensorData(ProstheticHand_SensorData_n name_num,void*extern_data)//取值函数，搬运假肢手外接传感器数据,@ProstheticHand_SensorData_n
{
	switch(name_num)
	{
		case FSR1_NormalPressure_n:
			*(float*)extern_data=ProstheticHand_SensorData.FSR1_NormalPressure;
			break;
		case FSR2_NormalPressure_n:
			*(float*)extern_data=ProstheticHand_SensorData.FSR2_NormalPressure;
			break;
		case FSR3_NormalPressure_n:
			*(float*)extern_data=ProstheticHand_SensorData.FSR3_NormalPressure;
			break;
		case FSR4_NormalPressure_n:
			*(float*)extern_data=ProstheticHand_SensorData.FSR4_NormalPressure;
			break;
		case FSR5_NormalPressure_n:
			*(float*)extern_data=ProstheticHand_SensorData.FSR5_NormalPressure;
			break;
		case Thumb_Bending_n:
			*(float*)extern_data=ProstheticHand_SensorData.Thumb_Bending;
			break;
		case IndexFinger_Bending_n:
			*(float*)extern_data=ProstheticHand_SensorData.IndexFinger_Bending;
			break;
		case MiddleFinger_Bending_n:
			*(float*)extern_data=ProstheticHand_SensorData.MiddleFinger_Bending;
			break;
		case RingFinger_Bending_n:
			*(float*)extern_data=ProstheticHand_SensorData.RingFinger_Bending;
			break;
		case LittleFinger_Bending_n:
			*(float*)extern_data=ProstheticHand_SensorData.LittleFinger_Bending;
			break;
		case ADC_Backup1_n:
			*(float*)extern_data=ProstheticHand_SensorData.ADC_Backup1;
			break;
		case ADC_Backup2_n:
			*(float*)extern_data=ProstheticHand_SensorData.ADC_Backup2;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		修改假肢手外接传感器数据
*@param		ProstheticHand_SensorData_n name_num：修改数据对应的枚举变量
*@param		void*extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_ProstheticHand_SensorData(ProstheticHand_SensorData_n name_num,void*extern_data)//修改假肢手外接传感器数据,@ProstheticHand_SensorData_n
{
	switch(name_num)
	{
		case FSR1_NormalPressure_n:
			ProstheticHand_SensorData.FSR1_NormalPressure=*(float*)extern_data;
			break;
		case FSR2_NormalPressure_n:
			ProstheticHand_SensorData.FSR2_NormalPressure=*(float*)extern_data;
			break;
		case FSR3_NormalPressure_n:
			ProstheticHand_SensorData.FSR3_NormalPressure=*(float*)extern_data;
			break;
		case FSR4_NormalPressure_n:
			ProstheticHand_SensorData.FSR4_NormalPressure=*(float*)extern_data;
			break;
		case FSR5_NormalPressure_n:
			ProstheticHand_SensorData.FSR5_NormalPressure=*(float*)extern_data;
			break;
		case Thumb_Bending_n:
			ProstheticHand_SensorData.Thumb_Bending=*(float*)extern_data;
			break;
		case IndexFinger_Bending_n:
			ProstheticHand_SensorData.IndexFinger_Bending=*(float*)extern_data;
			break;
		case MiddleFinger_Bending_n:
			ProstheticHand_SensorData.MiddleFinger_Bending=*(float*)extern_data;
			break;
		case RingFinger_Bending_n:
			ProstheticHand_SensorData.RingFinger_Bending=*(float*)extern_data;
			break;
		case LittleFinger_Bending_n:
			ProstheticHand_SensorData.LittleFinger_Bending=*(float*)extern_data;
			break;
		case ADC_Backup1_n:
			ProstheticHand_SensorData.ADC_Backup1=*(float*)extern_data;
			break;
		case ADC_Backup2_n:
			ProstheticHand_SensorData.ADC_Backup2=*(float*)extern_data;
			break;
		default:			
			break;
	}
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
