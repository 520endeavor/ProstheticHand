/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#include "can.h"
#include "gpio.h"

#include "math.h"

struct {
	volatile float  ThumbJointMot_AutoModeSpeed;					//Ĵָ�źϵ���Զ�ģʽת������
	volatile float  ThumbSwingMot_AutoModeSpeed;					//Ĵָ�ڶ�����Զ�ģʽת������
	volatile float  IndexFingerSwingMot_AutoModeSpeed;		//ʳָ�ڶ�����Զ�ģʽת������
	volatile float  MiddleFingerSwingMot_AutoModeSpeed;		//��ָ�ڶ�����Զ�ģʽת������
	volatile float  RingFingerSwingMot_AutoModeSpeed;			//����ָ�ڶ�����Զ�ģʽת������
	volatile float  LittleFingerSwingMot_AutoModeSpeed;		//Сָ�ڶ�����Զ�ģʽת������
	volatile uint16_t ThumbJointMot_Current;							//Ĵָ�źϵ����������
	volatile uint16_t ThumbSwingMot_Current;							//Ĵָ�ڶ������������
	volatile uint16_t IndexFingerSwingMot_Current;				//ʳָ�ڶ������������
	volatile uint16_t MiddleFingerSwingMot_Current;				//��ָ�ڶ������������
	volatile uint16_t RingFingerSwingMot_Current;					//����ָ�ڶ������������
	volatile uint16_t LittleFingerSwingMot_Current;				//Сָ�ڶ������������
}Faulhaber_FeedbackData;//Faulhaber�����������

struct{
	volatile uint16_t ThumbJointMot_NMT_Status;
	volatile uint16_t ThumbSwingMot_NMT_Status;
	volatile uint16_t IndexFingerSwingMot_NMT_Status;
	volatile uint16_t MiddleFingerSwingMot_NMT_Status;
	volatile uint16_t RingFingerSwingMot_NMT_Status;
	volatile uint16_t LittleFingerSwingMot_NMT_Status;
	volatile uint8_t ThumbJointMot_Bootup_Status;
	volatile uint8_t ThumbSwingMot_Bootup_Status;
	volatile uint8_t IndexFingerSwingMot_Bootup_Status;
	volatile uint8_t MiddleFingerSwingMot_Bootup_Status;
	volatile uint8_t RingFingerSwingMot_Bootup_Status;
	volatile uint8_t LittleFingerSwingMot_Bootup_Status;
}Faulhaber_StatusData;//Faulhaber�������״̬����

struct{
	volatile float FSR1_NormalPressure;										//FSR1����ѹ������
	volatile float FSR2_NormalPressure;										//FSR2����ѹ������
	volatile float FSR3_NormalPressure;										//FSR3����ѹ������
	volatile float FSR4_NormalPressure;										//FSR4����ѹ������
	volatile float FSR5_NormalPressure;										//FSR5����ѹ������
	volatile float Thumb_Bending;												  //Ĵָ����������
	volatile float IndexFinger_Bending;									  //ʳָ����������
	volatile float MiddleFinger_Bending;								  //��ָ����������
	volatile float RingFinger_Bending;									  //����ָ����������
	volatile float LittleFinger_Bending;								  //Сָ����������
	volatile float ADC_Backup1;								  					//����ADC����
	volatile float ADC_Backup2;								  					//����ADC����
}ProstheticHand_SensorData;//��֫����Ӵ���������


union Integer_Floating{																	
	volatile uint8_t Integer_Data[4];
	volatile float Floating_Data;
};//������������ת���ṹ

/* USER CODE BEGIN 0 */
static CanTxMsgTypeDef s1_TxMsg; //CAN1������Ϣ
static CanRxMsgTypeDef s1_RxMsg; //CAN1������Ϣ
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

osThreadId CAN1_ReceiveData_AnalysisTaskHandle;
osThreadId CAN1_Transmit3TaskHandle;
osThreadId CAN1_Transmit1TaskHandle;
osThreadId CAN1_Transmit2TaskHandle;

osMessageQId CAN1TransmitQueue3Handle;									//CAN1��������3����
osThreadId Get_CAN1TransmitQueue3Handle(void){
	return CAN1TransmitQueue3Handle;
}

osMessageQId CAN1TransmitQueue1Handle;									//CAN1��������1����
osThreadId Get_CAN1TransmitQueue1Handle(void){
	return CAN1TransmitQueue1Handle;
}

osMessageQId CAN1TransmitQueue2Handle;									//CAN1��������2����
osThreadId Get_CAN1TransmitQueue2Handle(void){
	return CAN1TransmitQueue2Handle;
}

void CAN1_ReceiveData_AnalysisTask(void const * argument);//can1�������ݽ�������
void CAN1_Transmit3Task(void const * argument);
void CAN1_Transmit1Task(void const * argument);
void CAN1_Transmit2Task(void const * argument);

static void CAN_FREERTOS_Init(void) 
{
	/* definition and creation of CAN1_ReceiveData_AnalysisTask */
	osThreadDef(CAN1_ReceiveData_AnalysisTask, CAN1_ReceiveData_AnalysisTask, osPriorityNormal, 0, 128);
	CAN1_ReceiveData_AnalysisTaskHandle = osThreadCreate(osThread(CAN1_ReceiveData_AnalysisTask), NULL);
	/* definition and creation of CAN1_Transmit3Task */
	osThreadDef(CAN1_Transmit3Task, CAN1_Transmit3Task, osPriorityNormal, 0, 128);
	CAN1_Transmit3TaskHandle = osThreadCreate(osThread(CAN1_Transmit3Task), NULL);
	/* definition and creation of CAN1_Transmit1Task */
	osThreadDef(CAN1_Transmit1Task, CAN1_Transmit1Task, osPriorityNormal, 0, 128);
	CAN1_Transmit1TaskHandle = osThreadCreate(osThread(CAN1_Transmit1Task), NULL);
	/* definition and creation of CAN1_Transmit2Task */
	osThreadDef(CAN1_Transmit2Task, CAN1_Transmit2Task, osPriorityNormal, 0, 128);
	CAN1_Transmit2TaskHandle = osThreadCreate(osThread(CAN1_Transmit2Task), NULL);
	/* Create the queue(s) */
	/* definition and creation of CAN1TransmitQueue3 */
  osMessageQDef(CAN1TransmitQueue3,Net_Mot_Node_Num, CAN1TransmitMessage3Typedef);
  CAN1TransmitQueue3Handle = osMessageCreate(osMessageQ(CAN1TransmitQueue3), NULL);	
	/* definition and creation of CAN1TransmitQueue1 */
  osMessageQDef(CAN1TransmitQueue1,Net_Mot_Node_Num*2, CAN1TransmitMessage1Typedef);
  CAN1TransmitQueue1Handle = osMessageCreate(osMessageQ(CAN1TransmitQueue1), NULL);	
	/* definition and creation of CAN1TransmitQueue2 */
  osMessageQDef(CAN1TransmitQueue2,Net_Mot_Node_Num*2, CAN1TransmitMessage2Typedef);
  CAN1TransmitQueue2Handle = osMessageCreate(osMessageQ(CAN1TransmitQueue2), NULL);	
}

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 24;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_10TQ;
  hcan1.Init.BS2 = CAN_BS2_7TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	APP_CAN_Config(&hcan1);											//����CAN������
	CAN_FREERTOS_Init();												//
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = STM32_CAN1_RX_Pin|STM32_CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOA, STM32_CAN1_RX_Pin|STM32_CAN1_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/*
*****************************************************************************
*@brief		CAN���������ú���	
*@param		CAN_HandleTypeDef* canHandle��can�ṹ�����		
*@retval	None
*@par
*****************************************************************************
*/
void APP_CAN_Config(CAN_HandleTypeDef* canHandle){
	CAN_FilterConfTypeDef sFilterConfig;
	//���ù�����
	if(canHandle->Instance==CAN1){
		//configure the filter0,											���������ͨ��PDO1Э�鷢�����ݸ���֫�ֿ�����
		sFilterConfig.FilterIdHigh=(((uint32_t)FaulhaberMotPDO1TxID<<21)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)FaulhaberMotPDO1TxID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
		sFilterConfig.FilterMaskIdHigh=(0xFF80<<5)|0x001F;
		sFilterConfig.FilterMaskIdLow=0xFFFF;		
		sFilterConfig.FilterNumber=0;								//������0
		sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
		sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterActivation=ENABLE;
		sFilterConfig.BankNumber=0;
		HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
		//configure the filter1,											���������ͨ��PDO2Э�鷢�����ݸ���֫�ֿ�����
		sFilterConfig.FilterIdHigh=(((uint32_t)FaulhaberMotPDO2TxID<<21)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)FaulhaberMotPDO2TxID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;		
		sFilterConfig.FilterNumber=1;								//������1
		HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
		//configure the filter2,											���������ͨ��PDO3Э�鷢�����ݸ���֫�ֿ�����
		sFilterConfig.FilterIdHigh=(((uint32_t)FaulhaberMotPDO3TxID<<21)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)FaulhaberMotPDO3TxID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;		
		sFilterConfig.FilterNumber=2;								//������2
		HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
		//configure the filter3,											���������ͨ��SDOЭ�鷢�����ݸ���֫�ֿ�����
		sFilterConfig.FilterIdHigh=(((uint32_t)FaulhaberMotSDOTxID<<21)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)FaulhaberMotSDOTxID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;		
		sFilterConfig.FilterNumber=3;								//������3
		HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);		
		//configure the filter4,											Collection�������ݸ�Controller
		sFilterConfig.FilterIdHigh=(((uint32_t)Collection_Controller_INFO0_ID<<21)&0xffff0000)>>16;					//32λID
		sFilterConfig.FilterIdLow=(((uint32_t)Collection_Controller_INFO0_ID<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;		
		sFilterConfig.FilterMaskIdHigh=(0xFFF8<<5)|0x001F;
		sFilterConfig.FilterMaskIdLow=0xFFFF;		
		sFilterConfig.FilterNumber=4;								//������4
		HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);	
		
		//initialize the msg pointer
		hcan1	.pTxMsg=&s1_TxMsg;
		hcan1	.pRxMsg=&s1_RxMsg;
		//configuer Rx message
		hcan1 .pRxMsg->StdId=0x7FF	;//Max_Data = 0x7FF			
		hcan1 .pRxMsg->IDE=CAN_ID_STD;
		hcan1 .pRxMsg->RTR=CAN_RTR_DATA;		
//		hcan1 .pRxMsg->DLC=8;
		//configuer Tx message
		hcan1 .pTxMsg->StdId=0x7FE;//Max_Data = x7FF	
		hcan1 .pTxMsg->ExtId=0x7FE;//Max_Data = 0x1FFFFFFF
		hcan1 .pTxMsg->IDE=CAN_ID_STD;
		hcan1 .pTxMsg->RTR=CAN_RTR_DATA;
		hcan1 .pTxMsg->DLC=8;			
	}	
}


/*
*****************************************************************************
*@brief		����PDO1���յ�������
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void CAN1_PDO1ReceiveData_Analysis(uint8_t *receive_data,uint32_t receive_ID)
{
	switch(receive_ID){
		case FaulhaberMotPDO1TxID+1:
			if(receive_data[0]==(uint8_t)FaulhaberMot_PDO1NMTSuccessData&&receive_data[1]==(uint8_t)(FaulhaberMot_PDO1NMTSuccessData>>8)){
				Faulhaber_StatusData.ThumbJointMot_NMT_Status=FaulhaberMot_PDO1NMTSuccessData;
			}
			else			
			break;
		case FaulhaberMotPDO1TxID+2:
		if(receive_data[0]==(uint8_t)FaulhaberMot_PDO1NMTSuccessData&&receive_data[1]==(uint8_t)(FaulhaberMot_PDO1NMTSuccessData>>8)){
			Faulhaber_StatusData.ThumbSwingMot_NMT_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		case FaulhaberMotPDO1TxID+3:
		if(receive_data[0]==(uint8_t)FaulhaberMot_PDO1NMTSuccessData&&receive_data[1]==(uint8_t)(FaulhaberMot_PDO1NMTSuccessData>>8)){
			Faulhaber_StatusData.IndexFingerSwingMot_NMT_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		case FaulhaberMotPDO1TxID+4:
		if(receive_data[0]==(uint8_t)FaulhaberMot_PDO1NMTSuccessData&&receive_data[1]==(uint8_t)(FaulhaberMot_PDO1NMTSuccessData>>8)){
			Faulhaber_StatusData.MiddleFingerSwingMot_NMT_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		case FaulhaberMotPDO1TxID+5:
		if(receive_data[0]==(uint8_t)FaulhaberMot_PDO1NMTSuccessData&&receive_data[1]==(uint8_t)(FaulhaberMot_PDO1NMTSuccessData>>8)){
			Faulhaber_StatusData.RingFingerSwingMot_NMT_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		case FaulhaberMotPDO1TxID+6:
		if(receive_data[0]==(uint8_t)FaulhaberMot_PDO1NMTSuccessData&&receive_data[1]==(uint8_t)(FaulhaberMot_PDO1NMTSuccessData>>8)){
			Faulhaber_StatusData.LittleFingerSwingMot_NMT_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		
		case FaulhaberMotBootUpID+1:
			if(receive_data[0]==(uint8_t)FaulhaberMot_BootupSuccessData){
				Faulhaber_StatusData.ThumbJointMot_Bootup_Status=FaulhaberMot_PDO1NMTSuccessData;
			}
			else			
			break;
		case FaulhaberMotBootUpID+2:
		if(receive_data[0]==(uint8_t)FaulhaberMot_BootupSuccessData){
			Faulhaber_StatusData.ThumbSwingMot_Bootup_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		case FaulhaberMotBootUpID+3:
		if(receive_data[0]==(uint8_t)FaulhaberMot_BootupSuccessData){
			Faulhaber_StatusData.IndexFingerSwingMot_Bootup_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		case FaulhaberMotBootUpID+4:
		if(receive_data[0]==(uint8_t)FaulhaberMot_BootupSuccessData){
			Faulhaber_StatusData.MiddleFingerSwingMot_Bootup_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		case FaulhaberMotBootUpID+5:
		if(receive_data[0]==(uint8_t)FaulhaberMot_BootupSuccessData){
			Faulhaber_StatusData.RingFingerSwingMot_Bootup_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
		case FaulhaberMotBootUpID+6:
		if(receive_data[0]==(uint8_t)FaulhaberMot_BootupSuccessData){
			Faulhaber_StatusData.LittleFingerSwingMot_Bootup_Status=FaulhaberMot_PDO1NMTSuccessData;
		}		
		break;
	}
}
/*
*****************************************************************************
*@brief		����SDO���յ�������
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void CAN1_SDOReceiveData_Analysis(uint8_t *receive_data,uint32_t receive_ID)
{
	switch(receive_ID){
		case FaulhaberMotSDOTxID+1:
			if(receive_data[0]==FaulhaberMot_UploadResponse4&&receive_data[1]==(uint8_t)FahlhaberMot_CIA402_VelocityActualSensorValue&&receive_data[2]==(uint8_t)(FahlhaberMot_CIA402_VelocityActualSensorValue>>8)){
				Faulhaber_FeedbackData.ThumbJointMot_AutoModeSpeed=(uint32_t)receive_data[4]+((uint32_t)receive_data[5]<<8)+((uint32_t)receive_data[6]<<16)+((uint32_t)receive_data[7]<<24);
			}		
			break;
		case FaulhaberMotSDOTxID+2:
			if(receive_data[0]==FaulhaberMot_UploadResponse4&&receive_data[1]==(uint8_t)FahlhaberMot_CIA402_VelocityActualSensorValue&&receive_data[2]==(uint8_t)(FahlhaberMot_CIA402_VelocityActualSensorValue>>8)){
					Faulhaber_FeedbackData.ThumbSwingMot_AutoModeSpeed=(uint32_t)receive_data[4]+((uint32_t)receive_data[5]<<8)+((uint32_t)receive_data[6]<<16)+((uint32_t)receive_data[7]<<24);
			}
			break;
		case FaulhaberMotSDOTxID+3:
			if(receive_data[0]==FaulhaberMot_UploadResponse4&&receive_data[1]==(uint8_t)FahlhaberMot_CIA402_VelocityActualSensorValue&&receive_data[2]==(uint8_t)(FahlhaberMot_CIA402_VelocityActualSensorValue>>8)){
					Faulhaber_FeedbackData.IndexFingerSwingMot_AutoModeSpeed=(uint32_t)receive_data[4]+((uint32_t)receive_data[5]<<8)+((uint32_t)receive_data[6]<<16)+((uint32_t)receive_data[7]<<24);
			}
			break;
		case FaulhaberMotSDOTxID+4:
			if(receive_data[0]==FaulhaberMot_UploadResponse4&&receive_data[1]==(uint8_t)FahlhaberMot_CIA402_VelocityActualSensorValue&&receive_data[2]==(uint8_t)(FahlhaberMot_CIA402_VelocityActualSensorValue>>8)){
					Faulhaber_FeedbackData.MiddleFingerSwingMot_AutoModeSpeed=(uint32_t)receive_data[4]+((uint32_t)receive_data[5]<<8)+((uint32_t)receive_data[6]<<16)+((uint32_t)receive_data[7]<<24);
			}
			break;
		case FaulhaberMotSDOTxID+5:
			if(receive_data[0]==FaulhaberMot_UploadResponse4&&receive_data[1]==(uint8_t)FahlhaberMot_CIA402_VelocityActualSensorValue&&receive_data[2]==(uint8_t)(FahlhaberMot_CIA402_VelocityActualSensorValue>>8)){
					Faulhaber_FeedbackData.RingFingerSwingMot_AutoModeSpeed=(uint32_t)receive_data[4]+((uint32_t)receive_data[5]<<8)+((uint32_t)receive_data[6]<<16)+((uint32_t)receive_data[7]<<24);
			}
			break;
		case FaulhaberMotSDOTxID+6:
			if(receive_data[0]==FaulhaberMot_UploadResponse4&&receive_data[1]==(uint8_t)FahlhaberMot_CIA402_VelocityActualSensorValue&&receive_data[2]==(uint8_t)(FahlhaberMot_CIA402_VelocityActualSensorValue>>8)){
					Faulhaber_FeedbackData.LittleFingerSwingMot_AutoModeSpeed=(uint32_t)receive_data[4]+((uint32_t)receive_data[5]<<8)+((uint32_t)receive_data[6]<<16)+((uint32_t)receive_data[7]<<24);
			}
			break;
		default:
			break;
	}
}

/*
*****************************************************************************
*@brief		����FSR1��ѹ-ѹ����Ϲ�ʽ
*@param		float FSR_Data��FSR1ADC����
*@retval	None
*@par
*****************************************************************************
*/
static float FSR1_DataFitting_Function(float FSR_Data)
{
	float FSR_ForceData=0;
	FSR_Data=FSR_Data/20*4.096;
	FSR_ForceData=0.1107*pow(FSR_Data,7)-1.187*pow(FSR_Data,6)+5.018*pow(FSR_Data,5)-10.41*pow(FSR_Data,4)+10.86*pow(FSR_Data,3)-4.821*pow(FSR_Data,2)+0.8464*FSR_Data+0.4987;
	if(FSR_ForceData>20){
		FSR_ForceData=20;
	}
	return FSR_ForceData;
}

/*
*****************************************************************************
*@brief		����FSR2��ѹ-ѹ����Ϲ�ʽ
*@param		float FSR_Data��FSR1ADC����
*@retval	None
*@par
*****************************************************************************
*/
static float FSR2_DataFitting_Function(float FSR_Data)
{
	float FSR_ForceData=0;
	FSR_Data=FSR_Data/20*4.096;
	FSR_ForceData=0.06684*pow(FSR_Data,7)-0.777*pow(FSR_Data,6)+3.772*pow(FSR_Data,5)-9.565*pow(FSR_Data,4)+13.15*pow(FSR_Data,3)-8.556*pow(FSR_Data,2)+2.329*FSR_Data+0.5956;
	if(FSR_ForceData>20){
		FSR_ForceData=20;
	}
	return FSR_ForceData;
}

/*
*****************************************************************************
*@brief		����FSR3��ѹ-ѹ����Ϲ�ʽ
*@param		float FSR_Data��FSR1ADC����
*@retval	None
*@par
*****************************************************************************
*/
static float FSR3_DataFitting_Function(float FSR_Data)
{
	float FSR_ForceData=0;
	FSR_Data=FSR_Data/20*4.096;
	FSR_ForceData=0.05259*pow(FSR_Data,7)-0.5163*pow(FSR_Data,6)+2.067*pow(FSR_Data,5)-4.236*pow(FSR_Data,4)+4.585*pow(FSR_Data,3)-1.975*pow(FSR_Data,2)+0.7993*FSR_Data+0.4921;
	if(FSR_ForceData>20){
		FSR_ForceData=20;
	}
	return FSR_ForceData;
}

/*
*****************************************************************************
*@brief		����FSR4��ѹ-ѹ����Ϲ�ʽ
*@param		float FSR_Data��FSR1ADC����
*@retval	None
*@par
*****************************************************************************
*/
static float FSR4_DataFitting_Function(float FSR_Data)
{
	float FSR_ForceData=0;
	FSR_Data=FSR_Data/20*4.096;
	FSR_ForceData=0.01195*pow(FSR_Data,9)-0.1391*pow(FSR_Data,8)+0.6527*pow(FSR_Data,7)-1.666*pow(FSR_Data,6)+2.942*pow(FSR_Data,5)-4.396*pow(FSR_Data,4)+4.79*pow(FSR_Data,3)-2.2*pow(FSR_Data,2)+0.8697*FSR_Data+0.3957;
	if(FSR_ForceData>20){
		FSR_ForceData=20;
	}
	return FSR_ForceData;
}

/*
*****************************************************************************
*@brief		����FSR5��ѹ-ѹ����Ϲ�ʽ
*@param		float FSR_Data��FSR1ADC����
*@retval	None
*@par
*****************************************************************************
*/
static float FSR5_DataFitting_Function(float FSR_Data)
{
	float FSR_ForceData=0;
	FSR_Data=FSR_Data/20*4.096;
	FSR_ForceData=0.07174*pow(FSR_Data,7)-0.7588*pow(FSR_Data,6)+3.207*pow(FSR_Data,5)-6.808*pow(FSR_Data,4)+7.581*pow(FSR_Data,3)-3.645*pow(FSR_Data,2)+0.9689*FSR_Data+0.4842;
	if(FSR_ForceData>20){
		FSR_ForceData=20;
	}
	return FSR_ForceData;
}

/*
*****************************************************************************
*@brief		����FSR1AndFSR2_ADC_INFO
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
	union Integer_Floating FSR1_PressureData,	FSR2_PressureData;
static void FSR1AndFSR2_ADC_INFO_Analysis(uint8_t *receive_data)
{
//	union Integer_Floating FSR1_PressureData,	FSR2_PressureData;
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		FSR1_PressureData.Integer_Data[i]=receive_data[i];
		FSR2_PressureData.Integer_Data[i]=receive_data[i+4];
	}
	ProstheticHand_SensorData.FSR1_NormalPressure=FSR1_DataFitting_Function(FSR1_PressureData.Floating_Data);
	ProstheticHand_SensorData.FSR2_NormalPressure=FSR2_DataFitting_Function(FSR2_PressureData.Floating_Data);
}

/*
*****************************************************************************
*@brief		����FSR3AndFSR4_ADC_INFO
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void FSR3AndFSR4_ADC_INFO_Analysis(uint8_t *receive_data)
{
	union Integer_Floating FSR3_PressureData,FSR4_PressureData;
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		FSR3_PressureData.Integer_Data[i]=receive_data[i];
		FSR4_PressureData.Integer_Data[i]=receive_data[i+4];
	}
	ProstheticHand_SensorData.FSR3_NormalPressure=FSR3_DataFitting_Function(FSR3_PressureData.Floating_Data);
	ProstheticHand_SensorData.FSR4_NormalPressure=FSR4_DataFitting_Function(FSR4_PressureData.Floating_Data);
}

/*
*****************************************************************************
*@brief		����FSR5AndBending1_INFO
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void FSR5AndBending1_INFO_Analysis(uint8_t *receive_data)
{
	union Integer_Floating FSR5_PressureData,Thumb_BendingData;
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		FSR5_PressureData.Integer_Data[i]=receive_data[i];
		Thumb_BendingData.Integer_Data[i]=receive_data[i+4];
	}
	ProstheticHand_SensorData.FSR5_NormalPressure=FSR5_DataFitting_Function(FSR5_PressureData.Floating_Data);
	ProstheticHand_SensorData.Thumb_Bending=Thumb_BendingData.Floating_Data;
}

/*
*****************************************************************************
*@brief		����Bending2AndBending3_INFO
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void Bending2AndBending3_INFO_Analysis(uint8_t *receive_data)
{
	union Integer_Floating IndexFinger_BendingData,MiddleFinger_BendingData;
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		IndexFinger_BendingData.Integer_Data[i]=receive_data[i];
		MiddleFinger_BendingData.Integer_Data[i]=receive_data[i+4];
	}
	ProstheticHand_SensorData.IndexFinger_Bending=IndexFinger_BendingData.Floating_Data;
	ProstheticHand_SensorData.MiddleFinger_Bending=MiddleFinger_BendingData.Floating_Data;
}

/*
*****************************************************************************
*@brief		����Bending4AndBending5_INFO
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void Bending4AndBending5_INFO_Analysis(uint8_t *receive_data)
{
	union Integer_Floating RingFinger_BendingData,LittleFinger_BendingData;
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		RingFinger_BendingData.Integer_Data[i]=receive_data[i];
		LittleFinger_BendingData.Integer_Data[i]=receive_data[i+4];
	}
	ProstheticHand_SensorData.RingFinger_Bending=RingFinger_BendingData.Floating_Data;
	ProstheticHand_SensorData.LittleFinger_Bending=LittleFinger_BendingData.Floating_Data;
}

/*
*****************************************************************************
*@brief		����ADC_Backup1AndADC_Backup2_INFO
*@param		uint8_t *receive_data��������յ�������
*@retval	None
*@par
*****************************************************************************
*/
static void ADC_Backup1AndADC_Backup2_INFO_Analysis(uint8_t *receive_data)
{
	union Integer_Floating ADC_Backup1_Data,ADC_Backup2_Data;
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		ADC_Backup1_Data.Integer_Data[i]=receive_data[i];
		ADC_Backup2_Data.Integer_Data[i]=receive_data[i+4];
	}
	ProstheticHand_SensorData.ADC_Backup1=ADC_Backup1_Data.Floating_Data;
	ProstheticHand_SensorData.ADC_Backup2=ADC_Backup2_Data.Floating_Data;
}

/*
*****************************************************************************
*@brief		����Collection_ControllerINFO
*@param		uint8_t *receive_data��������յ�������
*@param		uint32_t receive_ID,�������ݵ�ID��Դ
*@retval	None
*@par
*****************************************************************************
*/
static void Collection_Controller_INFO_Analysis(uint8_t *receive_data,uint32_t receive_ID)
{
	switch(receive_ID){
		case Collection_Controller_INFO0_ID:
			FSR1AndFSR2_ADC_INFO_Analysis(receive_data);		//����FSR1AndFSR2_ADC_INFO
			break;
		case Collection_Controller_INFO1_ID:
			FSR3AndFSR4_ADC_INFO_Analysis(receive_data);		//����FSR3AndFSR4_ADC_INFO
			break;
		case Collection_Controller_INFO2_ID:
			FSR5AndBending1_INFO_Analysis(receive_data);		//����FSR5AndBending1_INFO
			break;
		case Collection_Controller_INFO3_ID:
			Bending2AndBending3_INFO_Analysis(receive_data);//����Bending2AndBending3_INFO
			break;
		case Collection_Controller_INFO4_ID:
			Bending4AndBending5_INFO_Analysis(receive_data);//����Bending4AndBending5_INFO
			break;
		case Collection_Controller_INFO5_ID:
			ADC_Backup1AndADC_Backup2_INFO_Analysis(receive_data);//����ADC_Backup1AndADC_Backup2_INFO
			break;
		default:
			break;
	}
}


void CAN1_ReceiveData_AnalysisTask(void const * argument)
{	
  /* USER CODE BEGIN CAN1_ReceiveData_AnalysisTask */
  /* Infinite loop */			
	uint8_t Receive_Data[8];
	uint32_t Receive_ID;
  for(;;)
  {				
		if(HAL_CAN_Receive(&hcan1, CAN_FILTER_FIFO0, 100)==HAL_OK){
			Receive_ID=hcan1.pRxMsg->StdId;
			for(uint8_t i=0;i<8;i++){
				Receive_Data[i]=hcan1.pRxMsg->Data[i];
			}
			if(Receive_ID>=FaulhaberMotSDOTxID+ThumbJointMot_Node&&Receive_ID<=FaulhaberMotSDOTxID+LittleFingerSwingMot_Node){
				CAN1_SDOReceiveData_Analysis(Receive_Data,Receive_ID);
			}
			else if((Receive_ID>=FaulhaberMotPDO1TxID+ThumbJointMot_Node&&Receive_ID<=FaulhaberMotPDO1TxID+LittleFingerSwingMot_Node)||(Receive_ID>=FaulhaberMotBootUpID+ThumbJointMot_Node&&Receive_ID<=FaulhaberMotBootUpID+LittleFingerSwingMot_Node)){
				CAN1_PDO1ReceiveData_Analysis(Receive_Data,Receive_ID);
			}
			else if(Receive_ID==Collection_Controller_INFO0_ID||Receive_ID==Collection_Controller_INFO1_ID||Receive_ID==Collection_Controller_INFO2_ID
				||Receive_ID==Collection_Controller_INFO3_ID||Receive_ID==Collection_Controller_INFO4_ID||Receive_ID==Collection_Controller_INFO5_ID){
				Collection_Controller_INFO_Analysis(Receive_Data,Receive_ID);
			}
		}
		osDelay(2);
  }		
  /* USER CODE END CAN1_ReceiveData_AnalysisTask */
}


/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��NMTЭ�鷢�����ݸ��������
*@param		uint32_t Mot_StdId,Specifies the standard identifier,NMT:0x000.
*@param		uint8_t Mot_CS,Start Remote Node Command Specifies
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127������0ѡ�����������нڵ�
*@retval	None
*@par
*****************************************************************************
*/
static void Faulhaber_CAN1_NMTSend_Mot(uint32_t Mot_StdId,uint8_t Mot_CS,uint8_t Mot_Node)
{
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;						//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;							//��׼֡
	hcan1.pTxMsg->StdId = Mot_StdId;						//���ͱ�׼֡ID  ����:0x001
	hcan1.pTxMsg->Data[0]=Mot_CS;								//Start Remote Node	
	hcan1.pTxMsg->Data[1]=Mot_Node;							//�ڵ��
	hcan1.pTxMsg->DLC =2;												//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);	
}

/*
*****************************************************************************
*@brief		��֫�ֿ�������������ʹ���Boot_Up
*@param		uint32_t Mot_StdId,Specifies the standard identifier,NMT:0x000.
*@param		uint8_t Bootup_cmd,���Bootup����
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127������0ѡ�����������нڵ�
*@retval	None
*@par
*****************************************************************************
*/
static void Faulhaber_CAN1_BootUpSend_Mot(uint32_t Mot_StdId,uint8_t Bootup_cmd,uint8_t Mot_Node)
{
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;							//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;								//��׼֡
	hcan1.pTxMsg->StdId = Mot_StdId+Mot_Node;			//���ͱ�׼֡ID ����:0x201
	hcan1.pTxMsg->Data[0]=Bootup_cmd;							//Boot_Up
	hcan1.pTxMsg->DLC =1;													//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);	
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��NMTЭ�鷢�����ݸ����������,���з���
*@param		uint32_t Mot_StdId,Specifies the standard identifier,NMT:0x000.
*@param		uint8_t Mot_CS,Start Remote Node Command Specifies
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127������0ѡ�����������нڵ�
*@retval	None
*@par
*****************************************************************************
*/
void CAN1_NMTSend_Mot(uint32_t Mot_StdId,uint8_t Mot_CS,uint8_t Mot_Node)
{
	CAN1TransmitMessage1Typedef MotCtrl_Data;
	MotCtrl_Data.CANTransmitID=Mot_StdId;
	MotCtrl_Data.CANTransmitData[0]=Mot_CS;
	MotCtrl_Data.CANTransmitData[1]=Mot_Node;
	xQueueSend(CAN1TransmitQueue1Handle,&MotCtrl_Data,(TickType_t)0);
}

/*
*****************************************************************************
*@brief		��֫�ֿ�������������ʹ���Boot_Up,���з���
*@param		uint32_t Mot_StdId,Specifies the standard identifier,NMT:0x000.
*@param		uint8_t Bootup_cmd,���Bootup����
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127������0ѡ�����������нڵ�
*@retval	None
*@par
*****************************************************************************
*/
 void CAN1_BootUpSend_Mot(uint32_t Mot_StdId,uint8_t Bootup_cmd,uint8_t Mot_Node)
{
	CAN1TransmitMessage1Typedef MotCtrl_Data;
	MotCtrl_Data.CANTransmitID=Mot_StdId;
	MotCtrl_Data.CANTransmitData[0]=Bootup_cmd;
	MotCtrl_Data.CANTransmitData[1]=Mot_Node;
	xQueueSend(CAN1TransmitQueue1Handle,&MotCtrl_Data,(TickType_t)0);
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��NMTЭ�鼰Boot_Up�����ʼ�����,���з��ͣ��ں����������
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_Init_Mot(void)
{
	CAN1_NMTSend_Mot(FaulhaberMotNMTID,FaulhaberMotNMT_CS_StartRemoteNode,AllOfNodes);					//start all remote nodes.
	CAN1_BootUpSend_Mot(FaulhaberMotBootUpID,FaulhaberMot_BootUp_Messege,ThumbJointMot_Node);					//Ĵָ�źϵ��boot up
	CAN1_BootUpSend_Mot(FaulhaberMotBootUpID,FaulhaberMot_BootUp_Messege,ThumbSwingMot_Node);					//Ĵָ�ڶ����boot up
	CAN1_BootUpSend_Mot(FaulhaberMotBootUpID,FaulhaberMot_BootUp_Messege,IndexFingerSwingMot_Node);		//ʳָ�ڶ����boot up
	CAN1_BootUpSend_Mot(FaulhaberMotBootUpID,FaulhaberMot_BootUp_Messege,MiddleFingerSwingMot_Node);	//��ָ�ڶ����boot up
	CAN1_BootUpSend_Mot(FaulhaberMotBootUpID,FaulhaberMot_BootUp_Messege,RingFingerSwingMot_Node);		//����ָ�ڶ����boot up
	CAN1_BootUpSend_Mot(FaulhaberMotBootUpID,FaulhaberMot_BootUp_Messege,LittleFingerSwingMot_Node);	//Сָ�ڶ����boot up
//	osDelay(10);			//ͨ�����Ƚ��뷢�ͺ�������ֹ���ж���
}


/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��PDO2Э�鷢�����ݸ����������
*@param		uint32_t Mot_StdId��Specifies the standard identifier.
					This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF
*@param		uint8_t PDO2_Mot_CMD��PDO1Э���е����������
*@param		uint32_t PDO2_Cmd_Data��PDO2Э�鷢������
*@retval	None
*@par
*****************************************************************************
*/
static void Faulhaber_CAN1_PDO2Send_Mot(uint32_t Mot_StdId,uint8_t PDO2_Mot_CMD,uint32_t PDO2_Cmd_Data)
{
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;										//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;											//��׼֡
	hcan1.pTxMsg->StdId = Mot_StdId;										//���ͱ�׼֡ID uint16t ����:0x201
	hcan1.pTxMsg->Data[0]=PDO2_Mot_CMD;									//PDO2Э���е����������
	hcan1.pTxMsg->Data[1]=PDO2_Cmd_Data;								//PDO2Э���е��������������,LL
	hcan1.pTxMsg->Data[2]=PDO2_Cmd_Data>>8;							//PDO2Э���е��������������,LH
	hcan1.pTxMsg->Data[3]=PDO2_Cmd_Data>>16;						//PDO2Э���е��������������,HL
	hcan1.pTxMsg->Data[4]=PDO2_Cmd_Data>>24;						//PDO2Э���е��������������,HH
	hcan1.pTxMsg->DLC =5;																//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);	
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��PDO2Э�鷢�����ݸ����������,���з���
*@param		uint32_t Mot_StdId��Specifies the standard identifier.
					This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF
*@param		uint8_t Mot_CS,Start Remote Node Command Specifies
*@retval	None
*@par
*****************************************************************************
*/
void CAN1_DataPDO2Send(uint32_t Mot_StdId,uint8_t PDO2_Mot_CMD,uint32_t PDO2_Cmd_Data)
{
	CAN1TransmitMessage3Typedef MotCtrl_Data;
	MotCtrl_Data.CANTransmitID=Mot_StdId;
	MotCtrl_Data.CANTransmit_CMD=PDO2_Mot_CMD;									//PDO3Э���е����������
	MotCtrl_Data.CANTransmitData=PDO2_Cmd_Data;									//PDO3Э���е����������
	xQueueSend(CAN1TransmitQueue3Handle,&MotCtrl_Data,(TickType_t)0);
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��PDO2Э��ʹ�ܵ��,���з���
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_PDO2Enable_Mot(uint8_t Mot_Node)
{
	CAN1_DataPDO2Send(FaulhaberMotPDO2RxID+Mot_Node,FaulhaberMot_PDO2EnableCMD,(uint32_t)0);	
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��PDO2Э��ʧ�ܵ��,���з���
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_PDO2Disable_Mot(uint8_t Mot_Node)
{
	CAN1_DataPDO2Send(FaulhaberMotPDO2RxID+Mot_Node,FaulhaberMot_PDO2DisableCMD,(uint32_t)0);
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��SDOЭ�鷢�����ݸ���������������ͱ�׼֡��11λID
*@param		uint32_t Mot_StdId��Specifies the standard identifier.
					This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF
*@param		uint8_t WriteODEntries ��������λdata bytes
*@param		uint16_t Mot_CMD_Index����������������ֵ�ָ�������յ���������ֲ�
*@param		uint8_t Sub_Index  ��������
*@param		uint32_t Mot_Data��Ҫ���͵�32bits���ݡ�			
*@retval	None
*@par
*****************************************************************************
*/
static void Faulhaber_CAN1_SDOSend_Mot(uint32_t Mot_StdId,uint8_t WriteODEntries,uint16_t Mot_CMD_Index,uint8_t Sub_Index,uint32_t Mot_Data)
{
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;			//��׼֡
	hcan1.pTxMsg->StdId = Mot_StdId;		//���ͱ�׼֡ID ����:0x201
	hcan1.pTxMsg->Data[0]=WriteODEntries;									//��������λ����	
	hcan1.pTxMsg->Data[1]=Mot_CMD_Index&0x00ff;						//��λ��ǰ
	hcan1.pTxMsg->Data[2]=Mot_CMD_Index>>8;								//��λ�ں�
	hcan1.pTxMsg->Data[3]=Sub_Index;											//��������
	hcan1.pTxMsg->Data[4]=Mot_Data;
	hcan1.pTxMsg->Data[5]=Mot_Data>>8;
	hcan1.pTxMsg->Data[6]=Mot_Data>>16;
	hcan1.pTxMsg->Data[7]=Mot_Data>>24;
	hcan1.pTxMsg->DLC =8;																	//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);	
}

/*
*****************************************************************************
*@brief		CAN1 SDO Э�鷢�����ݺ���,���з���
*@param		uint32_t Mot_StdId��Specifies the standard identifier.
					This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF
*@param		uint8_t WriteODEntries ��������λdata bytes
*@param		uint16_t Mot_CMD_Index����������������ֵ�ָ�������յ���������ֲ�
*@param		uint8_t Sub_Index  ��������
*@param		uint32_t Mot_Data��Ҫ���͵�32bits���ݡ�	
*@par
*****************************************************************************
*/
void CAN1_DATA_SDOSend(uint32_t Mot_StdId,uint8_t WriteODEntries,uint16_t Mot_CMD_Index,uint8_t Sub_Index,uint32_t Mot_Data)
{	
	CAN1TransmitMessage2Typedef MotCtrl_Data;
	MotCtrl_Data.CANTransmitID=Mot_StdId;
	MotCtrl_Data.CANTransmit_DataBytes=WriteODEntries;
	MotCtrl_Data.CANTransmit_CMDIndex=Mot_CMD_Index;
	MotCtrl_Data.CANTransmit_SubIndex=Sub_Index;
	MotCtrl_Data.CANTransmitData=Mot_Data;
	xQueueSend(CAN1TransmitQueue2Handle,&MotCtrl_Data,(TickType_t)0);
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��SDOЭ��ʹ�ܵ��,���з��ͣ��ں����������
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_SDOEnable_Mot(uint8_t Mot_Node)
{
//	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_WriteODEntries2,FahlhaberMot_CIA402_Controlword,0,(uint32_t)FaulhaberMot_FaultReset);
	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_WriteODEntries2,FahlhaberMot_CIA402_Controlword,0,(uint32_t)FaulhaberMot_ShutDown);
	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_WriteODEntries2,FahlhaberMot_CIA402_Controlword,0,(uint32_t)FaulhaberMot_SwitchOn);
	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_WriteODEntries2,FahlhaberMot_CIA402_Controlword,0,(uint32_t)FaulhaberMot_EnableOperation);
	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_WriteODEntries1,FahlhaberMot_CIA402_OperationModes,0,(uint32_t)FaulhaberMot_VelocityControl);
//	osDelay(10);			//ͨ�����Ƚ��뷢�ͺ�������ֹ���ж���
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��SDOЭ��ʧ�ܵ��,���з���
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_SDODisable_Mot(uint8_t Mot_Node)
{
	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_WriteODEntries2,FahlhaberMot_CIA402_Controlword,0,(uint32_t)FaulhaberMot_SwitchOffOutputStage);
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��SDOЭ�����õ������ģʽ,���з���
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@param		uint32_t Mot_OperatingMode���������ģʽ��
					0:Controller not activated,1:Position control,3:Velocity control,6:Homing,8:cycling synchronous position
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_SDOSet_Mot_OperatingMode(uint8_t Mot_Node,uint32_t Mot_OperatingMode)
{
	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_WriteODEntries1,FahlhaberMot_CIA402_OperationModes,0,Mot_OperatingMode);
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��SDOЭ�����õ���ٶ�,���з���
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@param		uint32_t MotVelocity������ٶ�������,�����ø���������ʽ
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_SDOSet_MotVelocity(uint8_t Mot_Node,uint32_t MotVelocity)
{
	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_WriteODEntries4,FahlhaberMot_CIA402_TargetVelocity,0,MotVelocity);
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��SDOЭ�������������ǰת��,���з���
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_SDORequest_Mot_CurrentValue(uint8_t Mot_Node)
{
	CAN1_DATA_SDOSend(FaulhaberMotSDORxID+Mot_Node,FaulhaberMot_ReadODEntries,FahlhaberMot_CIA402_VelocityActualSensorValue,0,0);
}

void CAN1_Transmit3Task(void const * argument)
{
  /* USER CODE BEGIN CAN1_Transmit3Task */
  /* Infinite loop */
	CAN1TransmitMessage3Typedef MotCtrl_Data;
	for(;;)
  {
		if(xQueueReceive(CAN1TransmitQueue3Handle,&MotCtrl_Data,(TickType_t)0)==pdTRUE){
			Faulhaber_CAN1_PDO2Send_Mot(MotCtrl_Data.CANTransmitID,MotCtrl_Data.CANTransmit_CMD,MotCtrl_Data.CANTransmitData);
		}
		osDelay(1);
  }
  /* USER CODE END CAN1_Transmit3Task */
}

void CAN1_Transmit1Task(void const * argument)
{
  /* USER CODE BEGIN CAN1_Transmit1Task */
  /* Infinite loop */
	CAN1TransmitMessage1Typedef MotCtrl_Data;
	for(;;)
  {
		if(xQueueReceive(CAN1TransmitQueue1Handle,&MotCtrl_Data,(TickType_t)0)==pdTRUE){
			switch(MotCtrl_Data.CANTransmitID)
			{
				case FaulhaberMotNMTID:Faulhaber_CAN1_NMTSend_Mot(FaulhaberMotNMTID,MotCtrl_Data.CANTransmitData[0],MotCtrl_Data.CANTransmitData[1]) ;
					break;
				case FaulhaberMotBootUpID:Faulhaber_CAN1_BootUpSend_Mot(FaulhaberMotBootUpID,MotCtrl_Data.CANTransmitData[0],MotCtrl_Data.CANTransmitData[1]);
					break;
				default:
					break;			
			}
		}
		osDelay(1);
  }
  /* USER CODE END CAN1_Transmit1Task */
}

void CAN1_Transmit2Task(void const * argument)
{
  /* USER CODE BEGIN CAN1_Transmit2Task */
  /* Infinite loop */
	CAN1TransmitMessage2Typedef MotCtrl_Data;
	for(;;)
  {
		if(xQueueReceive(CAN1TransmitQueue2Handle,&MotCtrl_Data,(TickType_t)0)==pdTRUE){
			Faulhaber_CAN1_SDOSend_Mot(MotCtrl_Data.CANTransmitID,MotCtrl_Data.CANTransmit_DataBytes,MotCtrl_Data.CANTransmit_CMDIndex,MotCtrl_Data.CANTransmit_SubIndex,MotCtrl_Data.CANTransmitData);
		}
		osDelay(1);
  }
  /* USER CODE END CAN_Transmit2Task */
}

/*
*****************************************************************************
*@brief		���㲹�뺯��
*@param		float data		��������
*@retval	uint32_t complement32  �����������
*@par
*****************************************************************************
*/
uint32_t Calculate_Complement32(float data)
{
	uint32_t complement32=0;
	if(data>=0){
		if(data>2147483647){
			data=2147483647;	
		}		
		complement32=(uint32_t)data;				
	}
	else{
		if(data<-2147483648){
			data=-2147483648;	
		}
		complement32=((uint32_t)(-1*data)^0xFFFFFFFF)+1;			
	}
	return	complement32;
}

/*
*****************************************************************************
*@brief		��֫�ֵ���ٶȴ���
*@param		float TargetSpeed��������λ���ĵ���ٶ�
*@retval	None
*@par
*****************************************************************************
*/
uint32_t Faulhaber_Mot_SpeedProcess(float Input_TargetSpeed)
{
	uint32_t Output_TargetSpeed=0;	
	Output_TargetSpeed=Calculate_Complement32(Input_TargetSpeed*100);	
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��PDO1Э�鷢�����ݸ��������������ͨ�����У�ֱ�ӷ��ͣ�
*@param		uint16_t PDO1_Mot_CMD��PDO1Э���е����������
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_PDO1Send_Mot(uint16_t PDO1_Mot_CMD,uint8_t Mot_Node)
{
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;										//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;											//��׼֡
	hcan1.pTxMsg->StdId = FaulhaberMotPDO1RxID+Mot_Node;		//���ͱ�׼֡ID uint16t ����:0x201
	hcan1.pTxMsg->Data[0]=PDO1_Mot_CMD;									//PDO1Э���е����������,�Ͱ�λ
	hcan1.pTxMsg->Data[1]=PDO1_Mot_CMD>>8;							//PDO1Э���е����������,�߰�λ
	hcan1.pTxMsg->DLC =2;																//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);	
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��PDO1Э��ʹ�ܵ������ͨ�����У�ֱ�ӷ��ͣ�
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_PDO1Enable_Mot(uint8_t Mot_Node)
{
	Faulhaber_CAN1_PDO1Send_Mot(FaulhaberMot_FaultReset,Mot_Node);
	Faulhaber_CAN1_PDO1Send_Mot(FaulhaberMot_ShutDown,Mot_Node);
	Faulhaber_CAN1_PDO1Send_Mot(FaulhaberMot_SwitchOn,Mot_Node);
	Faulhaber_CAN1_PDO1Send_Mot(FaulhaberMot_EnableOperation,Mot_Node);
	
	//����Ϊ�ٶ�ģʽ
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
	hcan1.pTxMsg->IDE = CAN_ID_STD;			//��׼֡
	hcan1.pTxMsg->StdId = FaulhaberMotSDORxID+Mot_Node;		//���ͱ�׼֡ID uint16t ����:0x201
	hcan1.pTxMsg->Data[0]=FaulhaberMot_WriteODEntries1;		//��������λ����	
	hcan1.pTxMsg->Data[1]=0x60;						//��λ��ǰ
	hcan1.pTxMsg->Data[2]=0x60;								//��λ�ں�
	hcan1.pTxMsg->Data[3]=0;
	hcan1.pTxMsg->Data[4]=0x03;
	hcan1.pTxMsg->Data[5]=0;
	hcan1.pTxMsg->Data[6]=0;
	hcan1.pTxMsg->Data[7]=0;
	hcan1.pTxMsg->DLC =8;																	//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan1,10);	
}

/*
*****************************************************************************
*@brief		��֫�ֿ�����ͨ��PDO1Э��ʧ�ܵ������ͨ�����У�ֱ�ӷ��ͣ�
*@param		uint8_t Mot_Node������������ڵ��ţ�1��127��һ�ο���һ�����
*@retval	None
*@par
*****************************************************************************
*/
void Faulhaber_CAN1_PDO1Disable_Mot(uint8_t Mot_Node)
{
	Faulhaber_CAN1_PDO1Send_Mot(FaulhaberMot_FaultReset,Mot_Node);
}

/*
*****************************************************************************
*@brief		ȡֵ����������Faulhaber��������
*@param		Faulhaber_FeedbackData_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_Faulhaber_FeedbackData(Faulhaber_FeedbackData_n name_num,void*extern_data)//ȡֵ����������Faulhaber��������,@Faulhaber_FeedbackData_n
{
	switch(name_num)
	{
		case ThumbJointMot_AutoModeSpeed_n:
			*(float*)extern_data=Faulhaber_FeedbackData.ThumbJointMot_AutoModeSpeed;
			break;
		case ThumbSwingMot_AutoModeSpeed_n:
			*(float*)extern_data=Faulhaber_FeedbackData.ThumbSwingMot_AutoModeSpeed;
			break;
		case IndexFingerSwingMot_AutoModeSpeed_n:
			*(float*)extern_data=Faulhaber_FeedbackData.IndexFingerSwingMot_AutoModeSpeed;
			break;
		case MiddleFingerSwingMot_AutoModeSpeed_n:
			*(float*)extern_data=Faulhaber_FeedbackData.MiddleFingerSwingMot_AutoModeSpeed;
			break;
		case RingFingerSwingMot_AutoModeSpeed_n:
			*(float*)extern_data=Faulhaber_FeedbackData.RingFingerSwingMot_AutoModeSpeed;
			break;
		case LittleFingerSwingMot_AutoModeSpeed_n:
			*(float*)extern_data=Faulhaber_FeedbackData.LittleFingerSwingMot_AutoModeSpeed;
			break;
		case ThumbJointMot_Current_n:
			*(uint16_t*)extern_data=Faulhaber_FeedbackData.ThumbJointMot_Current;
			break;
		case ThumbSwingMot_Current_n:
			*(uint16_t*)extern_data=Faulhaber_FeedbackData.ThumbSwingMot_Current;
			break;
		case IndexFingerSwingMot_Current_n:
			*(uint16_t*)extern_data=Faulhaber_FeedbackData.IndexFingerSwingMot_Current;
			break;
		case MiddleFingerSwingMot_Current_n:
			*(uint16_t*)extern_data=Faulhaber_FeedbackData.MiddleFingerSwingMot_Current;
			break;
		case RingFingerSwingMot_Current_n:
			*(uint16_t*)extern_data=Faulhaber_FeedbackData.RingFingerSwingMot_Current;
			break;
		case LittleFingerSwingMot_Current_n:
			*(uint16_t*)extern_data=Faulhaber_FeedbackData.LittleFingerSwingMot_Current;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�Faulhaber��������
*@param		Faulhaber_FeedbackData_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_Faulhaber_FeedbackData(Faulhaber_FeedbackData_n name_num,void*extern_data)//�޸�Faulhaber��������,@Faulhaber_FeedbackData_n
{
	switch(name_num)
	{
		case ThumbJointMot_AutoModeSpeed_n:
			Faulhaber_FeedbackData.ThumbJointMot_AutoModeSpeed=*(float*)extern_data;
			break;
		case ThumbSwingMot_AutoModeSpeed_n:
			Faulhaber_FeedbackData.ThumbSwingMot_AutoModeSpeed=*(float*)extern_data;
			break;
		case IndexFingerSwingMot_AutoModeSpeed_n:
			Faulhaber_FeedbackData.IndexFingerSwingMot_AutoModeSpeed=*(float*)extern_data;
			break;
		case MiddleFingerSwingMot_AutoModeSpeed_n:
			Faulhaber_FeedbackData.MiddleFingerSwingMot_AutoModeSpeed=*(float*)extern_data;
			break;
		case RingFingerSwingMot_AutoModeSpeed_n:
			Faulhaber_FeedbackData.RingFingerSwingMot_AutoModeSpeed=*(float*)extern_data;
			break;
		case LittleFingerSwingMot_AutoModeSpeed_n:
			Faulhaber_FeedbackData.LittleFingerSwingMot_AutoModeSpeed=*(float*)extern_data;
			break;
		case ThumbJointMot_Current_n:
			Faulhaber_FeedbackData.ThumbJointMot_Current=*(uint16_t*)extern_data;
			break;
		case ThumbSwingMot_Current_n:
			Faulhaber_FeedbackData.ThumbSwingMot_Current=*(uint16_t*)extern_data;
			break;
		case IndexFingerSwingMot_Current_n:
			Faulhaber_FeedbackData.IndexFingerSwingMot_Current=*(uint16_t*)extern_data;
			break;
		case MiddleFingerSwingMot_Current_n:
			Faulhaber_FeedbackData.MiddleFingerSwingMot_Current=*(uint16_t*)extern_data;
			break;
		case RingFingerSwingMot_Current_n:
			Faulhaber_FeedbackData.RingFingerSwingMot_Current=*(uint16_t*)extern_data;
			break;
		case LittleFingerSwingMot_Current_n:
			Faulhaber_FeedbackData.LittleFingerSwingMot_Current=*(uint16_t*)extern_data;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ����������Faulhaber״̬����
*@param		Faulhaber_StatusData_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_Faulhaber_StatusData(Faulhaber_StatusData_n name_num,void*extern_data)//ȡֵ����������Faulhaber״̬����,@Faulhaber_StatusData_n
{
	switch(name_num)
	{
		case ThumbJointMot_NMT_Status_n:
			*(uint16_t*)extern_data=Faulhaber_StatusData.ThumbJointMot_NMT_Status;
			break;
		case ThumbSwingMot_NMT_Status_n:
			*(uint16_t*)extern_data=Faulhaber_StatusData.ThumbSwingMot_NMT_Status;
			break;
		case IndexFingerSwingMot_NMT_Status_n:
			*(uint16_t*)extern_data=Faulhaber_StatusData.IndexFingerSwingMot_NMT_Status;
			break;
		case MiddleFingerSwingMot_NMT_Status_n:
			*(uint16_t*)extern_data=Faulhaber_StatusData.MiddleFingerSwingMot_NMT_Status;
			break;
		case RingFingerSwingMot_NMT_Status_n:
			*(uint16_t*)extern_data=Faulhaber_StatusData.RingFingerSwingMot_NMT_Status;
			break;
		case LittleFingerSwingMot_NMT_Status_n:
			*(uint16_t*)extern_data=Faulhaber_StatusData.LittleFingerSwingMot_NMT_Status;
			break;
		case ThumbJointMot_Bootup_Status_n:
			*(uint8_t*)extern_data=Faulhaber_StatusData.ThumbJointMot_Bootup_Status;
			break;
		case ThumbSwingMot_Bootup_Status_n:
			*(uint8_t*)extern_data=Faulhaber_StatusData.ThumbSwingMot_Bootup_Status;
			break;
		case IndexFingerSwingMot_Bootup_Status_n:
			*(uint8_t*)extern_data=Faulhaber_StatusData.IndexFingerSwingMot_Bootup_Status;
			break;
		case MiddleFingerSwingMot_Bootup_Status_n:
			*(uint8_t*)extern_data=Faulhaber_StatusData.MiddleFingerSwingMot_Bootup_Status;
			break;
		case RingFingerSwingMot_Bootup_Status_n:
			*(uint8_t*)extern_data=Faulhaber_StatusData.RingFingerSwingMot_Bootup_Status;
			break;
		case LittleFingerSwingMot_Bootup_Status_n:
			*(uint8_t*)extern_data=Faulhaber_StatusData.LittleFingerSwingMot_Bootup_Status;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�Faulhaber״̬����
*@param		Faulhaber_StatusData_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_Faulhaber_StatusData(Faulhaber_StatusData_n name_num,void*extern_data)//ȡֵ����������Faulhaber״̬����,@Faulhaber_StatusData_n
{
	switch(name_num)
	{
		case ThumbJointMot_NMT_Status_n:
			Faulhaber_StatusData.ThumbJointMot_NMT_Status=*(uint16_t*)extern_data;
			break;
		case ThumbSwingMot_NMT_Status_n:
			Faulhaber_StatusData.ThumbSwingMot_NMT_Status=*(uint16_t*)extern_data;
			break;
		case IndexFingerSwingMot_NMT_Status_n:
			Faulhaber_StatusData.IndexFingerSwingMot_NMT_Status=*(uint16_t*)extern_data;
			break;
		case MiddleFingerSwingMot_NMT_Status_n:
			Faulhaber_StatusData.MiddleFingerSwingMot_NMT_Status=*(uint16_t*)extern_data;
			break;
		case RingFingerSwingMot_NMT_Status_n:
			Faulhaber_StatusData.RingFingerSwingMot_NMT_Status=*(uint16_t*)extern_data;
			break;
		case LittleFingerSwingMot_NMT_Status_n:
			Faulhaber_StatusData.LittleFingerSwingMot_NMT_Status=*(uint16_t*)extern_data;
			break;
		case ThumbJointMot_Bootup_Status_n:
			Faulhaber_StatusData.ThumbJointMot_Bootup_Status=*(uint8_t*)extern_data;
			break;
		case ThumbSwingMot_Bootup_Status_n:
			Faulhaber_StatusData.ThumbSwingMot_Bootup_Status=*(uint8_t*)extern_data;
			break;
		case IndexFingerSwingMot_Bootup_Status_n:
			Faulhaber_StatusData.IndexFingerSwingMot_Bootup_Status=*(uint8_t*)extern_data;
			break;
		case MiddleFingerSwingMot_Bootup_Status_n:
			Faulhaber_StatusData.MiddleFingerSwingMot_Bootup_Status=*(uint8_t*)extern_data;
			break;
		case RingFingerSwingMot_Bootup_Status_n:
			Faulhaber_StatusData.RingFingerSwingMot_Bootup_Status=*(uint8_t*)extern_data;
			break;
		case LittleFingerSwingMot_Bootup_Status_n:
			Faulhaber_StatusData.LittleFingerSwingMot_Bootup_Status=*(uint8_t*)extern_data;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ���������˼�֫����Ӵ���������
*@param		ProstheticHand_SensorData_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_ProstheticHand_SensorData(ProstheticHand_SensorData_n name_num,void*extern_data)//ȡֵ���������˼�֫����Ӵ���������,@ProstheticHand_SensorData_n
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
*@brief		�޸ļ�֫����Ӵ���������
*@param		ProstheticHand_SensorData_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_ProstheticHand_SensorData(ProstheticHand_SensorData_n name_num,void*extern_data)//�޸ļ�֫����Ӵ���������,@ProstheticHand_SensorData_n
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
