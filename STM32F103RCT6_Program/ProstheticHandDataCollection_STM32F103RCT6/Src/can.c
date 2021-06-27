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

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

union Integer_Floating{																	
	volatile uint8_t Integer_Data[4];
	volatile float Floating_Data;
};//������ת��������


osThreadId CAN_SendQueueTaskHandle;
osThreadId CAN_TransmitTaskHandle;

osMessageQId CANTransmitQueueHandle;									//CAN1�������ݶ���
osThreadId Get_CANTransmitQueueHandle(void){
	return CANTransmitQueueHandle;
}

TimerHandle_t CAN_10PeriodicSendTimer_Handle;//�����ʱ�����
TimerHandle_t Get_CAN_10PeriodicSendTimer_Handle(void){
	return CAN_10PeriodicSendTimer_Handle;
}
void CAN_10PeriodicSendCallback(TimerHandle_t xTimer);//��ʱ���ص�����

void CAN_SendQueueTask(void const * argument);
void CAN_TransmitTask(void const * argument);


static void CAN_FREERTOS_Init(void) 
{	
	/* definition and creation of CAN_SendQueueTask */
	osThreadDef(CAN_SendQueueTask,CAN_SendQueueTask, osPriorityNormal, 0, 128);
	CAN_SendQueueTaskHandle = osThreadCreate(osThread(CAN_SendQueueTask), NULL);	
	/* definition and creation of CAN_TransmitTask */
	osThreadDef(CAN_TransmitTask,CAN_TransmitTask, osPriorityNormal, 0, 128);
	CAN_TransmitTaskHandle = osThreadCreate(osThread(CAN_TransmitTask), NULL);	
	/* Create the queue(s) */
	/* definition and creation of CAN1TransmitQueue */
  osMessageQDef(CANTransmitQueue,10, CANTransmitMessageTypedef);
  CANTransmitQueueHandle = osMessageCreate(osMessageQ(CANTransmitQueue), NULL);		
	/*definition and creation of CAN_100PeriodicSendTimer_Handle */ 				//���ڶ�ʱ��������10ms(10��ʱ�ӽ���)������ģʽ //ID:1		      
	CAN_10PeriodicSendTimer_Handle=xTimerCreate((const char*)"CAN_100PeriodicSend",
																			(TickType_t)10,
																			(UBaseType_t)pdTRUE,
																			(void*)1,
																			(TimerCallbackFunction_t)CAN_10PeriodicSendCallback); 
}


CAN_HandleTypeDef hcan;

CanTxMsgTypeDef s1_TxMsg; //CAN1������Ϣ
CanRxMsgTypeDef s1_RxMsg; //CAN1������Ϣ

void APP_CAN_Config(void);

/* CAN init function */
void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_15TQ;
  hcan.Init.BS2 = CAN_BS2_8TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	APP_CAN_Config();
	CAN_FREERTOS_Init();
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
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = STM32_CAN1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(STM32_CAN1_RX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = STM32_CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STM32_CAN1_TX_GPIO_Port, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
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
  
    /**CAN GPIO Configuration    
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(GPIOA, STM32_CAN1_RX_Pin|STM32_CAN1_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/* USER CODE BEGIN 1 */
void APP_CAN_Config(void){
	CAN_FilterConfTypeDef sFilterConfig;	
	//���ù�����	
	//configure the filter0
	sFilterConfig.FilterIdHigh=(((uint32_t)0x000<<21)&0xffff0000)>>16;					//32λID
	sFilterConfig.FilterIdLow=(((uint32_t)0x000<<21)|CAN_ID_STD|CAN_RTR_DATA)&0xffff;
	sFilterConfig.FilterMaskIdHigh=0x0000;
	sFilterConfig.FilterMaskIdLow=0x0000;		
	sFilterConfig.FilterNumber=0;								//������0
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0;
	sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterActivation=ENABLE;
	sFilterConfig.BankNumber=1;
	HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
	//initialize the msg pointer
	hcan	.pTxMsg=&s1_TxMsg;
	hcan	.pRxMsg=&s1_RxMsg;
	//configuer Rx message
	hcan .pRxMsg->StdId=0x7FF	;//Max_Data = 0x7FF			
	hcan .pRxMsg->IDE=CAN_ID_STD;
	hcan .pRxMsg->RTR=CAN_RTR_DATA;		
//	hcan1 .pRxMsg->DLC=8;
	//configuer Tx message
	hcan .pTxMsg->StdId=0x000;//Max_Data = x7FF	
	hcan .pTxMsg->ExtId=0x0000;//Max_Data = 0x1FFFFFFF
	hcan .pTxMsg->IDE=CAN_ID_STD;
	hcan .pTxMsg->RTR=CAN_RTR_DATA;
	hcan .pTxMsg->DLC=8;			
}

/*
*****************************************************************************
*@brief		CAN���ͺ���,ʹ��CAN���������ͱ��ģ��������ñ���ID,
					���ͱ��������׵�ַ���������ݳ��ȣ�����һ�α�����������һ�����ݡ�	
*@param		uint32_t ID��32λID������0x7F0
*@param		uint8_t *pData�����ͱ��������׵�ַ�����緢������Ϊ
					CharBuff[8]={0,1,2,3,4,5,6,7},�����ڶ�������ΪCharBuff���ɡ�					
*@param		uint8_t Size,�������ݵĳ��ȣ�ȡֵӦΪ1~8��		
*@retval	None
*@par
*****************************************************************************
*/
void Collection_CAN_Send(uint32_t ID,uint8_t *pData, uint8_t Size){	//
	uint8_t a=0;
	hcan.pTxMsg->RTR = CAN_RTR_DATA;		//����֡
	hcan.pTxMsg->IDE = CAN_ID_STD;			//��׼֡
	hcan.pTxMsg->StdId = ID;						//���ͱ�׼֡ ID uint32_t ����:0x7E0
	for(a=0;a<Size;a++){
		hcan.pTxMsg->Data[a]=*(pData+a); 
	}
	hcan.pTxMsg->DLC = Size;						//�������ݳ��� 1-8
	HAL_CAN_Transmit(&hcan,10);					//CAN1
}


/*
*****************************************************************************
*@brief		FSR1��FSR2 ADCINFO����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/

static void FSR1AndFSR2_ADCINFO_Protocol_Send(uint32_t Protocol_ID)
{
	union Integer_Floating FSR1_PressureData,	FSR2_PressureData;
	CANTransmitMessageTypedef CANTransmitMessage;	
	CANTransmitMessage.CANTransmitID=Protocol_ID;	
	Get_ProstheticHand_SensorData(FSR1_NormalPressure_n,&FSR1_PressureData.Floating_Data);//ȡֵ����������FSR1ѹ������
	Get_ProstheticHand_SensorData(FSR2_NormalPressure_n,&FSR2_PressureData.Floating_Data);//ȡֵ����������FSR2ѹ������
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		CANTransmitMessage.CANTransmitData[i]=FSR1_PressureData.Integer_Data[i];
		CANTransmitMessage.CANTransmitData[i+4]=FSR2_PressureData.Integer_Data[i];
	}
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		FSR3��FSR4 ADCINFO����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void FSR3AndFSR4_ADCINFO_Protocol_Send(uint32_t Protocol_ID)
{
	union Integer_Floating FSR3_PressureData,FSR4_PressureData;
	CANTransmitMessageTypedef CANTransmitMessage;	
	CANTransmitMessage.CANTransmitID=Protocol_ID;	
	Get_ProstheticHand_SensorData(FSR3_NormalPressure_n,&FSR3_PressureData.Floating_Data);//ȡֵ����������FSR3ѹ������
	Get_ProstheticHand_SensorData(FSR4_NormalPressure_n,&FSR4_PressureData.Floating_Data);//ȡֵ����������FSR4ѹ������
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		CANTransmitMessage.CANTransmitData[i]=FSR3_PressureData.Integer_Data[i];
		CANTransmitMessage.CANTransmitData[i+4]=FSR4_PressureData.Integer_Data[i];
	}
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		FSR5��Bending1 ADCINFO����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void FSR5AndBending1_ADCINFO_Protocol_Send(uint32_t Protocol_ID)
{
	union Integer_Floating FSR5_PressureData,Thumb_BendingData;
	CANTransmitMessageTypedef CANTransmitMessage;	
	CANTransmitMessage.CANTransmitID=Protocol_ID;	
	Get_ProstheticHand_SensorData(FSR5_NormalPressure_n,&FSR5_PressureData.Floating_Data);//ȡֵ����������FSR5ѹ������
	Get_ProstheticHand_SensorData(Thumb_Bending_n,&Thumb_BendingData.Floating_Data);			//ȡֵ����������Bending1�����ź�����
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		CANTransmitMessage.CANTransmitData[i]=FSR5_PressureData.Integer_Data[i];
		CANTransmitMessage.CANTransmitData[i+4]=Thumb_BendingData.Integer_Data[i];
	}
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		Bending2��Bending3 ADCINFO����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void Bending2AndBending3_ADCINFO_Protocol_Send(uint32_t Protocol_ID)
{
	union Integer_Floating IndexFinger_BendingData,MiddleFinger_BendingData;
	CANTransmitMessageTypedef CANTransmitMessage;	
	CANTransmitMessage.CANTransmitID=Protocol_ID;	
	Get_ProstheticHand_SensorData(IndexFinger_Bending_n,&IndexFinger_BendingData.Floating_Data);	//ȡֵ����������Bending2�����ź�����
	Get_ProstheticHand_SensorData(MiddleFinger_Bending_n,&MiddleFinger_BendingData.Floating_Data);//ȡֵ����������Bending3�����ź�����
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		CANTransmitMessage.CANTransmitData[i]=IndexFinger_BendingData.Integer_Data[i];
		CANTransmitMessage.CANTransmitData[i+4]=MiddleFinger_BendingData.Integer_Data[i];
	}
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		Bending4��Bending5 ADCINFO����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void Bending4AndBending5_ADCINFO_Protocol_Send(uint32_t Protocol_ID)
{
	union Integer_Floating RingFinger_BendingData,LittleFinger_BendingData;
	CANTransmitMessageTypedef CANTransmitMessage;	
	CANTransmitMessage.CANTransmitID=Protocol_ID;	
	Get_ProstheticHand_SensorData(RingFinger_Bending_n,&RingFinger_BendingData.Floating_Data);		//ȡֵ����������Bending4�����ź�����
	Get_ProstheticHand_SensorData(LittleFinger_Bending_n,&LittleFinger_BendingData.Floating_Data);//ȡֵ����������Bending5�����ź�����
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		CANTransmitMessage.CANTransmitData[i]=RingFinger_BendingData.Integer_Data[i];
		CANTransmitMessage.CANTransmitData[i+4]=LittleFinger_BendingData.Integer_Data[i];
	}
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		ADC_Backup1��ADC_Backup2 ADCINFO����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void ADC_Backup1AndADC_Backup2_ADCINFO_Protocol_Send(uint32_t Protocol_ID)
{
	union Integer_Floating ADC_Backup1_Data,ADC_Backup2_Data;
	CANTransmitMessageTypedef CANTransmitMessage;	
	CANTransmitMessage.CANTransmitID=Protocol_ID;	
	Get_ProstheticHand_SensorData(ADC_Backup1_n,&ADC_Backup1_Data.Floating_Data);//ȡֵ���������˱���ADC1������
	Get_ProstheticHand_SensorData(ADC_Backup2_n,&ADC_Backup2_Data.Floating_Data);//ȡֵ���������˱���ADC2������
	uint8_t i=0;
	for(i=0;i<4;i++)
	{
		CANTransmitMessage.CANTransmitData[i]=ADC_Backup1_Data.Integer_Data[i];
		CANTransmitMessage.CANTransmitData[i+4]=ADC_Backup2_Data.Integer_Data[i];
	}
	xQueueSendToFront(CANTransmitQueueHandle,&CANTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		Collection_Controller_ADCINFO����Э�黯����
*@param		uint32_t Protocol_ID��Э��ID
*@retval	None
*@par
*****************************************************************************
*/
static void Collection_Controller_ADCINFO_Protocol_Send(uint32_t Protocol_ID)
{
	if(Protocol_ID==Collection_Controller_INFO0_ID){		
		FSR1AndFSR2_ADCINFO_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==Collection_Controller_INFO1_ID){		
		FSR3AndFSR4_ADCINFO_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==Collection_Controller_INFO2_ID){		
		FSR5AndBending1_ADCINFO_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==Collection_Controller_INFO3_ID){		
		Bending2AndBending3_ADCINFO_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==Collection_Controller_INFO4_ID){		
		Bending4AndBending5_ADCINFO_Protocol_Send(Protocol_ID);
	}
	else if(Protocol_ID==Collection_Controller_INFO5_ID){		
		ADC_Backup1AndADC_Backup2_ADCINFO_Protocol_Send(Protocol_ID);
	}
}

void CAN_10PeriodicSendCallback(TimerHandle_t xTimer)
{
//	Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO0_ID);//Collection���͸�Controller����0��FSR1��FSR2ѹ������
//	Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO1_ID);//Collection���͸�Controller����1��FSR3��FSR4ѹ������
//	Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO2_ID);//Collection���͸�Controller����2��FSR5��Bending1��������
//	Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO3_ID);//Collection���͸�Controller����3��Bending2��Bending3��������
//	Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO4_ID);//Collection���͸�Controller����4��Bending4��Bending5��������
//	Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO5_ID);//Collection���͸�Controller����5������ADC1��ADC2������
}

void CAN_SendQueueTask(void const * argument)
{
  /* USER CODE BEGIN CAN_SendQueueTask */
  /* Infinite loop */
	uint8_t i=0;
	for(;;)
  {
		for(i=0;i<6;i++)
		{
			switch(i)
			{
				case 0:Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO0_ID);//Collection���͸�Controller����0��FSR1��FSR2ѹ������
					break;
				case 1:Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO1_ID);//Collection���͸�Controller����1��FSR3��FSR4ѹ������
					break;
				case 2:Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO2_ID);//Collection���͸�Controller����2��FSR5��Bending1��������
					break;
				case 3:Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO3_ID);//Collection���͸�Controller����3��Bending2��Bending3��������
					break;
				case 4:Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO4_ID);//Collection���͸�Controller����4��Bending4��Bending5��������
					break;
				case 5:Collection_Controller_ADCINFO_Protocol_Send(Collection_Controller_INFO5_ID);//Collection���͸�Controller����5������ADC1��ADC2������
					break;
				default: 
					break;
			}
			osDelay(1);
		}		
		osDelay(1);
  }
  /* USER CODE END CAN_SendQueueTask */
}
	CANTransmitMessageTypedef Collection_Data;
void CAN_TransmitTask(void const * argument)
{
  /* USER CODE BEGIN CAN_TransmitTask */
  /* Infinite loop */
	if(Get_CAN_10PeriodicSendTimer_Handle()!=NULL){
		xTimerReset(Get_CAN_10PeriodicSendTimer_Handle(),10);			//��λ���ڶ�ʱ��
		xTimerStart(Get_CAN_10PeriodicSendTimer_Handle(),10);			//�������ڶ�ʱ����10ms������CAN���Ͷ����з�������
	}
//	CANTransmitMessageTypedef Collection_Data;
	for(;;)
  {
		if(xQueueReceive(CANTransmitQueueHandle,&Collection_Data,(TickType_t)0)==pdTRUE){
			Collection_CAN_Send(Collection_Data.CANTransmitID,Collection_Data.CANTransmitData, 8);
		}
		osDelay(1);
  }
  /* USER CODE END CAN_TransmitTask */
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
