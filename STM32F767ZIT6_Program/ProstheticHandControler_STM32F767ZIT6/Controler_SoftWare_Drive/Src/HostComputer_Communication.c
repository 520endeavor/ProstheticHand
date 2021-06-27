#include "HostComputer_Communication.h"

volatile uint8_t WirelessAnalysis_Data[5]={0};											//���߽������ݻ�����
volatile uint8_t WirelessReceice_Data_OneByte=0;									//���߽������ݻ������

//Get�ֶ�ģʽĿ��ת��ʱ����Сʮ������ֹת�ٹ���
struct{
	volatile uint16_t ManualOrAutomatic_ModeSelect; 			//�ֶ�Or�Զ�ģʽѡ������
	volatile uint16_t MotorEnableOrDisable;								//���ʹ��Orʧ������
	volatile float ThumbJointMot_TargetSpeed;							//Ĵָ�źϵ��Ŀ��ת������
	volatile float ThumbSwingMot_TargetSpeed;							//Ĵָ�ڶ����Ŀ��ת������
	volatile float IndexFingerSwingMot_TargetSpeed;				//ʳָ�ڶ����Ŀ��ת������
	volatile float MiddleFingerSwingMot_TargetSpeed;			//��ָ�ڶ����Ŀ��ת������
	volatile float RingFingerSwingMot_TargetSpeed;				//����ָ�ڶ����Ŀ��ת������
	volatile float LittleFingerSwingMot_TargetSpeed;			//Сָ�ڶ����Ŀ��ת������
	volatile uint16_t PowerUp_DefaultTypesofGestures;			//�ϵ�ʱĬ��������������
	volatile uint16_t Manual_TargetTypesofGesture;				//�ֶ�ģʽĿ��������������
	volatile uint16_t Bluetooth_AT_SetMasterMode_Flag;		//ATָ������Masterģʽ��������
	volatile float Grip_Control_Parameters_P;							//ץ�տ��Ʋ���������ϵ��
	volatile float Grip_Control_Parameters_I;							//ץ�տ��Ʋ���������ϵ��	
	volatile float Grip_Control_Parameters_D;							//ץ�տ��Ʋ�����΢��ϵ��
}Transmit1AndReceive1LabviewData;//Controler����ģ��1&����ģ��1����


osThreadId Usart3_ReceiveTaskHandle;

osThreadId Wireless_ReceiveLabview_SendMemTaskHandle;
osThreadId Get_Wireless_ReceiveLabview_SendMemTaskHandle(){		//��ȡWireless_ReceiveLabview_SendMemTaskHandle�������
	return Wireless_ReceiveLabview_SendMemTaskHandle;
}

osThreadId EEPROMMemoryTaskHandle;
osThreadId EEPROMreadAndGenericDataTaskHandle;
osThreadId SendLabviewReadbackData_Period50TaskHandle;
osThreadId SendLabviewModule2Data_Period100TaskHandle;
osThreadId SendLabviewModule2Data_Period20TaskHandle;
osThreadId Wireless_Transmit1_LabviewTaskHandle;
osThreadId Wireless_Transmit2_LabviewTaskHandle;

osMessageQId EEPROMMemoryQueueHandle;																		//EEPROM�洢���ݶ���

osMessageQId WirelessTransmit1_LabviewQueueHandle;									//Wireless��������1����
osMessageQId WirelessTransmit2_LabviewQueueHandle;									//Wireless��������2����

void Usart3_ReceiveTask(void const * argument);
void Wireless_ReceiveLabview_SendMemTask(void const * argument);
void EEPROMMemoryTask(void const * argument);
void EEPROMreadAndGenericDataTask(void const * argument);
void SendLabviewReadbackData_Period50Task(void const * argument);
void SendLabviewModule2Data_Period100Task(void const * argument);
void SendLabviewModule2Data_Period20Task(void const * argument);
void Wireless_Transmit1_LabviewTask(void const * argument);
void Wireless_Transmit2_LabviewTask(void const * argument);


void Wireless_FREERTOS_Init(void)
{
	/* definition and creation of Usart3_ReceiveTask */
	osThreadDef(Usart3_ReceiveTask, Usart3_ReceiveTask, osPriorityNormal, 0, 128);
	Usart3_ReceiveTaskHandle = osThreadCreate(osThread(Usart3_ReceiveTask), NULL);
	/* definition and creation of Wireless_ReceiveData_SendMemTask */
	osThreadDef(Wireless_ReceiveLabview_SendMemTask, Wireless_ReceiveLabview_SendMemTask, osPriorityNormal, 0, 128);
	Wireless_ReceiveLabview_SendMemTaskHandle = osThreadCreate(osThread(Wireless_ReceiveLabview_SendMemTask), NULL);
	/* definition and creation of EEPROMMemoryTask */
	osThreadDef(EEPROMMemoryTask, EEPROMMemoryTask, osPriorityNormal, 0, 128);
  EEPROMMemoryTaskHandle = osThreadCreate(osThread(EEPROMMemoryTask), NULL);	
	/* definition and creation of EEPROMreadAndGenericDataTask */
	osThreadDef(EEPROMreadAndGenericDataTask, EEPROMreadAndGenericDataTask, osPriorityNormal, 0, 128);
  EEPROMreadAndGenericDataTaskHandle = osThreadCreate(osThread(EEPROMreadAndGenericDataTask), NULL);	
	/* definition and creation of SendLabviewReadbackData_Period50Task */
	osThreadDef(SendLabviewReadbackData_Period50Task, SendLabviewReadbackData_Period50Task, osPriorityNormal, 0, 128);
	SendLabviewReadbackData_Period50TaskHandle = osThreadCreate(osThread(SendLabviewReadbackData_Period50Task), NULL);
	/* definition and creation of SendLabviewModule2Data_Period100Task */
	osThreadDef(SendLabviewModule2Data_Period100Task, SendLabviewModule2Data_Period100Task, osPriorityNormal, 0, 128);
	SendLabviewModule2Data_Period100TaskHandle = osThreadCreate(osThread(SendLabviewModule2Data_Period100Task), NULL);
	/* definition and creation of SendLabviewModule2Data_Period20Task */
	osThreadDef(SendLabviewModule2Data_Period20Task, SendLabviewModule2Data_Period20Task, osPriorityNormal, 0, 128);
	SendLabviewModule2Data_Period20TaskHandle = osThreadCreate(osThread(SendLabviewModule2Data_Period20Task), NULL);
	/* definition and creation of Wireless_Transmit1_LabviewTask */
	osThreadDef(Wireless_Transmit1_LabviewTask, Wireless_Transmit1_LabviewTask, osPriorityNormal, 0, 128);
	Wireless_Transmit1_LabviewTaskHandle = osThreadCreate(osThread(Wireless_Transmit1_LabviewTask), NULL);
	/* definition and creation of Wireless_Transmit2_LabviewTask */
	osThreadDef(Wireless_Transmit2_LabviewTask, Wireless_Transmit2_LabviewTask, osPriorityNormal, 0, 128);
	Wireless_Transmit2_LabviewTaskHandle = osThreadCreate(osThread(Wireless_Transmit2_LabviewTask), NULL);
	
	/* definition and creation of EEPROMMemoryQueue */
  osMessageQDef(EEPROMMemoryQueue, Transmit1AndReceive1LabviewDataNum, EEPROMMemoryMessageTypedef);
  EEPROMMemoryQueueHandle = osMessageCreate(osMessageQ(EEPROMMemoryQueue), NULL);
	/* definition and creation of WirelessTransmit1_LabviewQueue */
  osMessageQDef(WirelessTransmit1_LabviewQueue, Transmit1AndReceive1LabviewDataNum, WirelessTransmit1_MessageTypedef);
  WirelessTransmit1_LabviewQueueHandle = osMessageCreate(osMessageQ(WirelessTransmit1_LabviewQueue), NULL);
	/* definition and creation of WirelessTransmit2_LabviewQueue */
  osMessageQDef(WirelessTransmit2_LabviewQueue, Transmit2LabviewDataNum, WirelessTransmit2_MessageTypedef);
  WirelessTransmit2_LabviewQueueHandle = osMessageCreate(osMessageQ(WirelessTransmit2_LabviewQueue), NULL);
} 

/*
*****************************************************************************
*@brief		���߽������ݽ�������
*@param		uint8_t Wireless_ReceiveData��������յ�������
*@param		uint8_t *Analysis_Data����������
*@retval	None
*@par
*****************************************************************************
*/
uint8_t Wireless_ReceiveData_Analysis(uint8_t Wireless_ReceiveData,uint8_t *Analysis_Data)
{	
	static uint8_t WirelessRxCnt = 0;
	Analysis_Data[WirelessRxCnt]=Wireless_ReceiveData;	
	if(WirelessRxCnt==0){
		if (Analysis_Data[0]!=(uint8_t)(Transmit1AndReceive1LabviewHeader>>8)){				
			WirelessRxCnt=0;
			return 0;
		}		
	}	
	if(WirelessRxCnt==1){
		if (Analysis_Data[1]!=(uint8_t)Transmit1AndReceive1LabviewHeader){
			WirelessRxCnt=0;
			return 0;
		}		
	}
	WirelessRxCnt++;
	if (WirelessRxCnt<5) {
		return 0;
	}
	else{
		WirelessRxCnt=0;
		return 1;
	}
}

/*
*****************************************************************************
*@brief		��EEPROMͶ������ ����
*@param		uint8_t *Wireless_data������Wireless���յ�������
*@param		uint8_t Memory_ID���洢ID��ͬ��λ��ͨѶЭ��ID
*@retval	None
*@par
*****************************************************************************
*/
static void SendDataToEEPROM(uint8_t *Wireless_data,uint8_t Memory_ID)
{
	EEPROMMemoryMessageTypedef EEPROMMemoryMessage;
	if(Memory_ID>=Transmit1AndReceive1Labview_StartAddress_ID&&Memory_ID<=Transmit1AndReceive1Labview_EndAddress_ID&&~(Memory_ID%2)){
		EEPROMMemoryMessage.EEPROMMemoryID=Memory_ID;
		EEPROMMemoryMessage.EEPROMMemoryData[0]=*Wireless_data;
		EEPROMMemoryMessage.EEPROMMemoryData[1]=*(Wireless_data+1);
		xQueueSend(EEPROMMemoryQueueHandle,&EEPROMMemoryMessage,(TickType_t)0 );
	}
}

/* Usart3_ReceiveTask function */
void Usart3_ReceiveTask(void const * argument)
{	
  /* USER CODE BEGIN Usart3_ReceiveTask */
  /* Infinite loop */	
	uint8_t receive_data[2];
  for(;;)
  {	
		if(HAL_UART_Receive_DMA(&huart3, &WirelessReceice_Data_OneByte, 1)==HAL_OK){
			if(Wireless_ReceiveData_Analysis(WirelessReceice_Data_OneByte,WirelessAnalysis_Data)){	
				xTaskNotifyGive(Wireless_ReceiveLabview_SendMemTaskHandle);						
			}			
		}
		osDelay(1);
  }
  /* USER CODE END Usart3_ReceiveTask */
}

/* Wireless_ReceiveLabview_SendMemTask function */
void Wireless_ReceiveLabview_SendMemTask(void const * argument)
{	
  /* USER CODE BEGIN Wireless_ReceiveLabview_SendMemTask */
  /* Infinite loop */
	HAL_UART_Receive_IT(&huart3, &WirelessReceice_Data_OneByte, 1);	
	uint8_t receive_data[2];
  for(;;)
  {	
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(WirelessAnalysis_Data[0]==(uint8_t)(Transmit1AndReceive1LabviewHeader>>8)&&WirelessAnalysis_Data[1]==(uint8_t)Transmit1AndReceive1LabviewHeader){
			receive_data[0]=WirelessAnalysis_Data[3];
			receive_data[1]=WirelessAnalysis_Data[4];
			SendDataToEEPROM(receive_data,WirelessAnalysis_Data[2]);
		}						
  }
  /* USER CODE END Wireless_ReceiveLabview_SendMemTask */
}

/* EEPROMMemoryTask function */
void EEPROMMemoryTask(void const * argument)
{	
  /* USER CODE BEGIN EEPROMMemoryTask */
  /* Infinite loop */
	if (AT24Cxx_Check()){  //�������
		/*����EEPROM������Ϣ����ʾ��*/	
	}
	EEPROMMemoryMessageTypedef EEPROMMemoryMessage;
  for(;;)
  {	
		if(xQueueReceive(EEPROMMemoryQueueHandle,&EEPROMMemoryMessage,(TickType_t)0)==pdTRUE){
			AT24Cxx_WriteOneByte(EEPROMMemoryMessage.EEPROMMemoryID,EEPROMMemoryMessage.EEPROMMemoryData[0]);
			AT24Cxx_WriteOneByte(EEPROMMemoryMessage.EEPROMMemoryID+1,EEPROMMemoryMessage.EEPROMMemoryData[1]);
			xTaskNotify(EEPROMreadAndGenericDataTaskHandle,(uint32_t)EEPROMMemoryMessage.EEPROMMemoryID,eSetValueWithOverwrite);		
		}		
    osDelay(10);
  }
  /* USER CODE END EEPROMMemoryTask */
}

/*
*****************************************************************************
*@brief		����ECU1��Ҫʹ����ֵ����ֵ����Labview
*@param		uint8_t *Read_data��EEPROM�ж���������
*@param		uint8_t Memory_ID��EEPROM�д洢���ֵ�ID��ͬ��λ��Э����ID
*@retval	None
*@par
*****************************************************************************
*/
static void DataFromLabviewGeneric(uint8_t *Read_data,uint8_t Memory_ID)
{	
	if(Memory_ID==ManualOrAutomatic_ModeSelect_ID){
		Transmit1AndReceive1LabviewData.ManualOrAutomatic_ModeSelect=(Read_data[0]<<8)+Read_data[1];
	}
	if(Memory_ID==MotorEnableOrDisable_ID){
		Transmit1AndReceive1LabviewData.MotorEnableOrDisable=(Read_data[0]<<8)+Read_data[1];
	}
		if(Memory_ID==ThumbJointMot_TargetSpeed_ID){
		Transmit1AndReceive1LabviewData.ThumbJointMot_TargetSpeed=0.001*((Read_data[0]<<8)+Read_data[1]-29286);
	}
	if(Memory_ID==ThumbSwingMot_TargetSpeed_ID){
		Transmit1AndReceive1LabviewData.ThumbSwingMot_TargetSpeed=0.001*((Read_data[0]<<8)+Read_data[1]-29286);
	}
		if(Memory_ID==IndexFingerSwingMot_TargetSpeed_ID){
		Transmit1AndReceive1LabviewData.IndexFingerSwingMot_TargetSpeed=0.001*((Read_data[0]<<8)+Read_data[1]-29286);
	}
	if(Memory_ID==MiddleFingerSwingMot_TargetSpeed_ID){
		Transmit1AndReceive1LabviewData.MiddleFingerSwingMot_TargetSpeed=0.001*((Read_data[0]<<8)+Read_data[1]-29286);
	}
		if(Memory_ID==RingFingerSwingMot_TargetSpeed_ID){
		Transmit1AndReceive1LabviewData.RingFingerSwingMot_TargetSpeed=0.001*((Read_data[0]<<8)+Read_data[1]-29286);
	}
	if(Memory_ID==LittleFingerSwingMot_TargetSpeed_ID){
		Transmit1AndReceive1LabviewData.LittleFingerSwingMot_TargetSpeed=0.001*((Read_data[0]<<8)+Read_data[1]-29286);
	}
		if(Memory_ID==PowerUp_DefaultTypesofGestures_ID){
		Transmit1AndReceive1LabviewData.PowerUp_DefaultTypesofGestures=(Read_data[0]<<8)+Read_data[1];
	}
	if(Memory_ID==Manual_TargetTypesofGesture_ID){
		Transmit1AndReceive1LabviewData.Manual_TargetTypesofGesture=(Read_data[0]<<8)+Read_data[1];
	}	
	if(Memory_ID==Bluetooth_AT_SetMasterMode_Flag_ID){
		Transmit1AndReceive1LabviewData.Bluetooth_AT_SetMasterMode_Flag=(Read_data[0]<<8)+Read_data[1];
	}	
	
	if(Memory_ID==Grip_Control_Parameters_P_ID){
		Transmit1AndReceive1LabviewData.Grip_Control_Parameters_P=0.01*((Read_data[0]<<8)+Read_data[1]-32000);
	}	
	if(Memory_ID==Grip_Control_Parameters_I_ID){
		Transmit1AndReceive1LabviewData.Grip_Control_Parameters_I=0.01*((Read_data[0]<<8)+Read_data[1]-32000);
	}	
	if(Memory_ID==Grip_Control_Parameters_D_ID){
		Transmit1AndReceive1LabviewData.Grip_Control_Parameters_D=0.01*((Read_data[0]<<8)+Read_data[1]-32000);
	}	
}

/* EEPROMreadAndGenericDataTask function */
void EEPROMreadAndGenericDataTask(void const * argument)
{	
  /* USER CODE BEGIN EEPROMreadAndGenericDataTask */
  /* Infinite loop */
	if (AT24Cxx_Check()){  //�������
		/*����EEPROM������Ϣ����ʾ��*/	
	}
	uint8_t Memory_ID;
	uint8_t Read_Data[2];
  for(;;)
  {	
		xTaskNotifyWait( 0,0xFFFFFFFF,(uint32_t*)&Memory_ID,portMAX_DELAY);
		Read_Data[0]=AT24Cxx_ReadOneByte(Memory_ID);
		Read_Data[1]=AT24Cxx_ReadOneByte(Memory_ID+1);
		DataFromLabviewGeneric(Read_Data,Memory_ID);
	}
  /* USER CODE END EEPROMreadAndGenericDataTask */
}

/*
*****************************************************************************
*@brief		ECU1����Labview�󣬻�ִ�������ݺ��������ý����õ�������Э�黯���ͣ��ɼ������������Ƿ�����
*@param		uint8_t Labview_ID��Labview�н������ݵ�ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU1_Labview_Readback_Send(uint8_t Labview_ID)
{	
	WirelessTransmit1_MessageTypedef WirelessTransmitMessage;
	WirelessTransmitMessage.WirelessTransmitID=Labview_ID;	
	if(Labview_ID==ManualOrAutomatic_ModeSelect_ID){			
		WirelessTransmitMessage.WirelessTransmitData[0]=Transmit1AndReceive1LabviewData.ManualOrAutomatic_ModeSelect>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.ManualOrAutomatic_ModeSelect;		
	}
	if(Labview_ID==MotorEnableOrDisable_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Transmit1AndReceive1LabviewData.MotorEnableOrDisable>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.MotorEnableOrDisable;		
	}
	if(Labview_ID==ThumbJointMot_TargetSpeed_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.ThumbJointMot_TargetSpeed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.ThumbJointMot_TargetSpeed*1000+29286;
	}
	if(Labview_ID==ThumbSwingMot_TargetSpeed_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.ThumbSwingMot_TargetSpeed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.ThumbSwingMot_TargetSpeed*1000+29286;
	}
	if(Labview_ID==IndexFingerSwingMot_TargetSpeed_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.IndexFingerSwingMot_TargetSpeed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.IndexFingerSwingMot_TargetSpeed*1000+29286;
	}
	if(Labview_ID==MiddleFingerSwingMot_TargetSpeed_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.MiddleFingerSwingMot_TargetSpeed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.MiddleFingerSwingMot_TargetSpeed*1000+29286;
	}
	if(Labview_ID==RingFingerSwingMot_TargetSpeed_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.RingFingerSwingMot_TargetSpeed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.RingFingerSwingMot_TargetSpeed*1000+29286;
	}
	if(Labview_ID==LittleFingerSwingMot_TargetSpeed_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.LittleFingerSwingMot_TargetSpeed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.LittleFingerSwingMot_TargetSpeed*1000+29286;
	}
	if(Labview_ID==PowerUp_DefaultTypesofGestures_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Transmit1AndReceive1LabviewData.PowerUp_DefaultTypesofGestures>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.PowerUp_DefaultTypesofGestures;
	}
	if(Labview_ID==Manual_TargetTypesofGesture_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Transmit1AndReceive1LabviewData.Manual_TargetTypesofGesture>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.Manual_TargetTypesofGesture;
	}
	if(Labview_ID==Bluetooth_AT_SetMasterMode_Flag_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=Transmit1AndReceive1LabviewData.Bluetooth_AT_SetMasterMode_Flag>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.Bluetooth_AT_SetMasterMode_Flag;
	}	
	if(Labview_ID==Grip_Control_Parameters_P_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.Grip_Control_Parameters_P*100+32000)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.Grip_Control_Parameters_P*100+32000;
	}	
	if(Labview_ID==Grip_Control_Parameters_I_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.Grip_Control_Parameters_I*100+32000)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.Grip_Control_Parameters_I*100+32000;
	}	
	if(Labview_ID==Grip_Control_Parameters_D_ID){
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Transmit1AndReceive1LabviewData.Grip_Control_Parameters_D*100+32000)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Transmit1AndReceive1LabviewData.Grip_Control_Parameters_D*100+32000;
	}	
	xQueueSend(WirelessTransmit1_LabviewQueueHandle,&WirelessTransmitMessage,(TickType_t)0 );		
}

static void EMG_Data_LimitingFilter(void)
{
	uint8_t channel_i=0;
	float EmgData_i=0;
	for(uint8_t channel_i=0;channel_i<8;channel_i++)
	{
		if(EmgData_8Channels[channel_i]>2048){
			EmgData_i=2048;
		}
		else if(EmgData_8Channels[channel_i]<-2048){
			EmgData_i=-2048;
		}
		else{
			EmgData_i=EmgData_8Channels[channel_i];
		}
		Set_EMG_Sensor_Electrode_Data((EMG_Sensor_Electrode_n)(EMG_Sensor_Electrode1_n+channel_i),&EmgData_i);
	}
}

/*
*****************************************************************************
*@brief		ECU1����Labviewģ��2��EMG_Sensor����Э�黯����
*@param		uint8_t Labview_ID��Labview�н������ݵ�ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU1_Labview_EMG_SensorData_Send(uint8_t Labview_ID)
{
	WirelessTransmit2_MessageTypedef WirelessTransmitMessage;
	WirelessTransmitMessage.WirelessTransmitID=Labview_ID;	
	float EMG_Sensor_Electrode1_data,EMG_Sensor_Electrode2_data,EMG_Sensor_Electrode3_data,EMG_Sensor_Electrode4_data;
	float EMG_Sensor_Electrode5_data,EMG_Sensor_Electrode6_data,EMG_Sensor_Electrode7_data,EMG_Sensor_Electrode8_data;
	
	if(Labview_ID==EMG_Sensor_Electrode1_ID){
		Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode1_n,&EMG_Sensor_Electrode1_data);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(EMG_Sensor_Electrode1_data*10+32767)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=EMG_Sensor_Electrode1_data*10+32767;
	}
	else if(Labview_ID==EMG_Sensor_Electrode2_ID){
		Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode2_n,&EMG_Sensor_Electrode2_data);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(EMG_Sensor_Electrode2_data*10+32767)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=EMG_Sensor_Electrode2_data*10+32767;
	}	
	else if(Labview_ID==EMG_Sensor_Electrode3_ID){
		Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode3_n,&EMG_Sensor_Electrode3_data);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(EMG_Sensor_Electrode3_data*10+32767)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=EMG_Sensor_Electrode3_data*10+32767;
	}	
	else if(Labview_ID==EMG_Sensor_Electrode4_ID){
		Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode4_n,&EMG_Sensor_Electrode4_data);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(EMG_Sensor_Electrode4_data*10+32767)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=EMG_Sensor_Electrode4_data*10+32767;
	}	
	else if(Labview_ID==EMG_Sensor_Electrode5_ID){
		Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode5_n,&EMG_Sensor_Electrode5_data);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(EMG_Sensor_Electrode5_data*10+32767)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=EMG_Sensor_Electrode5_data*10+32767;
	}	
	else if(Labview_ID==EMG_Sensor_Electrode6_ID){
		Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode6_n,&EMG_Sensor_Electrode6_data);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(EMG_Sensor_Electrode6_data*10+32767)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=EMG_Sensor_Electrode6_data*10+32767;
	}	
	else if(Labview_ID==EMG_Sensor_Electrode7_ID){
		Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode7_n,&EMG_Sensor_Electrode7_data);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(EMG_Sensor_Electrode7_data*10+32767)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=EMG_Sensor_Electrode7_data*10+32767;
	}	
	else if(Labview_ID==EMG_Sensor_Electrode8_ID){
		Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode8_n,&EMG_Sensor_Electrode8_data);		
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(EMG_Sensor_Electrode8_data*10+32767)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=EMG_Sensor_Electrode8_data*10+32767;
	}	
	xQueueSend(WirelessTransmit2_LabviewQueueHandle,&WirelessTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		ECU1����Labviewģ��2��Faulhaber_Feedback����Э�黯����
*@param		uint8_t Labview_ID��Labview�н������ݵ�ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU1_Labview_Faulhaber_FeedbackData_Send(uint8_t Labview_ID)
{
	WirelessTransmit2_MessageTypedef WirelessTransmitMessage;
	WirelessTransmitMessage.WirelessTransmitID=Labview_ID;
	float ThumbJointMot_Speed=0,ThumbSwingMot_Speed=0,IndexFingerSwingMot_Speed=0,MiddleFingerSwingMot_Speed=0,RingFingerSwingMot_Speed=0,LittleFingerSwingMot_Speed=0;
	uint16_t ThumbJointMot_current=0,ThumbSwingMot_current=0,IndexFingerSwingMot_current=0,MiddleFingerSwingMot_current=0,RingFingerSwingMot_current=0,LittleFingerSwingMot_current=0;
	if(Labview_ID==ThumbJointMot_AutoModeSpeed_ID){
		Get_Faulhaber_FeedbackData(ThumbJointMot_AutoModeSpeed_n,&ThumbJointMot_Speed);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(ThumbJointMot_Speed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=ThumbJointMot_Speed*1000+29286;
	}	
	else if(Labview_ID==ThumbSwingMot_AutoModeSpeed_ID){
		Get_Faulhaber_FeedbackData(ThumbSwingMot_AutoModeSpeed_n,&ThumbSwingMot_Speed);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(ThumbSwingMot_Speed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=ThumbSwingMot_Speed*1000+29286;
	}	
	else if(Labview_ID==IndexFingerSwingMot_AutoModeSpeed_ID){
		Get_Faulhaber_FeedbackData(IndexFingerSwingMot_AutoModeSpeed_n,&IndexFingerSwingMot_Speed);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(IndexFingerSwingMot_Speed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=IndexFingerSwingMot_Speed*1000+29286;
	}	
	else if(Labview_ID==MiddleFingerSwingMot_AutoModeSpeed_ID){
		Get_Faulhaber_FeedbackData(MiddleFingerSwingMot_AutoModeSpeed_n,&MiddleFingerSwingMot_Speed);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(MiddleFingerSwingMot_Speed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=MiddleFingerSwingMot_Speed*1000+29286;
	}	
	else if(Labview_ID==RingFingerSwingMot_AutoModeSpeed_ID){
		Get_Faulhaber_FeedbackData(RingFingerSwingMot_AutoModeSpeed_n,&RingFingerSwingMot_Speed);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(RingFingerSwingMot_Speed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=RingFingerSwingMot_Speed*1000+29286;
	}	
	else if(Labview_ID==LittleFingerSwingMot_AutoModeSpeed_ID){
		Get_Faulhaber_FeedbackData(LittleFingerSwingMot_AutoModeSpeed_n,&LittleFingerSwingMot_Speed);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(LittleFingerSwingMot_Speed*1000+29286)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=LittleFingerSwingMot_Speed*1000+29286;
	}	
	else if(Labview_ID==ThumbJointMot_Current_ID){
		Get_Faulhaber_FeedbackData(ThumbJointMot_Current_n,&ThumbJointMot_current);
		WirelessTransmitMessage.WirelessTransmitData[0]=ThumbJointMot_current>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=ThumbJointMot_current;
	}	
	else if(Labview_ID==ThumbSwingMot_Current_ID){
		Get_Faulhaber_FeedbackData(ThumbSwingMot_Current_n,&ThumbSwingMot_current);
		WirelessTransmitMessage.WirelessTransmitData[0]=ThumbSwingMot_current>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=ThumbSwingMot_current;
	}	
	else if(Labview_ID==IndexFingerSwingMot_Current_ID){
		Get_Faulhaber_FeedbackData(IndexFingerSwingMot_Current_n,&IndexFingerSwingMot_current);
		WirelessTransmitMessage.WirelessTransmitData[0]=IndexFingerSwingMot_current>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=IndexFingerSwingMot_current;
	}	
	else if(Labview_ID==MiddleFingerSwingMot_Current_ID){
		Get_Faulhaber_FeedbackData(MiddleFingerSwingMot_Current_n,&MiddleFingerSwingMot_current);
		WirelessTransmitMessage.WirelessTransmitData[0]=MiddleFingerSwingMot_current>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=MiddleFingerSwingMot_current;
	}	
	else if(Labview_ID==RingFingerSwingMot_Current_ID){
		Get_Faulhaber_FeedbackData(RingFingerSwingMot_Current_n,&RingFingerSwingMot_current);
		WirelessTransmitMessage.WirelessTransmitData[0]=RingFingerSwingMot_current>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=RingFingerSwingMot_current;
	}	
	else if(Labview_ID==LittleFingerSwingMot_Current_ID){
		Get_Faulhaber_FeedbackData(LittleFingerSwingMot_Current_n,&LittleFingerSwingMot_current);
		WirelessTransmitMessage.WirelessTransmitData[0]=LittleFingerSwingMot_current>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=LittleFingerSwingMot_current;
	}	
	xQueueSend(WirelessTransmit2_LabviewQueueHandle,&WirelessTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		ECU1����Labviewģ��2��ProstheticHand_Sensor����Э�黯����
*@param		uint8_t Labview_ID��Labview�н������ݵ�ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU1_Labview_ProstheticHand_SensorData_Send(uint8_t Labview_ID)
{
	WirelessTransmit2_MessageTypedef WirelessTransmitMessage;
	WirelessTransmitMessage.WirelessTransmitID=Labview_ID;
	float FSR1_Pressure=0,FSR2_Pressure=0,FSR3_Pressure=0,FSR4_Pressure=0,Thumb_SwingBending=0,IndexFinger_SwingBending=0,MiddleFinger_SwingBending=0;
	if(Labview_ID==FSR1_NormalPressure_ID){
		Get_ProstheticHand_SensorData(FSR1_NormalPressure_n,&FSR1_Pressure);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(FSR1_Pressure*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=FSR1_Pressure*100;
	}	
	else if(Labview_ID==FSR2_NormalPressure_ID){
		Get_ProstheticHand_SensorData(FSR2_NormalPressure_n,&FSR2_Pressure);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(FSR2_Pressure*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=FSR2_Pressure*100;
	}	
	else if(Labview_ID==FSR3_NormalPressure_ID){
		Get_ProstheticHand_SensorData(FSR3_NormalPressure_n,&FSR3_Pressure);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(FSR3_Pressure*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=FSR3_Pressure*100;
	}	
	else if(Labview_ID==FSR4_NormalPressure_ID){
	Get_ProstheticHand_SensorData(FSR4_NormalPressure_n,&FSR4_Pressure);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(FSR4_Pressure*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=FSR4_Pressure*100;
	}	
	else if(Labview_ID==Thumb_Bending_ID){
		Get_ProstheticHand_SensorData(Thumb_Bending_n,&Thumb_SwingBending);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(Thumb_SwingBending*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=Thumb_SwingBending*100;
	}	
	else if(Labview_ID==IndexFinger_Bending_ID){
		Get_ProstheticHand_SensorData(IndexFinger_Bending_n,&IndexFinger_SwingBending);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(IndexFinger_SwingBending*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=IndexFinger_SwingBending*100;
	}
	else if(Labview_ID==MiddleFinger_Bending_ID){
		Get_ProstheticHand_SensorData(MiddleFinger_Bending_n,&MiddleFinger_SwingBending);
		WirelessTransmitMessage.WirelessTransmitData[0]=(uint16_t)(MiddleFinger_SwingBending*100)>>8;
		WirelessTransmitMessage.WirelessTransmitData[1]=MiddleFinger_SwingBending*100;
	}	
	xQueueSend(WirelessTransmit2_LabviewQueueHandle,&WirelessTransmitMessage,(TickType_t)0 );
}

/*
*****************************************************************************
*@brief		ECU1����Labviewģ��2����Э�黯����
*@param		uint8_t Labview_ID��Labview�н������ݵ�ID
*@retval	None
*@par
*****************************************************************************
*/
static void ECU1_Labview_Module2Data_Send(uint8_t Labview_ID)
{
	if(Labview_ID>=EMG_Sensor_Electrode1_ID&&Labview_ID<=EMG_Sensor_Electrode8_ID&&~(Labview_ID%2)){
		ECU1_Labview_EMG_SensorData_Send(Labview_ID);
	}
	else if(Labview_ID>=ThumbJointMot_AutoModeSpeed_ID&&Labview_ID<=LittleFingerSwingMot_Current_ID&&~(Labview_ID%2)){
		ECU1_Labview_Faulhaber_FeedbackData_Send(Labview_ID);
	}
	else if(Labview_ID>=FSR1_NormalPressure_ID&&Labview_ID<=MiddleFinger_Bending_ID&&~(Labview_ID%2)){
		ECU1_Labview_ProstheticHand_SensorData_Send(Labview_ID);
	}
	else if(Labview_ID>=Recognized_TypesofGestures_ID&&Labview_ID<=Bluetooth_PairingStatus_ID&&~(Labview_ID%2)){
		WirelessTransmit2_MessageTypedef WirelessTransmitMessage;
		WirelessTransmitMessage.WirelessTransmitID=Labview_ID;	
		uint16_t TypesofGestures=0,Bluetooth_Pairingstatus=0;
		if(Labview_ID==Recognized_TypesofGestures_ID){
			Get_Recognized_Gestures_Data(Recognized_TypesofGestures_n,&TypesofGestures);
			WirelessTransmitMessage.WirelessTransmitData[0]=TypesofGestures>>8;
			WirelessTransmitMessage.WirelessTransmitData[1]=TypesofGestures*100;
		}
		else if(Labview_ID==Bluetooth_PairingStatus_ID){
			Get_Bluetooth_Status_Data(Bluetooth_PairingStatus_n,&Bluetooth_Pairingstatus);
			WirelessTransmitMessage.WirelessTransmitData[0]=Bluetooth_Pairingstatus>>8;
			WirelessTransmitMessage.WirelessTransmitData[1]=Bluetooth_Pairingstatus*100;
		}		
		xQueueSend(WirelessTransmit2_LabviewQueueHandle,&WirelessTransmitMessage,(TickType_t)0 );
	}
}


/*
*****************************************************************************
*@brief		Wireless�������ݺ���,����ģ��1��2������
*@param		uint8_t Labview_ID��Labview�н�������ID
*@retval	None
*@par
*****************************************************************************
*/
void Wireless_DATA_Send1And2(uint8_t Labview_ID)
{	
	if(Labview_ID>=Transmit1AndReceive1Labview_StartAddress_ID&&Labview_ID<=Transmit1AndReceive1Labview_EndAddress_ID&&~(Labview_ID%2)){
		ECU1_Labview_Readback_Send(Labview_ID);
	}
	else if(Labview_ID>=EMG_Sensor_Electrode1_ID&&Labview_ID<=Bluetooth_PairingStatus_ID&&~(Labview_ID%2)){
		ECU1_Labview_Module2Data_Send(Labview_ID);
	}
}

/* SendLabviewReadbackData_Period50Task function */
void SendLabviewReadbackData_Period50Task(void const * argument)
{	
  /* USER CODE BEGIN SendLabviewReadbackData_Period50Task */
  /* Infinite loop */
	uint8_t i=0;
	for(;;)
  {	
//		Wireless_DATA_Send1And2((uint8_t)i*2);
//		i++;
//		if(i==Transmit1AndReceive1LabviewDataNum){
//			i=0;			
//		}
		osDelay(50);
  }
  /* USER CODE END SendLabviewReadbackData_Period50Task */
}

/* SendLabviewModule2Data_Period100Task function */
void SendLabviewModule2Data_Period100Task(void const * argument)
{	
  /* USER CODE BEGIN SendLabviewModule2Data_Period100Task */
  /* Infinite loop */
	//��������Ϊ100ms
	for(;;)
  {	
//		for(uint8_t i=0;i<6;i++)
//		{
//			Wireless_DATA_Send1And2(ThumbJointMot_AutoModeSpeed_ID+i*2);
//		}	
//		osDelay(10);
//		Wireless_DATA_Send1And2(Recognized_TypesofGestures_ID);	//ʶ�����������
//		Wireless_DATA_Send1And2(Bluetooth_PairingStatus_ID);		//�������״̬����		
//		osDelay(90);
//		Wireless_DATA_Send1And2(ThumbJointMot_AutoModeSpeed_ID);
		Wireless_DATA_Send1And2(ThumbSwingMot_AutoModeSpeed_ID);
		osDelay(20);
  }
  /* USER CODE END SendLabviewModule2Data_Period100Task */
}

/* SendLabviewModule2Data_Period20Task function */
void SendLabviewModule2Data_Period20Task(void const * argument)
{	
  /* USER CODE BEGIN SendLabviewModule2Data_Period20Task */
  /* Infinite loop */
	uint8_t i=0;
	//��������Ϊ26ms
	for(;;)
  {	
		//���ͼ����ź�
//		EMG_Data_LimitingFilter();
//		for(i=0;i<8;i++)
//		{
//			Wireless_DATA_Send1And2(EMG_Sensor_Electrode1_ID+i*2);
//		}		
//		osDelay(10);
		//���͵�������
//		for(i=0;i<6;i++)
//		{
//			Wireless_DATA_Send1And2(ThumbJointMot_Current_ID+i*2);
//		}
//		osDelay(8);
		//����FSR����
//		for(i=0;i<4;i++)
//		{
//			Wireless_DATA_Send1And2(FSR1_NormalPressure_ID+i*2);
//		}
		Wireless_DATA_Send1And2(FSR1_NormalPressure_ID);
		osDelay(20);
		//������������������
//		for(i=0;i<3;i++)
//		{
//			Wireless_DATA_Send1And2(Thumb_Bending_ID+i*2);
//		}
//		osDelay(10);
  }
  /* USER CODE END SendLabviewModule2Data_Period20Task */
}

/* Wireless_Transmit1_LabviewTask function */
void Wireless_Transmit1_LabviewTask(void const * argument)
{	
  /* USER CODE BEGIN Wireless_Transmit1_LabviewTask */
  /* Infinite loop */
	WirelessTransmit1_MessageTypedef WirelessTransmit_Message;
	uint8_t WirelessTransmit_Data[5];	
  for(;;)
  {	
		if(xQueueReceive(WirelessTransmit1_LabviewQueueHandle,&WirelessTransmit_Message,(TickType_t)0)==pdTRUE){
			WirelessTransmit_Data[2]=WirelessTransmit_Message.WirelessTransmitID;
			WirelessTransmit_Data[0]=Transmit1AndReceive1LabviewHeader>>8;
			WirelessTransmit_Data[1]=(uint8_t)Transmit1AndReceive1LabviewHeader;
			WirelessTransmit_Data[3]=WirelessTransmit_Message.WirelessTransmitData[0];
			WirelessTransmit_Data[4]=WirelessTransmit_Message.WirelessTransmitData[1];
			HAL_UART_Transmit_DMA(&huart3,WirelessTransmit_Data,5);
		}
		osDelay(1);
  }
  /* USER CODE END Wireless_Transmit1_LabviewTask */
}

/* Wireless_Transmit2_LabviewTask function */
void Wireless_Transmit2_LabviewTask(void const * argument)
{	
  /* USER CODE BEGIN Wireless_Transmit2_LabviewTask */
  /* Infinite loop */
	WirelessTransmit2_MessageTypedef WirelessTransmit_Message;
	uint8_t WirelessTransmit_Data[5];	
  for(;;)
  {	
		if(xQueueReceive(WirelessTransmit2_LabviewQueueHandle,&WirelessTransmit_Message,(TickType_t)0)==pdTRUE){
			WirelessTransmit_Data[2]=WirelessTransmit_Message.WirelessTransmitID;
			WirelessTransmit_Data[0]=Transmit2LabviewHeader>>8;
			WirelessTransmit_Data[1]=(uint8_t)Transmit2LabviewHeader;		
			WirelessTransmit_Data[3]=WirelessTransmit_Message.WirelessTransmitData[0];
			WirelessTransmit_Data[4]=WirelessTransmit_Message.WirelessTransmitData[1];
			HAL_UART_Transmit_DMA(&huart3,WirelessTransmit_Data,5);
		}
		osDelay(1);
  }
  /* USER CODE END Wireless_Transmit2_LabviewTask */
}

/*
*****************************************************************************
*@brief		ȡֵ����������Wireless���ղ������õ�����
*@param		Wireless_ReceiveData_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_Wireless_DataFrom_EEPROM(Transmit1AndReceive1LabviewData_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case ManualOrAutomatic_ModeSelect_n:
			*(uint16_t*)extern_data=Transmit1AndReceive1LabviewData.ManualOrAutomatic_ModeSelect;
			break;
		case MotorEnableOrDisable_n:
			*(uint16_t*)extern_data=Transmit1AndReceive1LabviewData.MotorEnableOrDisable;
			break;
		case ThumbJointMot_TargetSpeed_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.ThumbJointMot_TargetSpeed;
			break;
		case ThumbSwingMot_TargetSpeed_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.ThumbSwingMot_TargetSpeed;
			break;
		case IndexFingerSwingMot_TargetSpeed_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.IndexFingerSwingMot_TargetSpeed;
			break;
		case MiddleFingerSwingMot_TargetSpeed_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.MiddleFingerSwingMot_TargetSpeed;
			break;
		case RingFingerSwingMot_TargetSpeed_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.RingFingerSwingMot_TargetSpeed;
			break;
		case LittleFingerSwingMot_TargetSpeed_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.LittleFingerSwingMot_TargetSpeed;
			break;
		case PowerUp_DefaultTypesofGestures_n:
			*(uint16_t*)extern_data=Transmit1AndReceive1LabviewData.PowerUp_DefaultTypesofGestures;
			break;
		case Manual_TargetTypesofGesture_n:
			*(uint16_t*)extern_data=Transmit1AndReceive1LabviewData.Manual_TargetTypesofGesture;
			break;
		case Bluetooth_AT_SetMasterMode_Flag_n:
			*(uint16_t*)extern_data=Transmit1AndReceive1LabviewData.Bluetooth_AT_SetMasterMode_Flag;
			break;		
		case Grip_Control_Parameters_P_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.Grip_Control_Parameters_P;
			break;	
		case Grip_Control_Parameters_I_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.Grip_Control_Parameters_I;
			break;	
		case Grip_Control_Parameters_D_n:
			*(float*)extern_data=Transmit1AndReceive1LabviewData.Grip_Control_Parameters_D;
			break;			
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�Wireless���ղ������õ�����
*@param		Wireless_ReceiveData_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void* extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_Wireless_DataFrom_EEPROM(Transmit1AndReceive1LabviewData_n name_num,void*extern_data)//*extern_data���޸�Ŀ��ֵ����
{
	switch(name_num)
	{
		case ManualOrAutomatic_ModeSelect_n:
			Transmit1AndReceive1LabviewData.ManualOrAutomatic_ModeSelect=*(uint16_t*)extern_data;
			break;
		case MotorEnableOrDisable_n:
			Transmit1AndReceive1LabviewData.MotorEnableOrDisable=*(uint16_t*)extern_data;
			break;
		case ThumbJointMot_TargetSpeed_n:
			Transmit1AndReceive1LabviewData.ThumbJointMot_TargetSpeed=*(float*)extern_data;
			break;
		case ThumbSwingMot_TargetSpeed_n:
			Transmit1AndReceive1LabviewData.ThumbSwingMot_TargetSpeed=*(float*)extern_data;
			break;
		case IndexFingerSwingMot_TargetSpeed_n:
			Transmit1AndReceive1LabviewData.IndexFingerSwingMot_TargetSpeed=*(float*)extern_data;
			break;
		case MiddleFingerSwingMot_TargetSpeed_n:
			Transmit1AndReceive1LabviewData.MiddleFingerSwingMot_TargetSpeed=*(float*)extern_data;
			break;
		case RingFingerSwingMot_TargetSpeed_n:
			Transmit1AndReceive1LabviewData.RingFingerSwingMot_TargetSpeed=*(float*)extern_data;
			break;
		case LittleFingerSwingMot_TargetSpeed_n:
			Transmit1AndReceive1LabviewData.LittleFingerSwingMot_TargetSpeed=*(float*)extern_data;
			break;
		case PowerUp_DefaultTypesofGestures_n:
			Transmit1AndReceive1LabviewData.PowerUp_DefaultTypesofGestures=*(uint16_t*)extern_data;
			break;
		case Manual_TargetTypesofGesture_n:
			Transmit1AndReceive1LabviewData.Manual_TargetTypesofGesture=*(uint16_t*)extern_data;
			break;
		case Bluetooth_AT_SetMasterMode_Flag_n:
			Transmit1AndReceive1LabviewData.Bluetooth_AT_SetMasterMode_Flag=*(uint16_t*)extern_data;
			break;		
		case Grip_Control_Parameters_P_n:
			Transmit1AndReceive1LabviewData.Grip_Control_Parameters_P=*(float*)extern_data;
			break;	
		case Grip_Control_Parameters_I_n:
			Transmit1AndReceive1LabviewData.Grip_Control_Parameters_I=*(float*)extern_data;
			break;	
		case Grip_Control_Parameters_D_n:
			Transmit1AndReceive1LabviewData.Grip_Control_Parameters_D=*(float*)extern_data;
			break;	
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		EEPROM�������������ֻ����һ��
*@param		None
*@retval	None
*@par
*****************************************************************************
*/
static void CLEARData_For_EEPROM(void)
{
	for(uint8_t i=0;i<2*Transmit1AndReceive1LabviewDataNum;i++)
	{
		AT24Cxx_WriteOneByte(i,0);
	}
}

/*
*****************************************************************************
*@brief		�����ϵ��EEPROM���ݺ���
*@param		None
*@retval	None
*@par
*****************************************************************************
*/
void FirstPowerOnReadEEPROMInit(void)
{
	CLEARData_For_EEPROM();
	Transmit1AndReceive1LabviewData.ManualOrAutomatic_ModeSelect=(AT24Cxx_ReadOneByte(ManualOrAutomatic_ModeSelect_ID)<<8)+AT24Cxx_ReadOneByte(ManualOrAutomatic_ModeSelect_ID+1);
	Transmit1AndReceive1LabviewData.MotorEnableOrDisable=(AT24Cxx_ReadOneByte(MotorEnableOrDisable_ID)<<8)+AT24Cxx_ReadOneByte(MotorEnableOrDisable_ID+1);
	Transmit1AndReceive1LabviewData.ThumbJointMot_TargetSpeed=0.001*((AT24Cxx_ReadOneByte(ThumbJointMot_TargetSpeed_ID)<<8)+AT24Cxx_ReadOneByte(ThumbJointMot_TargetSpeed_ID+1)-29286);
	Transmit1AndReceive1LabviewData.ThumbSwingMot_TargetSpeed=0.001*((AT24Cxx_ReadOneByte(ThumbSwingMot_TargetSpeed_ID)<<8)+AT24Cxx_ReadOneByte(ThumbSwingMot_TargetSpeed_ID+1)-29286);
	Transmit1AndReceive1LabviewData.IndexFingerSwingMot_TargetSpeed=0.001*((AT24Cxx_ReadOneByte(IndexFingerSwingMot_TargetSpeed_ID)<<8)+AT24Cxx_ReadOneByte(IndexFingerSwingMot_TargetSpeed_ID+1)-29286);
	Transmit1AndReceive1LabviewData.MiddleFingerSwingMot_TargetSpeed=0.001*((AT24Cxx_ReadOneByte(MiddleFingerSwingMot_TargetSpeed_ID)<<8)+AT24Cxx_ReadOneByte(MiddleFingerSwingMot_TargetSpeed_ID+1)-29286);
	Transmit1AndReceive1LabviewData.RingFingerSwingMot_TargetSpeed=0.001*((AT24Cxx_ReadOneByte(RingFingerSwingMot_TargetSpeed_ID)<<8)+AT24Cxx_ReadOneByte(RingFingerSwingMot_TargetSpeed_ID+1)-29286);
	Transmit1AndReceive1LabviewData.LittleFingerSwingMot_TargetSpeed=0.001*((AT24Cxx_ReadOneByte(LittleFingerSwingMot_TargetSpeed_ID)<<8)+AT24Cxx_ReadOneByte(LittleFingerSwingMot_TargetSpeed_ID+1)-29286);
	Transmit1AndReceive1LabviewData.PowerUp_DefaultTypesofGestures=(AT24Cxx_ReadOneByte(PowerUp_DefaultTypesofGestures_ID)<<8)+AT24Cxx_ReadOneByte(PowerUp_DefaultTypesofGestures_ID+1);
	Transmit1AndReceive1LabviewData.Manual_TargetTypesofGesture=(AT24Cxx_ReadOneByte(Manual_TargetTypesofGesture_ID)<<8)+AT24Cxx_ReadOneByte(Manual_TargetTypesofGesture_ID+1);
	Transmit1AndReceive1LabviewData.Bluetooth_AT_SetMasterMode_Flag=(AT24Cxx_ReadOneByte(Bluetooth_AT_SetMasterMode_Flag_ID)<<8)+AT24Cxx_ReadOneByte(Bluetooth_AT_SetMasterMode_Flag_ID+1);
	Transmit1AndReceive1LabviewData.Grip_Control_Parameters_P=0.01*((AT24Cxx_ReadOneByte(Grip_Control_Parameters_P_ID)<<8)+AT24Cxx_ReadOneByte(Grip_Control_Parameters_P_ID+1)-32000);
	Transmit1AndReceive1LabviewData.Grip_Control_Parameters_I=0.01*((AT24Cxx_ReadOneByte(Grip_Control_Parameters_I_ID)<<8)+AT24Cxx_ReadOneByte(Grip_Control_Parameters_I_ID+1)-32000);
	Transmit1AndReceive1LabviewData.Grip_Control_Parameters_D=0.01*((AT24Cxx_ReadOneByte(Grip_Control_Parameters_D_ID)<<8)+AT24Cxx_ReadOneByte(Grip_Control_Parameters_D_ID+1)-32000);

}







