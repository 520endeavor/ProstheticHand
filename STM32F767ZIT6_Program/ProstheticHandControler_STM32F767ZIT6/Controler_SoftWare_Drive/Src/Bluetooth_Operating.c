#include "Bluetooth_Operating.h"

struct {
	volatile float  EMG_Sensor_Electrode1;								//���紫�����缫1����
	volatile float  EMG_Sensor_Electrode2;								//���紫�����缫2����
	volatile float  EMG_Sensor_Electrode3;								//���紫�����缫3����
	volatile float  EMG_Sensor_Electrode4;								//���紫�����缫4����
	volatile float  EMG_Sensor_Electrode5;								//���紫�����缫5����
	volatile float  EMG_Sensor_Electrode6;								//���紫�����缫6����
	volatile float  EMG_Sensor_Electrode7;								//���紫�����缫7����
	volatile float  EMG_Sensor_Electrode8;								//���紫�����缫8����
}EMG_Sensor_Electrode;//EMG�����ź�����

struct {
	volatile uint8_t Bluetooth_PairingStatus;							//�������״̬����
}Bluetooth_Status;//����״̬����


/*------------����ATָ���������ģ��Ϊ��ģ�飬ʱʹ��--------------------*/
uint8_t AT_SetMasterMode[]="AT+ROLE=1\r\n";							//��������ģ��Ϊ��ģ��ATָ��
uint8_t AT_Receivedata[]="OK\r\n";											//���óɹ�������Ϣ
uint8_t receiveTest[20]={0};														//���ATָ�����Ϣ

volatile double EmgData_8Channels[EmgChannel]={0};			//��ŵ�ǰ�����ź�����

volatile uint8_t DMA_recv_complete_flag=0;							//��������DMA����������ɱ�־λ

osThreadId Bluetooth_AT_SetMasterModeTaskHandle;
osThreadId Bluetooth_Match_IndicateTaskHandle;
osThreadId Bluetooth_USART1_ReceiveTaskHandle;

void Bluetooth_AT_SetMasterModeTask(void const * argument);
void Bluetooth_Match_IndicateTask(void const * argument);
void Bluetooth_USART1_ReceiveTask(void const * argument);

void USART_FREERTOS_Init(void) 
{
	/* definition and creation of Bluetooth_AT_SetMasterModeTask */
  osThreadDef(Bluetooth_AT_SetMasterModeTask, Bluetooth_AT_SetMasterModeTask, osPriorityNormal, 0, 128);
  Bluetooth_AT_SetMasterModeTaskHandle = osThreadCreate(osThread(Bluetooth_AT_SetMasterModeTask), NULL);
	/* definition and creation of Bluetooth_Match_InstructionTask */
  osThreadDef(Bluetooth_Match_IndicateTask, Bluetooth_Match_IndicateTask, osPriorityNormal, 0, 128);
  Bluetooth_Match_IndicateTaskHandle = osThreadCreate(osThread(Bluetooth_Match_IndicateTask), NULL);
	/* definition and creation of Bluetooth_USART1_ReceiveTask */
	osThreadDef(Bluetooth_USART1_ReceiveTask, Bluetooth_USART1_ReceiveTask, osPriorityNormal, 0, 128);
	Bluetooth_USART1_ReceiveTaskHandle = osThreadCreate(osThread(Bluetooth_USART1_ReceiveTask), NULL);
}

/* Bluetooth_AT_SetMasterModeTask function */
void Bluetooth_AT_SetMasterModeTask(void const * argument)
{
  /* USER CODE BEGIN Bluetooth_AT_SetMasterModeTask */
  /* Infinite loop */
	
  for(;;)
  {
//		����λ����������������߶�EEPROM��ATָ��ģʽ��־
//		HAL_GPIO_WritePin(GPIOC, STM32_Bluetooth_AT_CS_Pin, GPIO_PIN_SET);											//����ATָ��ģʽ
//		HAL_UART_Transmit_DMA(&huart1,AT_SetMasterMode,strlen(AT_SetMasterMode));								//������������ģ��Ϊ��ģ���ATָ�� 
		osDelay(10);    
  }
  /* USER CODE END Bluetooth_AT_SetMasterModeTask */
}

/* Bluetooth_Match_IndicateTask function */
void Bluetooth_Match_IndicateTask(void const * argument)
{
  /* USER CODE BEGIN Bluetooth_Match_IndicateTask */
  /* Infinite loop */
	uint8_t Bluetooth_PairingStatus_data=0;
  for(;;)
  {
		if(HAL_GPIO_ReadPin(STM32_Bluetooth_AT_CS_GPIO_Port, STM32_Bluetooth_AT_CS_Pin)==GPIO_PIN_SET){
			HAL_GPIO_WritePin(STM32_Bluetooth_Pairing_LED_GPIO_Port, STM32_Bluetooth_Pairing_LED_Pin,GPIO_PIN_SET);						//����ATָ��ģʽ���Զ�������ָʾ�Ʋ���
		}
		else{
			if(HAL_GPIO_ReadPin(STM32_Bluetooth_Pairing_Info_GPIO_Port, STM32_Bluetooth_Pairing_Info_Pin)==GPIO_PIN_SET){
				HAL_GPIO_WritePin(STM32_Bluetooth_Pairing_LED_GPIO_Port, STM32_Bluetooth_Pairing_LED_Pin,GPIO_PIN_RESET);				//��Գɹ����Զ�������ָʾ�Ƴ���
				Bluetooth_PairingStatus_data=1;
				Set_Bluetooth_Status_Data(Bluetooth_PairingStatus_n,&Bluetooth_PairingStatus_data);															//��Գɹ�����־λ��1
			}
			else{
				HAL_GPIO_TogglePin(STM32_Bluetooth_Pairing_LED_GPIO_Port, STM32_Bluetooth_Pairing_LED_Pin);											//δ��ԣ��Զ�������ָʾ����˸
				Bluetooth_PairingStatus_data=0;
				Set_Bluetooth_Status_Data(Bluetooth_PairingStatus_n,&Bluetooth_PairingStatus_data);															//δ��ԣ���־λ��0
				osDelay(200);
			}
		}    
		osDelay(10);    
  }
  /* USER CODE END Bluetooth_Match_IndicateTask */
}

/* Bluetooth_USART1_ReceiveTask function */
void Bluetooth_USART1_ReceiveTask(void const * argument)
{
  /* USER CODE BEGIN Bluetooth_USART1_ReceiveTask */
  /* Infinite loop */
	initiate_filter();									//filter initiate
	uint16_t  DataBuffer_Index=0;				//��ǰ�����źŴ��λ�����		
	HAL_UART_Receive_DMA(&huart1,recv_buff,recv_buff_index_max);													//���ռ����ֻ�����
  for(;;)
  {		
		if(DMA_recv_complete_flag==1){
			HAL_UART_Receive_DMA(&huart1,recv_buff,recv_buff_index_max);											//���ռ����ֻ�����
			DataBuffer_Index = deal_with_recv_buff(recv_buff);
			for(uint8_t channel_i=0;channel_i<8;channel_i++)
			{
				EmgData_8Channels[channel_i]=DataBuffer[DataBuffer_Index][channel_i];
			}		
		}
		osDelay(1);
  }
  /* USER CODE END Bluetooth_USART1_ReceiveTask */
}

/*
*****************************************************************************
*@brief		ȡֵ����������EMG�����ź�����
*@param		EMG_Sensor_Electrode_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case EMG_Sensor_Electrode1_n:
			*(float*)extern_data=EMG_Sensor_Electrode.EMG_Sensor_Electrode1;
			break;
		case EMG_Sensor_Electrode2_n:
			*(float*)extern_data=EMG_Sensor_Electrode.EMG_Sensor_Electrode2;
			break;
		case EMG_Sensor_Electrode3_n:
			*(float*)extern_data=EMG_Sensor_Electrode.EMG_Sensor_Electrode3;
			break;
		case EMG_Sensor_Electrode4_n:
			*(float*)extern_data=EMG_Sensor_Electrode.EMG_Sensor_Electrode4;
			break;
		case EMG_Sensor_Electrode5_n:
			*(float*)extern_data=EMG_Sensor_Electrode.EMG_Sensor_Electrode5;
			break;
		case EMG_Sensor_Electrode6_n:
			*(float*)extern_data=EMG_Sensor_Electrode.EMG_Sensor_Electrode6;
			break;
		case EMG_Sensor_Electrode7_n:
			*(float*)extern_data=EMG_Sensor_Electrode.EMG_Sensor_Electrode7;
			break;
		case EMG_Sensor_Electrode8_n:
			*(float*)extern_data=EMG_Sensor_Electrode.EMG_Sensor_Electrode8;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�EMG�����ź�����
*@param		EMG_Sensor_Electrode_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case EMG_Sensor_Electrode1_n:
			EMG_Sensor_Electrode.EMG_Sensor_Electrode1=*(float*)extern_data;
			break;
		case EMG_Sensor_Electrode2_n:
			EMG_Sensor_Electrode.EMG_Sensor_Electrode2=*(float*)extern_data;
			break;
		case EMG_Sensor_Electrode3_n:
			EMG_Sensor_Electrode.EMG_Sensor_Electrode3=*(float*)extern_data;
			break;
		case EMG_Sensor_Electrode4_n:
			EMG_Sensor_Electrode.EMG_Sensor_Electrode4=*(float*)extern_data;
			break;
		case EMG_Sensor_Electrode5_n:
			EMG_Sensor_Electrode.EMG_Sensor_Electrode5=*(float*)extern_data;
			break;
		case EMG_Sensor_Electrode6_n:
			EMG_Sensor_Electrode.EMG_Sensor_Electrode6=*(float*)extern_data;
			break;
		case EMG_Sensor_Electrode7_n:
			EMG_Sensor_Electrode.EMG_Sensor_Electrode7=*(float*)extern_data;
			break;
		case EMG_Sensor_Electrode8_n:
			EMG_Sensor_Electrode.EMG_Sensor_Electrode8=*(float*)extern_data;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		ȡֵ��������������״̬�ź�����
*@param		Bluetooth_Status_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_Bluetooth_Status_Data(Bluetooth_Status_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case Bluetooth_PairingStatus_n:
			*(uint16_t*)extern_data=Bluetooth_Status.Bluetooth_PairingStatus;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		�޸�����״̬�ź�����
*@param		Bluetooth_Status_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_Bluetooth_Status_Data(Bluetooth_Status_n name_num,void*extern_data)//*extern_data�������get����ֵ
{
	switch(name_num)
	{
		case Bluetooth_PairingStatus_n:
			Bluetooth_Status.Bluetooth_PairingStatus=*(uint8_t*)extern_data;
			break;
		default:			
			break;
	}
}

