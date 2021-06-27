#include "Bluetooth_Operating.h"

struct {
	volatile float  EMG_Sensor_Electrode1;								//肌电传感器电极1数据
	volatile float  EMG_Sensor_Electrode2;								//肌电传感器电极2数据
	volatile float  EMG_Sensor_Electrode3;								//肌电传感器电极3数据
	volatile float  EMG_Sensor_Electrode4;								//肌电传感器电极4数据
	volatile float  EMG_Sensor_Electrode5;								//肌电传感器电极5数据
	volatile float  EMG_Sensor_Electrode6;								//肌电传感器电极6数据
	volatile float  EMG_Sensor_Electrode7;								//肌电传感器电极7数据
	volatile float  EMG_Sensor_Electrode8;								//肌电传感器电极8数据
}EMG_Sensor_Electrode;//EMG肌电信号数据

struct {
	volatile uint8_t Bluetooth_PairingStatus;							//蓝牙配对状态数据
}Bluetooth_Status;//蓝牙状态数据


/*------------发送AT指令，设置蓝牙模块为主模块，时使用--------------------*/
uint8_t AT_SetMasterMode[]="AT+ROLE=1\r\n";							//设置蓝牙模块为主模块AT指令
uint8_t AT_Receivedata[]="OK\r\n";											//设置成功反馈信息
uint8_t receiveTest[20]={0};														//存放AT指令反馈信息

volatile double EmgData_8Channels[EmgChannel]={0};			//存放当前肌电信号数据

volatile uint8_t DMA_recv_complete_flag=0;							//蓝牙串口DMA接收数据完成标志位

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
//		等上位机发送设置命令或者读EEPROM中AT指令模式标志
//		HAL_GPIO_WritePin(GPIOC, STM32_Bluetooth_AT_CS_Pin, GPIO_PIN_SET);											//进入AT指令模式
//		HAL_UART_Transmit_DMA(&huart1,AT_SetMasterMode,strlen(AT_SetMasterMode));								//发送设置蓝牙模块为主模块的AT指令 
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
			HAL_GPIO_WritePin(STM32_Bluetooth_Pairing_LED_GPIO_Port, STM32_Bluetooth_Pairing_LED_Pin,GPIO_PIN_SET);						//进入AT指令模式，自定义蓝牙指示灯不亮
		}
		else{
			if(HAL_GPIO_ReadPin(STM32_Bluetooth_Pairing_Info_GPIO_Port, STM32_Bluetooth_Pairing_Info_Pin)==GPIO_PIN_SET){
				HAL_GPIO_WritePin(STM32_Bluetooth_Pairing_LED_GPIO_Port, STM32_Bluetooth_Pairing_LED_Pin,GPIO_PIN_RESET);				//配对成功，自定义蓝牙指示灯常亮
				Bluetooth_PairingStatus_data=1;
				Set_Bluetooth_Status_Data(Bluetooth_PairingStatus_n,&Bluetooth_PairingStatus_data);															//配对成功，标志位置1
			}
			else{
				HAL_GPIO_TogglePin(STM32_Bluetooth_Pairing_LED_GPIO_Port, STM32_Bluetooth_Pairing_LED_Pin);											//未配对，自定义蓝牙指示灯闪烁
				Bluetooth_PairingStatus_data=0;
				Set_Bluetooth_Status_Data(Bluetooth_PairingStatus_n,&Bluetooth_PairingStatus_data);															//未配对，标志位置0
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
	uint16_t  DataBuffer_Index=0;				//当前肌电信号存放位置序号		
	HAL_UART_Receive_DMA(&huart1,recv_buff,recv_buff_index_max);													//接收肌电手环数据
  for(;;)
  {		
		if(DMA_recv_complete_flag==1){
			HAL_UART_Receive_DMA(&huart1,recv_buff,recv_buff_index_max);											//接收肌电手环数据
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
*@brief		取值函数，搬运EMG肌电信号数据
*@param		EMG_Sensor_Electrode_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode_n name_num,void*extern_data)//*extern_data用来存放get到的值
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
*@brief		修改EMG肌电信号数据
*@param		EMG_Sensor_Electrode_n name_num：修改数据对应的枚举变量
*@param		void*extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode_n name_num,void*extern_data)//*extern_data用来存放get到的值
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
*@brief		取值函数，搬运蓝牙状态信号数据
*@param		Bluetooth_Status_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_Bluetooth_Status_Data(Bluetooth_Status_n name_num,void*extern_data)//*extern_data用来存放get到的值
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
*@brief		修改蓝牙状态信号数据
*@param		Bluetooth_Status_n name_num：修改数据对应的枚举变量
*@param		void*extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_Bluetooth_Status_Data(Bluetooth_Status_n name_num,void*extern_data)//*extern_data用来存放get到的值
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

