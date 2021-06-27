#ifndef BLUETOOTH_OPERATING_H
#define	BLUETOOTH_OPERATING_H

#include "stm32f7xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "usart.h"
#include "gpio.h"
#include "dma.h"

#include "bluetooth.h"

/*------------EMG肌电信号数据--------------------*/
typedef enum{
	EMG_Sensor_Electrode1_n=300,
	EMG_Sensor_Electrode2_n,
	EMG_Sensor_Electrode3_n,
	EMG_Sensor_Electrode4_n,
	EMG_Sensor_Electrode5_n,
	EMG_Sensor_Electrode6_n,
	EMG_Sensor_Electrode7_n,
	EMG_Sensor_Electrode8_n,
}EMG_Sensor_Electrode_n;
/*------------蓝牙状态数据序号--------------------*/
typedef enum{
	Bluetooth_PairingStatus_n=310,
}Bluetooth_Status_n;

volatile extern double EmgData_8Channels[EmgChannel];			//存放当前肌电信号数据
volatile extern uint8_t DMA_recv_complete_flag;				//蓝牙串口DMA接收数据完成标志位

void USART_FREERTOS_Init(void) ;											//蓝牙通信中建立任务的函数

//数据调用，处理函数
void Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode_n name_num,void*extern_data);//取值函数，搬运EMG肌电信号数据,@EMG_Sensor_Electrode_n
void Set_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode_n name_num,void*extern_data);//修改EMG肌电信号数据,@EMG_Sensor_Electrode_n
void Get_Bluetooth_Status_Data(Bluetooth_Status_n name_num,void*extern_data);//取值函数，搬运蓝牙状态数据,@Bluetooth_Status_n
void Set_Bluetooth_Status_Data(Bluetooth_Status_n name_num,void*extern_data);//修改蓝牙状态数据,@Bluetooth_Status_n

#endif


