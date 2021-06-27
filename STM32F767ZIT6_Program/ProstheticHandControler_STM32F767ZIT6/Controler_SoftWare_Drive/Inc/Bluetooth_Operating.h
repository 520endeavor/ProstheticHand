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

/*------------EMG�����ź�����--------------------*/
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
/*------------����״̬�������--------------------*/
typedef enum{
	Bluetooth_PairingStatus_n=310,
}Bluetooth_Status_n;

volatile extern double EmgData_8Channels[EmgChannel];			//��ŵ�ǰ�����ź�����
volatile extern uint8_t DMA_recv_complete_flag;				//��������DMA����������ɱ�־λ

void USART_FREERTOS_Init(void) ;											//����ͨ���н�������ĺ���

//���ݵ��ã�������
void Get_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode_n name_num,void*extern_data);//ȡֵ����������EMG�����ź�����,@EMG_Sensor_Electrode_n
void Set_EMG_Sensor_Electrode_Data(EMG_Sensor_Electrode_n name_num,void*extern_data);//�޸�EMG�����ź�����,@EMG_Sensor_Electrode_n
void Get_Bluetooth_Status_Data(Bluetooth_Status_n name_num,void*extern_data);//ȡֵ��������������״̬����,@Bluetooth_Status_n
void Set_Bluetooth_Status_Data(Bluetooth_Status_n name_num,void*extern_data);//�޸�����״̬����,@Bluetooth_Status_n

#endif


