#ifndef HOSTCOMPUTER_COMMUNICATION_H
#define	HOSTCOMPUTER_COMMUNICATION_H


#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "usart.h"
#include "AT24Cxx.h" 
#include "Bluetooth_Operating.h"
#include "Gestures_Recognized.h"
#include "can.h"

/*------------Labview——Controler&Controler——Labview ID--------------------*/
//Controler接收模块1&发送模块1 ID,该列ID同为EEPROM中的存储地址

#define Transmit1AndReceive1LabviewHeader		(uint16_t)0x0197		  //Controler接收模块1&发送模块1的数据的表头		407d

#define ManualOrAutomatic_ModeSelect_ID						0x00 						//手动Or自动模式选择数据ID
#define MotorEnableOrDisable_ID										0x02						//电机使能Or失能数据ID
#define ThumbJointMot_TargetSpeed_ID							0x04						//拇指张合电机目标转速数据ID
#define ThumbSwingMot_TargetSpeed_ID							0x06						//拇指摆动电机目标转速数据ID
#define IndexFingerSwingMot_TargetSpeed_ID				0x08						//食指摆动电机目标转速数据ID
#define MiddleFingerSwingMot_TargetSpeed_ID				0x0A						//中指摆动电机目标转速数据ID
#define RingFingerSwingMot_TargetSpeed_ID					0x0C						//无名指摆动电机目标转速数据ID
#define LittleFingerSwingMot_TargetSpeed_ID				0x0E						//小指摆动电机目标转速数据ID
#define PowerUp_DefaultTypesofGestures_ID					0x10						//上电时默认手势类型数据ID
#define Manual_TargetTypesofGesture_ID						0x12						//手动模式目标手势类型数据ID
#define Bluetooth_AT_SetMasterMode_Flag_ID				0x14						//AT指令设置Master模式命令数据ID,存储地址：(EE_TYPE-1)
#define Grip_Control_Parameters_P_ID							0x16						//抓握控制参数，比例系数数据ID
#define Grip_Control_Parameters_I_ID							0x18						//抓握控制参数，积分系数数据ID
#define Grip_Control_Parameters_D_ID							0x1A						//抓握控制参数，微分系数数据ID

#define Transmit1AndReceive1Labview_StartAddress_ID						ManualOrAutomatic_ModeSelect_ID		//Controler接收模块1&发送模块1的数据的首地址
#define Transmit1AndReceive1Labview_EndAddress_ID							Grip_Control_Parameters_D_ID			//Controler接收模块1&发送模块1的数据的末地址

#define Transmit1AndReceive1LabviewDataNum				14								//Controler接收模块1&发送模块1的数据的个数

/*------------Controler——Labview ID--------------------*/
//Controler发送模块2 ID
#define Transmit2LabviewHeader							(uint16_t)0x043A		  //Controler发送模块2的数据的表头		1082d

#define EMG_Sensor_Electrode1_ID									0x64 						//肌电传感器电极1数据ID
#define EMG_Sensor_Electrode2_ID									0x66 						//肌电传感器电极2数据ID
#define EMG_Sensor_Electrode3_ID									0x68 						//肌电传感器电极3数据ID
#define EMG_Sensor_Electrode4_ID									0x6A 						//肌电传感器电极4数据ID
#define EMG_Sensor_Electrode5_ID									0x6C 						//肌电传感器电极5数据ID
#define EMG_Sensor_Electrode6_ID									0x6E 						//肌电传感器电极6数据ID
#define EMG_Sensor_Electrode7_ID									0x70 						//肌电传感器电极7数据ID
#define EMG_Sensor_Electrode8_ID									0x72 						//肌电传感器电极8数据ID
#define ThumbJointMot_AutoModeSpeed_ID						0x74						//拇指张合电机自动模式转速数据ID
#define ThumbSwingMot_AutoModeSpeed_ID						0x76						//拇指摆动电机自动模式转速数据ID
#define IndexFingerSwingMot_AutoModeSpeed_ID			0x78						//食指摆动电机自动模式转速数据ID
#define MiddleFingerSwingMot_AutoModeSpeed_ID			0x7A						//中指摆动电机自动模式转速数据ID
#define RingFingerSwingMot_AutoModeSpeed_ID				0x7C						//无名指摆动电机自动模式转速数据ID
#define LittleFingerSwingMot_AutoModeSpeed_ID			0x7E						//小指摆动电机自动模式转速数据ID
#define ThumbJointMot_Current_ID									0x80						//拇指张合电机电流数据ID
#define ThumbSwingMot_Current_ID									0x82						//拇指摆动电机电流数据ID
#define IndexFingerSwingMot_Current_ID						0x84						//食指摆动电机电流数据ID
#define MiddleFingerSwingMot_Current_ID						0x86						//中指摆动电机电流数据ID
#define RingFingerSwingMot_Current_ID							0x88						//无名指摆动电机电流数据ID
#define LittleFingerSwingMot_Current_ID						0x8A						//小指摆动电机电流数据ID
#define FSR1_NormalPressure_ID										0x8C						//FSR1法向压力数据ID
#define FSR2_NormalPressure_ID										0x8E						//FSR2法向压力数据ID
#define FSR3_NormalPressure_ID										0x90						//FSR3法向压力数据ID
#define FSR4_NormalPressure_ID										0x92						//FSR4法向压力数据ID
#define Thumb_Bending_ID													0x94						//拇指弯曲度数据ID
#define IndexFinger_Bending_ID										0x96						//食指弯曲度数据ID
#define MiddleFinger_Bending_ID										0x98						//中指弯曲度数据ID
#define Recognized_TypesofGestures_ID							0x9A						//识别的手势类型数据ID
#define Bluetooth_PairingStatus_ID								0x9C						//蓝牙配对状态数据ID

#define Transmit2Labview_StartAddress_ID					EMG_Sensor_Electrode1_ID		//Controler发送模块2的数据的首地址
#define Transmit2Labview_EndAddress_ID						Bluetooth_PairingStatus_ID	//Controler发送模块2的数据的末地址

#define Transmit2LabviewDataNum										29							//Controler发送模块2的数据的个数

/*------------Controler接收模块1&发送模块1数据序号--------------------*/
typedef enum{
	ManualOrAutomatic_ModeSelect_n=150, 			
	MotorEnableOrDisable_n,								
	ThumbJointMot_TargetSpeed_n,					
	ThumbSwingMot_TargetSpeed_n,							
	IndexFingerSwingMot_TargetSpeed_n,				
	MiddleFingerSwingMot_TargetSpeed_n,			
	RingFingerSwingMot_TargetSpeed_n,				
	LittleFingerSwingMot_TargetSpeed_n,			
	PowerUp_DefaultTypesofGestures_n,
	Manual_TargetTypesofGesture_n,	
	Bluetooth_AT_SetMasterMode_Flag_n,
	Grip_Control_Parameters_P_n,
	Grip_Control_Parameters_I_n,
	Grip_Control_Parameters_D_n,
}Transmit1AndReceive1LabviewData_n;


typedef struct{
	uint8_t EEPROMMemoryID;
	uint8_t EEPROMMemoryData[2];
}EEPROMMemoryMessageTypedef;

typedef struct{
	uint8_t WirelessTransmitID;
	uint8_t WirelessTransmitData[2];
}WirelessTransmit1_MessageTypedef;

typedef struct{
	uint8_t WirelessTransmitID;
	uint8_t WirelessTransmitData[2];
}WirelessTransmit2_MessageTypedef;

volatile extern uint8_t WirelessAnalysis_Data[5];								//无线接收数据缓存区
volatile extern uint8_t WirelessReceice_Data_OneByte;						//无线接收数据缓存变量

osThreadId Get_Wireless_ReceiveLabview_SendMemTaskHandle(); 		//获取Wireless_ReceiveLabview_SendMemTaskHandle句柄函数

void Wireless_FREERTOS_Init(void);															//Wireless通信中建立任务的函数

uint8_t Wireless_ReceiveData_Analysis(uint8_t Wireless_ReceiveData,uint8_t *Analysis_Data);	//无线接收数据解析函数

void Wireless_DATA_Send1And2(uint8_t Labview_ID);								//投递Wireless发送内容到队列的函数，发送模块1和2的数据

//数据调用，处理函数
void Get_Wireless_DataFrom_EEPROM(Transmit1AndReceive1LabviewData_n name_num,void*extern_data);//取值函数，搬运Wireless接收并解析好的数据,@Transmit1AndReceive1LabviewData_n
void Set_Wireless_DataFrom_EEPROM(Transmit1AndReceive1LabviewData_n name_num,void*extern_data);//修改Wireless接收并解析好的数据,@Transmit1AndReceive1LabviewData_n
void FirstPowerOnReadEEPROMInit(void);													//第一次上电读取EEPROM中数据

#endif

