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

/*------------Labview����Controler&Controler����Labview ID--------------------*/
//Controler����ģ��1&����ģ��1 ID,����IDͬΪEEPROM�еĴ洢��ַ

#define Transmit1AndReceive1LabviewHeader		(uint16_t)0x0197		  //Controler����ģ��1&����ģ��1�����ݵı�ͷ		407d

#define ManualOrAutomatic_ModeSelect_ID						0x00 						//�ֶ�Or�Զ�ģʽѡ������ID
#define MotorEnableOrDisable_ID										0x02						//���ʹ��Orʧ������ID
#define ThumbJointMot_TargetSpeed_ID							0x04						//Ĵָ�źϵ��Ŀ��ת������ID
#define ThumbSwingMot_TargetSpeed_ID							0x06						//Ĵָ�ڶ����Ŀ��ת������ID
#define IndexFingerSwingMot_TargetSpeed_ID				0x08						//ʳָ�ڶ����Ŀ��ת������ID
#define MiddleFingerSwingMot_TargetSpeed_ID				0x0A						//��ָ�ڶ����Ŀ��ת������ID
#define RingFingerSwingMot_TargetSpeed_ID					0x0C						//����ָ�ڶ����Ŀ��ת������ID
#define LittleFingerSwingMot_TargetSpeed_ID				0x0E						//Сָ�ڶ����Ŀ��ת������ID
#define PowerUp_DefaultTypesofGestures_ID					0x10						//�ϵ�ʱĬ��������������ID
#define Manual_TargetTypesofGesture_ID						0x12						//�ֶ�ģʽĿ��������������ID
#define Bluetooth_AT_SetMasterMode_Flag_ID				0x14						//ATָ������Masterģʽ��������ID,�洢��ַ��(EE_TYPE-1)
#define Grip_Control_Parameters_P_ID							0x16						//ץ�տ��Ʋ���������ϵ������ID
#define Grip_Control_Parameters_I_ID							0x18						//ץ�տ��Ʋ���������ϵ������ID
#define Grip_Control_Parameters_D_ID							0x1A						//ץ�տ��Ʋ�����΢��ϵ������ID

#define Transmit1AndReceive1Labview_StartAddress_ID						ManualOrAutomatic_ModeSelect_ID		//Controler����ģ��1&����ģ��1�����ݵ��׵�ַ
#define Transmit1AndReceive1Labview_EndAddress_ID							Grip_Control_Parameters_D_ID			//Controler����ģ��1&����ģ��1�����ݵ�ĩ��ַ

#define Transmit1AndReceive1LabviewDataNum				14								//Controler����ģ��1&����ģ��1�����ݵĸ���

/*------------Controler����Labview ID--------------------*/
//Controler����ģ��2 ID
#define Transmit2LabviewHeader							(uint16_t)0x043A		  //Controler����ģ��2�����ݵı�ͷ		1082d

#define EMG_Sensor_Electrode1_ID									0x64 						//���紫�����缫1����ID
#define EMG_Sensor_Electrode2_ID									0x66 						//���紫�����缫2����ID
#define EMG_Sensor_Electrode3_ID									0x68 						//���紫�����缫3����ID
#define EMG_Sensor_Electrode4_ID									0x6A 						//���紫�����缫4����ID
#define EMG_Sensor_Electrode5_ID									0x6C 						//���紫�����缫5����ID
#define EMG_Sensor_Electrode6_ID									0x6E 						//���紫�����缫6����ID
#define EMG_Sensor_Electrode7_ID									0x70 						//���紫�����缫7����ID
#define EMG_Sensor_Electrode8_ID									0x72 						//���紫�����缫8����ID
#define ThumbJointMot_AutoModeSpeed_ID						0x74						//Ĵָ�źϵ���Զ�ģʽת������ID
#define ThumbSwingMot_AutoModeSpeed_ID						0x76						//Ĵָ�ڶ�����Զ�ģʽת������ID
#define IndexFingerSwingMot_AutoModeSpeed_ID			0x78						//ʳָ�ڶ�����Զ�ģʽת������ID
#define MiddleFingerSwingMot_AutoModeSpeed_ID			0x7A						//��ָ�ڶ�����Զ�ģʽת������ID
#define RingFingerSwingMot_AutoModeSpeed_ID				0x7C						//����ָ�ڶ�����Զ�ģʽת������ID
#define LittleFingerSwingMot_AutoModeSpeed_ID			0x7E						//Сָ�ڶ�����Զ�ģʽת������ID
#define ThumbJointMot_Current_ID									0x80						//Ĵָ�źϵ����������ID
#define ThumbSwingMot_Current_ID									0x82						//Ĵָ�ڶ������������ID
#define IndexFingerSwingMot_Current_ID						0x84						//ʳָ�ڶ������������ID
#define MiddleFingerSwingMot_Current_ID						0x86						//��ָ�ڶ������������ID
#define RingFingerSwingMot_Current_ID							0x88						//����ָ�ڶ������������ID
#define LittleFingerSwingMot_Current_ID						0x8A						//Сָ�ڶ������������ID
#define FSR1_NormalPressure_ID										0x8C						//FSR1����ѹ������ID
#define FSR2_NormalPressure_ID										0x8E						//FSR2����ѹ������ID
#define FSR3_NormalPressure_ID										0x90						//FSR3����ѹ������ID
#define FSR4_NormalPressure_ID										0x92						//FSR4����ѹ������ID
#define Thumb_Bending_ID													0x94						//Ĵָ����������ID
#define IndexFinger_Bending_ID										0x96						//ʳָ����������ID
#define MiddleFinger_Bending_ID										0x98						//��ָ����������ID
#define Recognized_TypesofGestures_ID							0x9A						//ʶ���������������ID
#define Bluetooth_PairingStatus_ID								0x9C						//�������״̬����ID

#define Transmit2Labview_StartAddress_ID					EMG_Sensor_Electrode1_ID		//Controler����ģ��2�����ݵ��׵�ַ
#define Transmit2Labview_EndAddress_ID						Bluetooth_PairingStatus_ID	//Controler����ģ��2�����ݵ�ĩ��ַ

#define Transmit2LabviewDataNum										29							//Controler����ģ��2�����ݵĸ���

/*------------Controler����ģ��1&����ģ��1�������--------------------*/
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

volatile extern uint8_t WirelessAnalysis_Data[5];								//���߽������ݻ�����
volatile extern uint8_t WirelessReceice_Data_OneByte;						//���߽������ݻ������

osThreadId Get_Wireless_ReceiveLabview_SendMemTaskHandle(); 		//��ȡWireless_ReceiveLabview_SendMemTaskHandle�������

void Wireless_FREERTOS_Init(void);															//Wirelessͨ���н�������ĺ���

uint8_t Wireless_ReceiveData_Analysis(uint8_t Wireless_ReceiveData,uint8_t *Analysis_Data);	//���߽������ݽ�������

void Wireless_DATA_Send1And2(uint8_t Labview_ID);								//Ͷ��Wireless�������ݵ����еĺ���������ģ��1��2������

//���ݵ��ã�������
void Get_Wireless_DataFrom_EEPROM(Transmit1AndReceive1LabviewData_n name_num,void*extern_data);//ȡֵ����������Wireless���ղ������õ�����,@Transmit1AndReceive1LabviewData_n
void Set_Wireless_DataFrom_EEPROM(Transmit1AndReceive1LabviewData_n name_num,void*extern_data);//�޸�Wireless���ղ������õ�����,@Transmit1AndReceive1LabviewData_n
void FirstPowerOnReadEEPROMInit(void);													//��һ���ϵ��ȡEEPROM������

#endif

