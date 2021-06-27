/**
  ******************************************************************************
  * File Name          : CAN.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
	 
#include "arm_math.h" 
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */
/*------------Faulhaber���������CANOpenЭ�鱨��ID--------------------*/
#define FaulhaberMotNMTID				0x000			//��������� ��NMTЭ�鱨��ID
#define FaulhaberMotBootUpID		0x700			//��������� Boot-UpЭ�鱨��ID 
#define FaulhaberMotPDO1RxID		0x200			//��������� ��PDO1Э�鱨��ID
#define FaulhaberMotPDO1TxID		0x180			//��������� ��PDO1Э�鱨��ID
#define FaulhaberMotPDO2RxID		0x300			//��������� ��PDO2Э�鱨��ID
#define FaulhaberMotPDO2TxID		0x280			//��������� ��PDO2Э�鱨��ID
#define FaulhaberMotPDO3RxID		0x400			//��������� ��PDO3Э�鱨��ID
#define FaulhaberMotPDO3TxID		0x380			//��������� ��PDO3Э�鱨��ID
#define FaulhaberMotSDORxID			0x600			//��������� ��SDOЭ�鱨��ID
#define FaulhaberMotSDOTxID			0x580			//��������� ��SDOЭ�鱨��ID 	 

/*------------Collection����Controller����ID����Ŷ���---------------*/
#define	Collection_Controller_INFO0_ID			0x7F0		//Collectionģ�鷢��Controller����1��FSR1��FSR2ѹ������
#define	Collection_Controller_INFO1_ID			0x7F1		//Collectionģ�鷢��Controller����2��FSR3��FSR4ѹ������
#define	Collection_Controller_INFO2_ID			0x7F2		//Collectionģ�鷢��Controller����3��FSR5ѹ����Bending1��ѹ����
#define	Collection_Controller_INFO3_ID			0x7F3		//Collectionģ�鷢��Controller����4��Bending2��ѹ��Bending3��ѹ����
#define	Collection_Controller_INFO4_ID			0x7F4		//Collectionģ�鷢��Controller����5��Bending4��ѹ��Bending5��ѹ����
#define	Collection_Controller_INFO5_ID			0x7F5		//Collectionģ�鷢��Controller����6��Backup1��Backup2����
/*------------Faulhaber���������CANOpen�ڵ���--------------------*/
#define AllOfNodes												0x00				//�������е���ڵ㣬����Э�鲻�����ã��ο�Faulhaber���CanopenЭ��
#define Net_Mot_Node_Num									0x06				//�����е���ڵ�����
#define ThumbJointMot_Node								0x01				//Ĵָ�źϵ�� ������ �ڵ��
#define ThumbSwingMot_Node								0x02				//Ĵָ�ڶ���� ������ �ڵ��
#define IndexFingerSwingMot_Node					0x03				//ʳָ�ڶ���� ������ �ڵ��
#define MiddleFingerSwingMot_Node					0x04				//��ָ�ڶ���� ������ �ڵ��
#define RingFingerSwingMot_Node						0x05				//����ָ�ڶ���� ������ �ڵ��
#define LittleFingerSwingMot_Node					0x06				//Сָ�ڶ���� ������ �ڵ��	 
/*------------Faulhaber���������CANOpen����ָ��CMD--------------------*/
//Index
#define FahlhaberMot_CIA402_Controlword						0x6040			//CIA402_Controlword	@FaulhaberMotSDORxID		
#define FahlhaberMot_CIA402_OperationModes				0x6060			//CIA402_OperationModes	@FaulhaberMotSDORxID
#define FahlhaberMot_CIA402_VelocityActualSensorValue		0x6069			//CIA402_VelocityActualSensorValue	@FaulhaberMotSDORxID	
#define FahlhaberMot_CIA402_TargetVelocity				0x60FF			//CIA402_TargetVelocity	@FaulhaberMotSDORxID	
#define FahlhaberMot_CIA402_VelocityActualValue		0x606C			//CIA402_VelocityActualValue	@FaulhaberMotSDORxID	
//CMD
#define FaulhaberMotNMT_CS_StartRemoteNode				0x01				//Start Remote Node @FaulhaberMotNMTID
#define FaulhaberMot_BootUp_Messege								0x00				//Boot_Up	@FaulhaberMotBootUpID
#define FaulhaberMot_FaultReset										0x0080			//Fault reset	@FaulhaberMotPDO1RxID	or @FaulhaberMotSDORxID
#define FaulhaberMot_ShutDown											0x0006			//Shutdown	@FaulhaberMotPDO1RxID	or @FaulhaberMotSDORxID
#define FaulhaberMot_SwitchOn											0x0007			//Switch on	@FaulhaberMotPDO1RxID	or @FaulhaberMotSDORxID
#define FaulhaberMot_EnableOperation							0x000F			//Enable operation	@FaulhaberMotPDO1RxID	or @FaulhaberMotSDORxID
#define FaulhaberMot_SwitchOffOutputStage					0x000D			//Switch Off	@FaulhaberMotPDO1RxID	or @FaulhaberMotSDORxID
#define FaulhaberMot_PDO2EnableCMD								0x0F				//Enable	@FaulhaberMotPDO2RxID
#define FaulhaberMot_PDO2DisableCMD								0x08				//Disable	@FaulhaberMotPDO2RxID
#define FaulhaberMot_PDO1NMTSuccessData						(uint16_t)0x0060			//NMTSuccessFull	@FaulhaberMotPDO1TxID
#define FaulhaberMot_BootupSuccessData						(uint8_t)0x05				//BootupSuccessFull	@FaulhaberMotBootUpID

//Operating Modes
#define FaulhaberMot_NotActivate									0x00				//Controller Not Activate,cmd
#define FaulhaberMot_PositionControl							0x01				//Position Control,cmd
#define FaulhaberMot_VelocityControl							0x03				//Velocity Control,cmd
#define FaulhaberMot_Homing												0x06				//Homing,cmd
#define FaulhaberMot_CyclingSynchronousPosition		0x08				//cycling synchronous position,cmd
//Data bytes control
#define FaulhaberMot_ReadODEntries								0x40				//ReadODEntries,Upload-Request	@FaulhaberMotSDORxID
#define FaulhaberMot_UploadResponse1							0x4F				//UploadResponse,1 data bytes	@FaulhaberMotSDOTxID
#define FaulhaberMot_UploadResponse2							0x4B				//UploadResponse,2 data bytes	@FaulhaberMotSDOTxID
#define FaulhaberMot_UploadResponse3							0x47				//UploadResponse,3 data bytes	@FaulhaberMotSDOTxID
#define FaulhaberMot_UploadResponse4							0x43				//UploadResponse,4 data bytes	@FaulhaberMotSDOTxID
#define FaulhaberMot_WriteODEntries1							0x2F				//WriteODEntries,1 data bytes	@FaulhaberMotSDORxID
#define FaulhaberMot_WriteODEntries2							0x2B				//WriteODEntries,2 data bytes	@FaulhaberMotSDORxID
#define FaulhaberMot_WriteODEntries3							0x27				//WriteODEntries,3 data bytes	@FaulhaberMotSDORxID
#define FaulhaberMot_WriteODEntries4							0x23				//WriteODEntries,4 data bytes	@FaulhaberMotSDORxID

/*------------Faulhaber��������������--------------------*/
typedef enum{
	ThumbJointMot_AutoModeSpeed_n=1,
	ThumbSwingMot_AutoModeSpeed_n,
	IndexFingerSwingMot_AutoModeSpeed_n,
	MiddleFingerSwingMot_AutoModeSpeed_n,
	RingFingerSwingMot_AutoModeSpeed_n,
	LittleFingerSwingMot_AutoModeSpeed_n,
	ThumbJointMot_Current_n,
	ThumbSwingMot_Current_n,
	IndexFingerSwingMot_Current_n,
	MiddleFingerSwingMot_Current_n,
	RingFingerSwingMot_Current_n,
	LittleFingerSwingMot_Current_n,
}Faulhaber_FeedbackData_n;

/*------------Faulhaber�������״̬�������--------------------*/
typedef enum{
	ThumbJointMot_NMT_Status_n=100,
	ThumbSwingMot_NMT_Status_n,
	IndexFingerSwingMot_NMT_Status_n,
	MiddleFingerSwingMot_NMT_Status_n,
	RingFingerSwingMot_NMT_Status_n,
	LittleFingerSwingMot_NMT_Status_n,
	ThumbJointMot_Bootup_Status_n,
	ThumbSwingMot_Bootup_Status_n,
	IndexFingerSwingMot_Bootup_Status_n,
	MiddleFingerSwingMot_Bootup_Status_n,
	RingFingerSwingMot_Bootup_Status_n,
	LittleFingerSwingMot_Bootup_Status_n,
}Faulhaber_StatusData_n;//Faulhaber�������״̬����

/*------------��֫����Ӵ������������--------------------*/
typedef enum {
	FSR1_NormalPressure_n=50,
	FSR2_NormalPressure_n,
	FSR3_NormalPressure_n,
	FSR4_NormalPressure_n,
	FSR5_NormalPressure_n,
	Thumb_Bending_n,													
	IndexFinger_Bending_n,										
	MiddleFinger_Bending_n,
	RingFinger_Bending_n,										
	LittleFinger_Bending_n,
	ADC_Backup1_n,
	ADC_Backup2_n,
}ProstheticHand_SensorData_n;

typedef struct{
	uint32_t CANTransmitID;
	uint8_t CANTransmit_CMD;
	uint32_t CANTransmitData
}CAN1TransmitMessage3Typedef;				//CAN1����3������������

typedef struct{
	uint32_t CANTransmitID;
	uint8_t CANTransmitData[2];
}CAN1TransmitMessage1Typedef;				//CAN1����1������������

typedef struct{
	uint32_t CANTransmitID;
	uint8_t CANTransmit_DataBytes;
	uint16_t CANTransmit_CMDIndex;
	uint8_t CANTransmit_SubIndex;
	uint32_t CANTransmitData;
}CAN1TransmitMessage2Typedef;				//CAN1����2������������

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
osThreadId Get_CAN1TransmitQueueHandle(void);							//��ȡCAN1���Ͷ��о��
void APP_CAN_Config(CAN_HandleTypeDef* canHandle);				//CAN���������ú���

//�������ã�δͨ�����з������ݣ
void Faulhaber_CAN1_PDO1Send_Mot(uint16_t PDO1_Mot_CMD,uint8_t Mot_Node);				//��֫�ֿ�����ͨ��PDO1Э�鷢�����ݸ����������
void Faulhaber_CAN1_PDO1Enable_Mot(uint8_t Mot_Node);						//��֫�ֿ�����ͨ��PDO1Э��ʹ�ܵ��
void Faulhaber_CAN1_PDO1Disable_Mot(uint8_t Mot_Node);					//��֫�ֿ�����ͨ��PDO1Э��ʧ�ܵ��
uint32_t Calculate_Complement32(float data);										//���㲹�뺯��
uint32_t Faulhaber_Mot_SpeedProcess(float Input_TargetSpeed);		//��֫�ֵ���ٶȴ���

//�Ƽ�ʹ�����º���ʹ�ܺͷ���������к���ͨ�����з������ݣ�
void CAN1_NMTSend_Mot(uint32_t Mot_StdId,uint8_t Mot_CS,uint8_t Mot_Node);				//��֫�ֿ�����ͨ��NMTЭ�鷢�����ݸ����������(���з���)
void CAN1_BootUpSend_Mot(uint32_t Mot_StdId,uint8_t Bootup_cmd,uint8_t Mot_Node);	//��֫�ֿ�������������ʹ���Boot_Up(���з���)
void Faulhaber_CAN1_Init_Mot(void);		//��֫�ֿ�����ͨ��NMTЭ�鼰Boot_Up�����ʼ�����(���з��ͣ��ں����������)
void CAN1_DataPDO2Send(uint32_t Mot_StdId,uint8_t PDO2_Mot_CMD,uint32_t PDO2_Cmd_Data);	//��֫�ֿ�����ͨ��PDO2Э�鷢�����ݸ����������(���з���)
void Faulhaber_CAN1_PDO2Enable_Mot(uint8_t Mot_Node);																		//��֫�ֿ�����ͨ��PDO2Э��ʹ�ܵ��(���з���)
void Faulhaber_CAN1_PDO2Disable_Mot(uint8_t Mot_Node);																	//��֫�ֿ�����ͨ��PDO2Э��ʧ�ܵ��(���з���)
void CAN1_DATA_SDOSend(uint32_t Mot_StdId,uint8_t WriteODEntries,uint16_t Mot_CMD_Index,uint8_t Sub_Index,uint32_t Mot_Data);	//CAN1 SDO Э�鷢�����ݺ���(���з���)
void Faulhaber_CAN1_SDOEnable_Mot(uint8_t Mot_Node);		//��֫�ֿ�����ͨ��SDOЭ��ʹ�ܵ��(���з��ͣ��ں����������)
void Faulhaber_CAN1_SDODisable_Mot(uint8_t Mot_Node);															//��֫�ֿ�����ͨ��SDOЭ��ʧ�ܵ��(���з���)
void Faulhaber_CAN1_SDOSet_Mot_OperatingMode(uint8_t Mot_Node,uint32_t Mot_OperatingMode);	//��֫�ֿ�����ͨ��SDOЭ�����õ������ģʽ(���з���)
void Faulhaber_CAN1_SDOSet_MotVelocity(uint8_t Mot_Node,uint32_t MotVelocity);		//��֫�ֿ�����ͨ��SDOЭ�����õ���ٶ�(���з���)
void Faulhaber_CAN1_SDORequest_Mot_CurrentValue(uint8_t Mot_Node);//��֫�ֿ�����ͨ��SDOЭ�������������ǰת��(���з���)
	
//���ݵ��ã�������
void Get_Faulhaber_FeedbackData(Faulhaber_FeedbackData_n name_num,void*extern_data);//ȡֵ����������Faulhaber��������,@Faulhaber_FeedbackData_n
void Set_Faulhaber_FeedbackData(Faulhaber_FeedbackData_n name_num,void*extern_data);//�޸�Faulhaber��������,@Faulhaber_FeedbackData_n
void Get_Faulhaber_StatusData(Faulhaber_StatusData_n name_num,void*extern_data);//ȡֵ����������Faulhaber״̬����,@Faulhaber_StatusData_n
void Set_Faulhaber_StatusData(Faulhaber_StatusData_n name_num,void*extern_data);//ȡֵ����������Faulhaber״̬����,@Faulhaber_StatusData_n
void Get_ProstheticHand_SensorData(ProstheticHand_SensorData_n name_num,void*extern_data);//ȡֵ���������˼�֫����Ӵ���������,@ProstheticHand_SensorData_n
void Set_ProstheticHand_SensorData(ProstheticHand_SensorData_n name_num,void*extern_data);//�޸ļ�֫����Ӵ���������,@ProstheticHand_SensorData_n

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
