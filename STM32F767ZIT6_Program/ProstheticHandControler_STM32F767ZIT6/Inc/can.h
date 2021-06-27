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
/*------------Faulhaberµç»ú¿ØÖÆÆ÷CANOpenĞ­Òé±¨ÎÄID--------------------*/
#define FaulhaberMotNMTID				0x000			//µç»ú¿ØÖÆÆ÷ ÊÕNMTĞ­Òé±¨ÎÄID
#define FaulhaberMotBootUpID		0x700			//µç»ú¿ØÖÆÆ÷ Boot-UpĞ­Òé±¨ÎÄID 
#define FaulhaberMotPDO1RxID		0x200			//µç»ú¿ØÖÆÆ÷ ÊÕPDO1Ğ­Òé±¨ÎÄID
#define FaulhaberMotPDO1TxID		0x180			//µç»ú¿ØÖÆÆ÷ ·¢PDO1Ğ­Òé±¨ÎÄID
#define FaulhaberMotPDO2RxID		0x300			//µç»ú¿ØÖÆÆ÷ ÊÕPDO2Ğ­Òé±¨ÎÄID
#define FaulhaberMotPDO2TxID		0x280			//µç»ú¿ØÖÆÆ÷ ·¢PDO2Ğ­Òé±¨ÎÄID
#define FaulhaberMotPDO3RxID		0x400			//µç»ú¿ØÖÆÆ÷ ÊÕPDO3Ğ­Òé±¨ÎÄID
#define FaulhaberMotPDO3TxID		0x380			//µç»ú¿ØÖÆÆ÷ ·¢PDO3Ğ­Òé±¨ÎÄID
#define FaulhaberMotSDORxID			0x600			//µç»ú¿ØÖÆÆ÷ ÊÕSDOĞ­Òé±¨ÎÄID
#define FaulhaberMotSDOTxID			0x580			//µç»ú¿ØÖÆÆ÷ ·¢SDOĞ­Òé±¨ÎÄID 	 

/*------------Collection·¢ËÍController±¨ÎÄID¼°±àºÅ¶¨Òå---------------*/
#define	Collection_Controller_INFO0_ID			0x7F0		//CollectionÄ£¿é·¢¸øController±¨ÎÄ1£¬FSR1ºÍFSR2Ñ¹Á¦Êı¾İ
#define	Collection_Controller_INFO1_ID			0x7F1		//CollectionÄ£¿é·¢¸øController±¨ÎÄ2£¬FSR3ºÍFSR4Ñ¹Á¦Êı¾İ
#define	Collection_Controller_INFO2_ID			0x7F2		//CollectionÄ£¿é·¢¸øController±¨ÎÄ3£¬FSR5Ñ¹Á¦ºÍBending1µçÑ¹Êı¾İ
#define	Collection_Controller_INFO3_ID			0x7F3		//CollectionÄ£¿é·¢¸øController±¨ÎÄ4£¬Bending2µçÑ¹ºÍBending3µçÑ¹Êı¾İ
#define	Collection_Controller_INFO4_ID			0x7F4		//CollectionÄ£¿é·¢¸øController±¨ÎÄ5£¬Bending4µçÑ¹ºÍBending5µçÑ¹Êı¾İ
#define	Collection_Controller_INFO5_ID			0x7F5		//CollectionÄ£¿é·¢¸øController±¨ÎÄ6£¬Backup1ºÍBackup2Êı¾İ
/*------------Faulhaberµç»ú¿ØÖÆÆ÷CANOpen½Úµã±àºÅ--------------------*/
#define AllOfNodes												0x00				//¿ØÖÆËùÓĞµç»ú½Úµã£¬²¿·ÖĞ­Òé²»¿ÉÒÔÓÃ£¬²Î¿¼Faulhaberµç»úCanopenĞ­Òé
#define Net_Mot_Node_Num									0x06				//ÍøÂçÖĞµç»ú½ÚµãÊıÁ¿
#define ThumbJointMot_Node								0x01				//Ä´Ö¸ÕÅºÏµç»ú ¿ØÖÆÆ÷ ½ÚµãºÅ
#define ThumbSwingMot_Node								0x02				//Ä´Ö¸°Ú¶¯µç»ú ¿ØÖÆÆ÷ ½ÚµãºÅ
#define IndexFingerSwingMot_Node					0x03				//Ê³Ö¸°Ú¶¯µç»ú ¿ØÖÆÆ÷ ½ÚµãºÅ
#define MiddleFingerSwingMot_Node					0x04				//ÖĞÖ¸°Ú¶¯µç»ú ¿ØÖÆÆ÷ ½ÚµãºÅ
#define RingFingerSwingMot_Node						0x05				//ÎŞÃûÖ¸°Ú¶¯µç»ú ¿ØÖÆÆ÷ ½ÚµãºÅ
#define LittleFingerSwingMot_Node					0x06				//Ğ¡Ö¸°Ú¶¯µç»ú ¿ØÖÆÆ÷ ½ÚµãºÅ	 
/*------------Faulhaberµç»ú¿ØÖÆÆ÷CANOpen¹¦ÄÜÖ¸ÁîCMD--------------------*/
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

/*------------Faulhaberµç»ú·´À¡Êı¾İĞòºÅ--------------------*/
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

/*------------Faulhaberµç»ú·´À¡×´Ì¬Êı¾İĞòºÅ--------------------*/
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
}Faulhaber_StatusData_n;//Faulhaberµç»ú·´À¡×´Ì¬Êı¾İ

/*------------¼ÙÖ«ÊÖÍâ½Ó´«¸ĞÆ÷Êı¾İĞòºÅ--------------------*/
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
}CAN1TransmitMessage3Typedef;				//CAN1¶ÓÁĞ3·¢ËÍÊı¾İÀàĞÍ

typedef struct{
	uint32_t CANTransmitID;
	uint8_t CANTransmitData[2];
}CAN1TransmitMessage1Typedef;				//CAN1¶ÓÁĞ1·¢ËÍÊı¾İÀàĞÍ

typedef struct{
	uint32_t CANTransmitID;
	uint8_t CANTransmit_DataBytes;
	uint16_t CANTransmit_CMDIndex;
	uint8_t CANTransmit_SubIndex;
	uint32_t CANTransmitData;
}CAN1TransmitMessage2Typedef;				//CAN1¶ÓÁĞ2·¢ËÍÊı¾İÀàĞÍ

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
osThreadId Get_CAN1TransmitQueueHandle(void);							//»ñÈ¡CAN1·¢ËÍ¶ÓÁĞ¾ä±ú
void APP_CAN_Config(CAN_HandleTypeDef* canHandle);				//CAN¹ıÂËÆ÷ÅäÖÃº¯Êı

//½ö²âÊÔÓÃ£¨Î´Í¨¹ı¶ÓÁĞ·¢ËÍÊı¾İ£
void Faulhaber_CAN1_PDO1Send_Mot(uint16_t PDO1_Mot_CMD,uint8_t Mot_Node);				//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıPDO1Ğ­Òé·¢ËÍÊı¾İ¸øµç»ú¿ØÖÆÆ÷
void Faulhaber_CAN1_PDO1Enable_Mot(uint8_t Mot_Node);						//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıPDO1Ğ­ÒéÊ¹ÄÜµç»ú
void Faulhaber_CAN1_PDO1Disable_Mot(uint8_t Mot_Node);					//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıPDO1Ğ­ÒéÊ§ÄÜµç»ú
uint32_t Calculate_Complement32(float data);										//¼ÆËã²¹Âëº¯Êı
uint32_t Faulhaber_Mot_SpeedProcess(float Input_TargetSpeed);		//¼ÙÖ«ÊÖµç»úËÙ¶È´¦Àí

//ÍÆ¼öÊ¹ÓÃÒÔÏÂº¯ÊıÊ¹ÄÜºÍ·¢ËÍÃüÁî£¨ÏÂÁĞº¯ÊıÍ¨¹ı¶ÓÁĞ·¢ËÍÊı¾İ£©
void CAN1_NMTSend_Mot(uint32_t Mot_StdId,uint8_t Mot_CS,uint8_t Mot_Node);				//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıNMTĞ­Òé·¢ËÍÊı¾İ¸øµç»ú¿ØÖÆÆ÷(¶ÓÁĞ·¢ËÍ)
void CAN1_BootUpSend_Mot(uint32_t Mot_StdId,uint8_t Bootup_cmd,uint8_t Mot_Node);	//¼ÙÖ«ÊÖ¿ØÖÆÆ÷·¢ËÍÊı¾İÊ¹µç»úBoot_Up(¶ÓÁĞ·¢ËÍ)
void Faulhaber_CAN1_Init_Mot(void);		//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıNMTĞ­Òé¼°Boot_UpÃüÁî³õÊ¼»¯µç»ú(¶ÓÁĞ·¢ËÍ£¬ÄÚº¬ÈÎÎñµ÷¶ÈÆ÷)
void CAN1_DataPDO2Send(uint32_t Mot_StdId,uint8_t PDO2_Mot_CMD,uint32_t PDO2_Cmd_Data);	//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıPDO2Ğ­Òé·¢ËÍÊı¾İ¸øµç»ú¿ØÖÆÆ÷(¶ÓÁĞ·¢ËÍ)
void Faulhaber_CAN1_PDO2Enable_Mot(uint8_t Mot_Node);																		//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıPDO2Ğ­ÒéÊ¹ÄÜµç»ú(¶ÓÁĞ·¢ËÍ)
void Faulhaber_CAN1_PDO2Disable_Mot(uint8_t Mot_Node);																	//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıPDO2Ğ­ÒéÊ§ÄÜµç»ú(¶ÓÁĞ·¢ËÍ)
void CAN1_DATA_SDOSend(uint32_t Mot_StdId,uint8_t WriteODEntries,uint16_t Mot_CMD_Index,uint8_t Sub_Index,uint32_t Mot_Data);	//CAN1 SDO Ğ­Òé·¢ËÍÊı¾İº¯Êı(¶ÓÁĞ·¢ËÍ)
void Faulhaber_CAN1_SDOEnable_Mot(uint8_t Mot_Node);		//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıSDOĞ­ÒéÊ¹ÄÜµç»ú(¶ÓÁĞ·¢ËÍ£¬ÄÚº¬ÈÎÎñµ÷¶ÈÆ÷)
void Faulhaber_CAN1_SDODisable_Mot(uint8_t Mot_Node);															//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıSDOĞ­ÒéÊ§ÄÜµç»ú(¶ÓÁĞ·¢ËÍ)
void Faulhaber_CAN1_SDOSet_Mot_OperatingMode(uint8_t Mot_Node,uint32_t Mot_OperatingMode);	//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıSDOĞ­ÒéÉèÖÃµç»ú¿ØÖÆÄ£Ê½(¶ÓÁĞ·¢ËÍ)
void Faulhaber_CAN1_SDOSet_MotVelocity(uint8_t Mot_Node,uint32_t MotVelocity);		//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıSDOĞ­ÒéÉèÖÃµç»úËÙ¶È(¶ÓÁĞ·¢ËÍ)
void Faulhaber_CAN1_SDORequest_Mot_CurrentValue(uint8_t Mot_Node);//¼ÙÖ«ÊÖ¿ØÖÆÆ÷Í¨¹ıSDOĞ­ÒéÇëÇó·´À¡µç»úµ±Ç°×ªËÙ(¶ÓÁĞ·¢ËÍ)
	
//Êı¾İµ÷ÓÃ£¬´¦Àíº¯Êı
void Get_Faulhaber_FeedbackData(Faulhaber_FeedbackData_n name_num,void*extern_data);//È¡Öµº¯Êı£¬°áÔËFaulhaber·´À¡Êı¾İ,@Faulhaber_FeedbackData_n
void Set_Faulhaber_FeedbackData(Faulhaber_FeedbackData_n name_num,void*extern_data);//ĞŞ¸ÄFaulhaber·´À¡Êı¾İ,@Faulhaber_FeedbackData_n
void Get_Faulhaber_StatusData(Faulhaber_StatusData_n name_num,void*extern_data);//È¡Öµº¯Êı£¬°áÔËFaulhaber×´Ì¬Êı¾İ,@Faulhaber_StatusData_n
void Set_Faulhaber_StatusData(Faulhaber_StatusData_n name_num,void*extern_data);//È¡Öµº¯Êı£¬°áÔËFaulhaber×´Ì¬Êı¾İ,@Faulhaber_StatusData_n
void Get_ProstheticHand_SensorData(ProstheticHand_SensorData_n name_num,void*extern_data);//È¡Öµº¯Êı£¬°áÔË¼ÙÖ«ÊÖÍâ½Ó´«¸ĞÆ÷Êı¾İ,@ProstheticHand_SensorData_n
void Set_ProstheticHand_SensorData(ProstheticHand_SensorData_n name_num,void*extern_data);//ĞŞ¸Ä¼ÙÖ«ÊÖÍâ½Ó´«¸ĞÆ÷Êı¾İ,@ProstheticHand_SensorData_n

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
