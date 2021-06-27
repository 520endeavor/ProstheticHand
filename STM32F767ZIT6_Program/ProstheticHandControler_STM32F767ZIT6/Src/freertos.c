/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"
#include "AT24Cxx.h" 
#include "LTC1867.h"	
#include "arm_math.h" 
#include "bluetooth.h"
#include "Bluetooth_Operating.h"
#include "Gestures_Recognized.h"
#include "MFAC_Controller.h"
#include "HostComputer_Communication.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId FaulhaberMot_NMT_BootupTaskHandle;
osThreadId FaulhaberMot_Power_EnableAndInitTaskHandle;
osThreadId FaulhaberMot_UpRequestTaskHandle;
osThreadId FaulhaberMot_SelectControlModeTaskHandle;
osThreadId FaulhaberMot_ManualControlTaskHandle;
osThreadId FaulhaberMot_AutoControlTaskHandle;
osThreadId GripForce_PIDControlTaskHandle;
osThreadId GripForce_MFACControlTaskHandle;
osThreadId LTC1867_GetdataTaskHandle;
osThreadId RGB_PowerOn_StartUp_TaskHandle;
osThreadId RGB_Run_TaskHandle;

osMessageQId myQueue01Handle;

/* USER CODE BEGIN Variables */
volatile float GripForce_pid_Out=0;	
const float GripForce_pid_Adjust_Ability=6000.0;
volatile float GripForce_MFAC_Out=0;	
const float GripForce_MFAC_Adjust_Ability=60000.0/10.0/1024.0;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void FaulhaberMot_NMT_BootupTask(void const * argument);
void FaulhaberMot_Power_EnableAndInitTask(void const * argument);
void FaulhaberMot_UpRequestTask(void const * argument);
void FaulhaberMot_SelectControlModeTask(void const * argument);
void FaulhaberMot_ManualControlTask(void const * argument);
void FaulhaberMot_AutoControlTask(void const * argument);
void GripForce_PIDControlTask(void const * argument);
void GripForce_MFACControlTask(void const * argument);
void LTC1867_GetdataTask(void const * argument);
void RGB_PowerOn_StartUp_Task(void const * argument);
void RGB_Run_Task(void const * argument);

extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
//  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
	/* definition and creation of FaulhaberMot_NMT_BootupTask */
  osThreadDef(FaulhaberMot_NMT_BootupTask, FaulhaberMot_NMT_BootupTask, osPriorityNormal, 0, 256);
  FaulhaberMot_NMT_BootupTaskHandle = osThreadCreate(osThread(FaulhaberMot_NMT_BootupTask), NULL);
	/* definition and creation of LTC1867_GetdataTask */
  osThreadDef(LTC1867_GetdataTask, LTC1867_GetdataTask, osPriorityNormal, 0, 128);
  LTC1867_GetdataTaskHandle = osThreadCreate(osThread(LTC1867_GetdataTask), NULL);
	/* definition and creation of RGB_PowerOn_StartUp_Task */
  osThreadDef(RGB_PowerOn_StartUp_Task, RGB_PowerOn_StartUp_Task, osPriorityNormal, 0, 128);
  RGB_PowerOn_StartUp_TaskHandle = osThreadCreate(osThread(RGB_PowerOn_StartUp_Task), NULL);
	
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
//  osMessageQDef(myQueue01, 16, uint16_t);
//  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

uint16_t Memery_Buffer[5]={14,23,32,41,50};
uint16_t Read_Buffer[100];
FRESULT SRAM_res=FR_DISK_ERR;
/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	HAL_SRAM_Write_16b(&hsram1, (uint32_t *)Bank1_SRAM1_ADDR, Memery_Buffer, 5);
	HAL_SRAM_Read_16b(&hsram1, (uint32_t *)Bank1_SRAM1_ADDR, Read_Buffer, 5);
  /* init code for FATFS */
  MX_FATFS_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */	
//	SRAM_res=f_mount(&SRAMDISKFatFS,SRAMDISKPath,1); 		//挂载SRAM
  for(;;)
  {    
		osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* FaulhaberMot_NMT_BootupTask function */
void FaulhaberMot_NMT_BootupTask(void const * argument)
{
  /* USER CODE BEGIN FaulhaberMot_NMT_BootupTask */
  /* Infinite loop */	
	osDelay(2000);											//延时等待电机功率器件上电
	uint8_t count=0;
	for(;;)
  {    
		count++;	
		Faulhaber_CAN1_Init_Mot();				//电机初始化函数		
		if(count>10){
			/* definition and creation of FaulhaberMot_Power_EnableAndInitTask */
			osThreadDef(FaulhaberMot_Power_EnableAndInitTask, FaulhaberMot_Power_EnableAndInitTask, osPriorityNormal, 0, 128);
			FaulhaberMot_Power_EnableAndInitTaskHandle = osThreadCreate(osThread(FaulhaberMot_Power_EnableAndInitTask), NULL);
			vTaskDelete( NULL );
		}
		osDelay(10);
  }
  /* USER CODE END FaulhaberMot_NMT_BootupTask */
}

/* FaulhaberMot_Power_EnableAndInitTask function */
void FaulhaberMot_Power_EnableAndInitTask(void const * argument)
{
  /* USER CODE BEGIN FaulhaberMot_Power_EnableAndInitTask */
  /* Infinite loop */
	uint8_t count=0;
	uint8_t i=0;
  for(;;)
  {		
		if(i==5){
			i=0;
			count++;
		}
		Faulhaber_CAN1_SDOEnable_Mot(ThumbSwingMot_Node+i);
		Faulhaber_CAN1_SDOSet_Mot_OperatingMode(ThumbSwingMot_Node+i,FaulhaberMot_VelocityControl);					//拇指摆动电机
		i++;
		if(count==5){
			/* definition and creation of FaulhaberMot_UpRequestTask */
			osThreadDef(FaulhaberMot_UpRequestTask, FaulhaberMot_UpRequestTask, osPriorityNormal, 0, 128);
			FaulhaberMot_UpRequestTaskHandle = osThreadCreate(osThread(FaulhaberMot_UpRequestTask), NULL);
	//		/* definition and creation of FaulhaberMot_SelectControlModeTask */
	//		osThreadDef(FaulhaberMot_SelectControlModeTask, FaulhaberMot_SelectControlModeTask, osPriorityNormal, 0, 128);
	//		FaulhaberMot_SelectControlModeTaskHandle = osThreadCreate(osThread(FaulhaberMot_SelectControlModeTask), NULL);
	//		/* definition and creation of FaulhaberMot_ManualControlTask */
	//		osThreadDef(FaulhaberMot_ManualControlTask, FaulhaberMot_ManualControlTask, osPriorityNormal, 0, 128);
	//		FaulhaberMot_ManualControlTaskHandle = osThreadCreate(osThread(FaulhaberMot_ManualControlTask), NULL);
			/* definition and creation of FaulhaberMot_AutoControlTask */
			osThreadDef(FaulhaberMot_AutoControlTask, FaulhaberMot_AutoControlTask, osPriorityNormal, 0, 128);
			FaulhaberMot_AutoControlTaskHandle = osThreadCreate(osThread(FaulhaberMot_AutoControlTask), NULL);
			/* definition and creation of GripForce_PIDControlTask */
			osThreadDef(GripForce_PIDControlTask, GripForce_PIDControlTask, osPriorityNormal, 0, 256);
			GripForce_PIDControlTaskHandle = osThreadCreate(osThread(GripForce_PIDControlTask), NULL);
			/* definition and creation of GripForce_MFACControlTask */
			osThreadDef(GripForce_MFACControlTask, GripForce_MFACControlTask, osPriorityNormal, 0, 256);
			GripForce_MFACControlTaskHandle = osThreadCreate(osThread(GripForce_MFACControlTask), NULL);
			vTaskDelete( NULL );	
		}
		osDelay(10);
  }
  /* USER CODE END FaulhaberMot_Power_EnableAndInitTask */
}

/* FaulhaberMot_UpRequestTask function */
void FaulhaberMot_UpRequestTask(void const * argument)
{
  /* USER CODE BEGIN FaulhaberMot_UpRequestTask */
  /* Infinite loop */
  for(;;)
  {		
//		Faulhaber_CAN1_SDORequest_Mot_CurrentValue(ThumbSwingMot_Node);		//请求转速信息
																																			//请求电流信息
		osDelay(20);
  }
  /* USER CODE END FaulhaberMot_UpRequestTask */
}

/* FaulhaberMot_SelectControlModeTask function */
void FaulhaberMot_SelectControlModeTask(void const * argument)
{
  /* USER CODE BEGIN FaulhaberMot_SelectControlModeTask */
  /* Infinite loop */
	uint16_t ManualOrAutomatic_Mode=0;
	uint8_t Manual_Mode_lock=0;
  for(;;)
  {		
		Get_Wireless_DataFrom_EEPROM(ManualOrAutomatic_ModeSelect_n,&ManualOrAutomatic_Mode);
		if(Manual_Mode_lock==0&&ManualOrAutomatic_Mode==1){			//手动模式		
			Manual_Mode_lock=1;			
			vTaskSuspend(FaulhaberMot_AutoControlTaskHandle);
			vTaskResume(FaulhaberMot_ManualControlTask);
		}
		else if(Manual_Mode_lock==1&&ManualOrAutomatic_Mode==0){	//自动模式
			Manual_Mode_lock=0;
			vTaskSuspend(FaulhaberMot_ManualControlTask);
			vTaskResume(FaulhaberMot_AutoControlTaskHandle);
			//使能电机
			Faulhaber_CAN1_SDOEnable_Mot(ThumbJointMot_Node);					//拇指张合电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(ThumbSwingMot_Node);					//拇指摆动电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(IndexFingerSwingMot_Node);		//食指摆动电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(MiddleFingerSwingMot_Node);	//中指摆动电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(RingFingerSwingMot_Node);		//无名指指摆动电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(LittleFingerSwingMot_Node);	//小指摆动电机
		}
		osDelay(20);
  }
  /* USER CODE END FaulhaberMot_SelectControlModeTask */
}

/* FaulhaberMot_ManualControlTask function */
void FaulhaberMot_ManualControlTask(void const * argument)
{
  /* USER CODE BEGIN FaulhaberMot_ManualControlTask */
  /* Infinite loop */
	uint16_t MotorEnableOrDisable_Status=0,Clear_MotorEnableOrDisable_Status=0;
	uint8_t MotorEnableLock=0;
	float ThumbJointMot_ManualSpeed=0,ThumbSwingMot_ManualSpeed=0,IndexFingerSwingMot_ManualSpeed=0;			
	float MiddleFingerSwingMot_ManualSpeed=0,RingFingerSwingMot_ManualSpeed=0,LittleFingerSwingMot_ManualSpeed=0;
  for(;;)
  {		
		Get_Wireless_DataFrom_EEPROM(MotorEnableOrDisable_n,&MotorEnableOrDisable_Status);
		if(MotorEnableLock==0&&MotorEnableOrDisable_Status==1){
			MotorEnableLock=1;
			//使能电机
			Faulhaber_CAN1_SDOEnable_Mot(ThumbJointMot_Node);					//拇指张合电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(ThumbSwingMot_Node);					//拇指张合电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(IndexFingerSwingMot_Node);		//食指摆动电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(MiddleFingerSwingMot_Node);	//中指摆动电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(RingFingerSwingMot_Node);		//无名指指摆动电机
				osDelay(10);
			Faulhaber_CAN1_SDOEnable_Mot(LittleFingerSwingMot_Node);	//小指摆动电机
				osDelay(10);			
		}
		else if(MotorEnableLock==1&&MotorEnableOrDisable_Status==0){
			//失能电机
			Faulhaber_CAN1_SDODisable_Mot(ThumbJointMot_Node);					//拇指张合电机
			Faulhaber_CAN1_SDODisable_Mot(ThumbSwingMot_Node);					//拇指张合电机
			Faulhaber_CAN1_SDODisable_Mot(IndexFingerSwingMot_Node);		//食指摆动电机
			Faulhaber_CAN1_SDODisable_Mot(MiddleFingerSwingMot_Node);		//中指摆动电机
			Faulhaber_CAN1_SDODisable_Mot(RingFingerSwingMot_Node);			//无名指指摆动电机
			Faulhaber_CAN1_SDODisable_Mot(LittleFingerSwingMot_Node);		//小指摆动电机
			osDelay(10);
			MotorEnableLock=0;
		}
		if(MotorEnableLock==1){
			Get_Wireless_DataFrom_EEPROM(ThumbJointMot_TargetSpeed_n,&ThumbJointMot_ManualSpeed);
			Get_Wireless_DataFrom_EEPROM(ThumbSwingMot_TargetSpeed_n,&ThumbSwingMot_ManualSpeed);
			Get_Wireless_DataFrom_EEPROM(IndexFingerSwingMot_TargetSpeed_n,&IndexFingerSwingMot_ManualSpeed);
			Get_Wireless_DataFrom_EEPROM(MiddleFingerSwingMot_TargetSpeed_n,&MiddleFingerSwingMot_ManualSpeed);
			Get_Wireless_DataFrom_EEPROM(RingFingerSwingMot_TargetSpeed_n,&RingFingerSwingMot_ManualSpeed);
			Get_Wireless_DataFrom_EEPROM(LittleFingerSwingMot_TargetSpeed_n,&LittleFingerSwingMot_ManualSpeed);

			Faulhaber_CAN1_SDOSet_MotVelocity(ThumbJointMot_Node,Faulhaber_Mot_SpeedProcess(ThumbJointMot_ManualSpeed));								//拇指张合电机
			Faulhaber_CAN1_SDOSet_MotVelocity(ThumbSwingMot_Node,Faulhaber_Mot_SpeedProcess(ThumbSwingMot_ManualSpeed));								//拇指张合电机
			Faulhaber_CAN1_SDOSet_MotVelocity(IndexFingerSwingMot_Node,Faulhaber_Mot_SpeedProcess(IndexFingerSwingMot_ManualSpeed));		//食指摆动电机
			Faulhaber_CAN1_SDOSet_MotVelocity(MiddleFingerSwingMot_Node,Faulhaber_Mot_SpeedProcess(MiddleFingerSwingMot_ManualSpeed));	//中指摆动电机
			Faulhaber_CAN1_SDOSet_MotVelocity(RingFingerSwingMot_Node,Faulhaber_Mot_SpeedProcess(RingFingerSwingMot_ManualSpeed));			//无名指指摆动电机
			Faulhaber_CAN1_SDOSet_MotVelocity(LittleFingerSwingMot_Node,Faulhaber_Mot_SpeedProcess(LittleFingerSwingMot_ManualSpeed));	//小指摆动电机
		}

		osDelay(20);
  }
  /* USER CODE END FaulhaberMot_ManualControlTask */
}
uint32_t TargetVelocity=0;
/* FaulhaberMot_AutoControlTask function */
void FaulhaberMot_AutoControlTask(void const * argument)
{
  /* USER CODE BEGIN FaulhaberMot_AutoControlTask */
  /* Infinite loop */
	static uint8_t Algorithm_Mode=0;		//当前使用的算法类型，0：抓握力的PID控制 1：抓握力的MFAC控制 2：手势识别后的控制算法
//	uint32_t TargetVelocity=0;
	float ThumbFinger_SwingOutput_Velocity=0;
  for(;;)
  {	
		if(Algorithm_Mode==0){
			TargetVelocity=Calculate_Complement32(-GripForce_pid_Out/1.0);
			ThumbFinger_SwingOutput_Velocity=-GripForce_pid_Out/1.0/1024.0;
		}
		else if(Algorithm_Mode==1){
			if(GripForce_MFAC_Out>GripForce_MFAC_Adjust_Ability){
				TargetVelocity=Calculate_Complement32(GripForce_MFAC_Out/10.0);
				ThumbFinger_SwingOutput_Velocity=GripForce_MFAC_Out/10.0/1024.0;
			}
			else{
				TargetVelocity=Calculate_Complement32(GripForce_MFAC_Out*1024.0);
				ThumbFinger_SwingOutput_Velocity=GripForce_MFAC_Out;
			}				
		}
		Set_Faulhaber_FeedbackData(ThumbSwingMot_AutoModeSpeed_n,&ThumbFinger_SwingOutput_Velocity);
		//自动模式仅调速，等待其它任务通知
		Faulhaber_CAN1_SDOSet_MotVelocity(ThumbSwingMot_Node,TargetVelocity);					//拇指摆动电机
		osDelay(10);
  }
  /* USER CODE END FaulhaberMot_AutoControlTask */
}
float GripForce_pid_EK=0,FSR1_Actual_GripForce=0;
float GripForce_pid_view=0;
//const float sEMG_Force[20]={0,0.60,1.70,2.49,2.98,3.20,3.57,3.66,4.43,5.50,6.07,6.32,6.17,5.96,6.08,5.96,6.03,5.92,6.01,5.78};
//步长0.2s，力数据已减0.49清零
//m5
//const float sEMG_Force[32]={0.06,0.01,0.04,0.32,0.63,2.00,4.17,4.64,4.65,4.70,4.94,5.02,5.07,4.67,4.41,4.41,4.50,5.0,5.32,4.81,4.43,4.07,3.92,3.79,3.86,3.85,4.15,4.22,4.15,4.35,4.47,4.51};	
//s7
//const float sEMG_Force[38]={0,0.13,0.25,0.23,0.10,0.09,0.04,0.05,0.25,0.47,0.80,1.35,1.85,2.33,2.61,2.57,2.59,2.49,2.40,2.42,2.44,2.61,2.54,2.35,2.29,2.20,2.27,2.19,2.44,2.49,2.15,2.16,1.85,1.83,2.03,2.16,2.62,2.53};
//h6
const float sEMG_Force[35]={0,0.28,0.98,2.22,3.57,4.45,4.50,4.28,4.42,5.16,5.21,4.76,4.96,5.17,4.61,5.15,6.18,6.21,6.17,6.09,6.37,6.38,6.06,6.04,6.27,5.73,5.94,6.31,6.31,6.14,6.07,5.38,5.25,6.20,5.96};
	/* GripForce_PIDControlTask function */
void GripForce_PIDControlTask(void const * argument)
{
  /* USER CODE BEGIN GripForce_PIDControlTask */
  /* Infinite loop */
	arm_pid_instance_f32 GripForce_PID_Instance;
	GripForce_PID_Instance.state[0]=0;
	GripForce_PID_Instance.state[1]=0;
	GripForce_PID_Instance.state[2]=0;
	GripForce_PID_Instance.Kp=55.5;				//3N用55.5
	GripForce_PID_Instance.Ki=0;//1.2;
	GripForce_PID_Instance.Kd=0;
	arm_pid_init_f32(&GripForce_PID_Instance,1);
	
	float Expected_Force=0.49;																						//期望抓握力
	Expected_Force=sEMG_Force[0]+0.49;																			//初始期望抓握力
	uint8_t PID_Control_Startflag=0;																					//PID控制开启标志位
	uint8_t i=0,j=0,k=15;
	for(;;)
  {				
//		Get_Wireless_DataFrom_EEPROM(Grip_Control_Parameters_P_n,&GripForce_PID_Instance.Kp);	//参数P	
//		Get_Wireless_DataFrom_EEPROM(Grip_Control_Parameters_I_n,&GripForce_PID_Instance.Ki);	//参数I
//		Get_Wireless_DataFrom_EEPROM(Grip_Control_Parameters_D_n,&GripForce_PID_Instance.Kd); //参数D
//		arm_pid_init_f32(&GripForce_PID_Instance,1);
		Get_ProstheticHand_SensorData(FSR1_NormalPressure_n,&FSR1_Actual_GripForce);					//FSR1反馈拇指实际抓握力
		if(PID_Control_Startflag==0&&FSR1_Actual_GripForce>=Expected_Force/3.0){								//如果抓握力大于期望力1/3，开启PID控制，防止误差累积
			PID_Control_Startflag=1;																														//开启PID控制
		}
		else if(PID_Control_Startflag==0&&FSR1_Actual_GripForce<Expected_Force/3.0){
			GripForce_pid_Out=-1024.0;																													//抓握力小于期望力1/3，表示手指未触碰到物体
		}
		else if(PID_Control_Startflag==1){
			GripForce_pid_EK=FSR1_Actual_GripForce-Expected_Force;															//计算抓握力误差
			GripForce_pid_view=arm_pid_f32(&GripForce_PID_Instance,GripForce_pid_EK);						//PID算法函数
			GripForce_pid_Out=GripForce_pid_view/100.0*GripForce_pid_Adjust_Ability;
			if(GripForce_pid_Out>GripForce_pid_Adjust_Ability){																  //实验观察GripForce_pid_Out的范围之后取定
					GripForce_pid_Out=GripForce_pid_Adjust_Ability;	
			}
			else if(GripForce_pid_Out<-GripForce_pid_Adjust_Ability){														//实验观察GripForce_pid_Out的范围之后取定
					GripForce_pid_Out=-GripForce_pid_Adjust_Ability;	
			}
			if(j<35){
				i++;
				if(i>=30){
					i=0;
					j++;
					Expected_Force=sEMG_Force[j]+0.49;	
				}
			}
		}			
		osDelay(20);
	}
  /* USER CODE END GripForce_PIDControlTask */
}
float PPD_Estimate_ksub1=0,U_ksub1=0,U_ksub2=0,Delta_U_ksub1=0,Y_k=0,Y_ksub1=0,Delta_Y_k=0;
float PPD_Estimate_k=0,U_k=0;															//第k个PPD估计值phi(k)和控制变量输入值U(k)
float Expected_Y_kadd1=6;																	//期望抓握力
/* GripForce_MFACControlTask function */
void GripForce_MFACControlTask(void const * argument)
{
  /* USER CODE BEGIN GripForce_MFACControlTask */
  /* Infinite loop */						
	static uint8_t k=0,MFAC_Control_Startflag=0;								//MFAC_Control_Startflag:MFAC控制开启标志位；
	Get_ProstheticHand_SensorData(FSR1_NormalPressure_n,&Y_k);  //FSR1反馈拇指实际抓握力
	
	for(;;)
  {		
		if(MFAC_Control_Startflag==0&&Y_k>Expected_Y_kadd1/3.0){	//如果抓握力大于期望力1/3，开启MFAC控制，防止误差累积
			MFAC_Control_Startflag=1;																//开启MFAC控制
		}
		else if(MFAC_Control_Startflag==0&&Y_k<Expected_Y_kadd1/3.0){
			GripForce_MFAC_Out=10240.0;																//抓握力小于期望力1/3，表示手指未触碰到物体
			Get_ProstheticHand_SensorData(FSR1_NormalPressure_n,&Y_k);	//FSR1反馈拇指实际抓握力
		}
		if(MFAC_Control_Startflag==1){																//MFAC控制过程
			if(k==0){
				PPD_Estimate_k=1;
				U_ksub1=0;
				U_k=1;
				Get_ProstheticHand_SensorData(FSR1_NormalPressure_n,&Y_k);	//FSR1反馈拇指实际抓握力
				Y_ksub1=0;
				k=1;
			}			
			PPD_Estimate_ksub1=PPD_Estimate_k;
			U_ksub2=U_ksub1;
			U_ksub1=U_k;    
			Delta_U_ksub1=U_ksub1-U_ksub2;
			Delta_Y_k=Y_k-Y_ksub1;
			PPD_Estimate_k=Calculate_PPD_Estimate_k(PPD_Estimate_ksub1,Delta_U_ksub1,Delta_Y_k);
			
			//调试时查看数据
			debug_data1=eta*Delta_U_ksub1/(mu+Delta_U_ksub1*Delta_U_ksub1);
			debug_data2=Delta_Y_k-PPD_Estimate_ksub1*Delta_U_ksub1;
			debug_data3=rho*PPD_Estimate_k/(lambda+PPD_Estimate_k*PPD_Estimate_k);
			debug_data4=Expected_Y_kadd1-Y_k;
			
			U_k=Calculate_U_k(U_ksub1,PPD_Estimate_k,Expected_Y_kadd1,Y_k);
			Y_ksub1=Y_k;
			Get_ProstheticHand_SensorData(FSR1_NormalPressure_n,&Y_k);   //FSR1反馈拇指实际抓握力		
//			GripForce_MFAC_Out=U_k/20.0;
			GripForce_MFAC_Out=U_k;
			if(GripForce_MFAC_Out>GripForce_MFAC_Adjust_Ability){					 //实验观察GripForce_MFAC_Out的范围之后取定
				GripForce_MFAC_Out=GripForce_MFAC_Adjust_Ability;	
			}
			else if(GripForce_MFAC_Out<-GripForce_MFAC_Adjust_Ability){		 //实验观察GripForce_MFAC_Out的范围之后取定
				GripForce_MFAC_Out=-GripForce_MFAC_Adjust_Ability;
			}	
			U_k=GripForce_MFAC_Out;
			Set_Faulhaber_FeedbackData(ThumbJointMot_AutoModeSpeed_n,&PPD_Estimate_k);
		}
		osDelay(20);
  }
  /* USER CODE END GripForce_MFACControlTask */
}

uint16_t LTC1867_ADCDate[8]={0};
uint16_t adc_date_temp=100;
/* LTC1867_GetdataTask function */
void LTC1867_GetdataTask(void const * argument)
{
  /* USER CODE BEGIN LTC1867_GetdataTask */
  /* Infinite loop */
//	LTC1867_setup();
  for(;;)
  {		

//	LTC1867_read(LTC1867_CH0, &adc_date_temp); 		// Wakes up ADC if it was in sleep mode
//		single_ended_ch7(LTC1867_ADCDate);
		osDelay(10);
  }
  /* USER CODE END LTC1867_GetdataTask */
}

/* RGB_PowerOn_StartUp_Task function */
void RGB_PowerOn_StartUp_Task(void const * argument)
{
  /* USER CODE BEGIN RGB_PowerOn_StartUp_Task */
  /* Infinite loop */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	uint8_t count_j=0;
  for(;;)
  {
		for(uint8_t i=0;i<100;i++)
		{
			if(i<51){
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,i*39);	
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,i*39);	
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,i*39);	
			}
			else{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,3900-i*39);	
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,3900-i*39);	
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,3900-i*39);				
			}
			osDelay(20);
		}
		count_j++;
		if(count_j==3){
			/* definition and creation of RGB_Run_Task */
			osThreadDef(RGB_Run_Task, RGB_Run_Task, osPriorityNormal, 0, 32);
			RGB_Run_TaskHandle = osThreadCreate(osThread(RGB_Run_Task), NULL);
			vTaskDelete( NULL );
		}
	}
  /* USER CODE END RGB_PowerOn_StartUp_Task */
}

/* RGB_Run_Task function */
void RGB_Run_Task(void const * argument)
{

  /* USER CODE BEGIN RGB_Run_Task */
  /* Infinite loop */
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_MspDeInit(&htim3);
	HAL_TIM_Base_MspDeInit(&htim4);
	RGB_GPIO_Init();
	uint8_t i=0;
  for(;;)
  {
		switch(i){
			case 0: HAL_GPIO_TogglePin(GPIOB, STM32_TIM3_CH2_R_Pin);
				break;
			case 1: HAL_GPIO_TogglePin(GPIOB, STM32_TIM3_CH1_G_Pin);
				break;
			case 2: HAL_GPIO_TogglePin(GPIOB, STM32_TIM4_CH1_B_Pin);
				break;
			default:break;
		}
		i++;
		if (i==3){
			i=0;
		}
		osDelay(200);		
		HAL_GPIO_WritePin(GPIOB, STM32_TIM3_CH1_G_Pin|STM32_TIM3_CH2_R_Pin|STM32_TIM4_CH1_B_Pin,GPIO_PIN_RESET);		
  }
  /* USER CODE END RGB_Run_Task */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
