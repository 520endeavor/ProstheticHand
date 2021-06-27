/**
  ******************************************************************************
  * File Name          : readme.txt
  *	Author			   : Enzhi.Zhou
  * Description        : Introduction for this program
  ******************************************************************************
  * Copyright (c) Enzhi.Zhou
  * All rights reserved.
  *
  ******************************************************************************
  **/  
	Notice
	使用队列发送数据时，注意能调度进入清空队列的任务，防止队列内容被覆盖
	
	枚举类型编号:
	Can.h								:1~149						(未使用的为保留编号)
	HostComputer_Communication.h		:150~299
	Bluetooth_Operating.h				:300~349
	Gestures_Recognized.h				:350~399
	
	Fram存储器，地址分配:
	(占两个字节)
	ManualOrAutomatic_ModeSelect_ID						0x00 						//手动Or自动模式选择数据ID
	MotorEnableOrDisable_ID								0x02						//电机使能Or失能数据ID
	ThumbJointMot_TargetSpeed_ID						0x04						//拇指张合电机目标转速数据ID
	ThumbSwingMot_TargetSpeed_ID						0x06						//拇指摆动电机目标转速数据ID
	IndexFingerSwingMot_TargetSpeed_ID					0x08						//食指摆动电机目标转速数据ID
	MiddleFingerSwingMot_TargetSpeed_ID					0x0A						//中指摆动电机目标转速数据ID
	RingFingerSwingMot_TargetSpeed_ID					0x0C						//无名指摆动电机目标转速数据ID
	LittleFingerSwingMot_TargetSpeed_ID					0x0E						//小指摆动电机目标转速数据ID
	PowerUp_DefaultTypesofGestures_ID					0x10						//上电时默认手势类型数据ID
	Manual_TargetTypesofGesture_ID						0x12						//手动模式目标手势类型数据ID
	Bluetooth_AT_SetMasterMode_Flag_ID					0x14						//AT指令设置Master模式,标志位数据ID
	Grip_Control_Parameters_P_ID						0x16						//抓握控制参数，比例系数数据ID
	Grip_Control_Parameters_I_ID						0x18						//抓握控制参数，积分系数数据ID
	Grip_Control_Parameters_D_ID						0x1A						//抓握控制参数，微分系数数据ID
	
	蓝牙指示灯状态含义：
		1.AT 响应模式，DS2不亮，DS1同标准HC-05蓝牙设备 (2s闪烁一次)
		2.配对成功，蓝牙指示灯DS2常亮,DS1同标准HC-05蓝牙设备 (2s闪烁两次)
		3.未配对，自定义蓝牙指示灯DS2 200ms闪烁一次，DS1同标准HC-05蓝牙设备 (200ms闪烁一次)

	Fatfs移植    其中，bsp_driver_sram.c中BSP_SRAM_Init函数和sram_diskio.c中4个函数  可修改
		 SRAMDISK_status (BYTE);
		 SRAMDISK_read (BYTE, BYTE*, DWORD, UINT);
		 SRAMDISK_write (BYTE, const BYTE*, DWORD, UINT);
		 SRAMDISK_ioctl (BYTE, BYTE, void*);
	
	
	