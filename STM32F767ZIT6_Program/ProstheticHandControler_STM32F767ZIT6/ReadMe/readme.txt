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
	ʹ�ö��з�������ʱ��ע���ܵ��Ƚ�����ն��е����񣬷�ֹ�������ݱ�����
	
	ö�����ͱ��:
	Can.h								:1~149						(δʹ�õ�Ϊ�������)
	HostComputer_Communication.h		:150~299
	Bluetooth_Operating.h				:300~349
	Gestures_Recognized.h				:350~399
	
	Fram�洢������ַ����:
	(ռ�����ֽ�)
	ManualOrAutomatic_ModeSelect_ID						0x00 						//�ֶ�Or�Զ�ģʽѡ������ID
	MotorEnableOrDisable_ID								0x02						//���ʹ��Orʧ������ID
	ThumbJointMot_TargetSpeed_ID						0x04						//Ĵָ�źϵ��Ŀ��ת������ID
	ThumbSwingMot_TargetSpeed_ID						0x06						//Ĵָ�ڶ����Ŀ��ת������ID
	IndexFingerSwingMot_TargetSpeed_ID					0x08						//ʳָ�ڶ����Ŀ��ת������ID
	MiddleFingerSwingMot_TargetSpeed_ID					0x0A						//��ָ�ڶ����Ŀ��ת������ID
	RingFingerSwingMot_TargetSpeed_ID					0x0C						//����ָ�ڶ����Ŀ��ת������ID
	LittleFingerSwingMot_TargetSpeed_ID					0x0E						//Сָ�ڶ����Ŀ��ת������ID
	PowerUp_DefaultTypesofGestures_ID					0x10						//�ϵ�ʱĬ��������������ID
	Manual_TargetTypesofGesture_ID						0x12						//�ֶ�ģʽĿ��������������ID
	Bluetooth_AT_SetMasterMode_Flag_ID					0x14						//ATָ������Masterģʽ,��־λ����ID
	Grip_Control_Parameters_P_ID						0x16						//ץ�տ��Ʋ���������ϵ������ID
	Grip_Control_Parameters_I_ID						0x18						//ץ�տ��Ʋ���������ϵ������ID
	Grip_Control_Parameters_D_ID						0x1A						//ץ�տ��Ʋ�����΢��ϵ������ID
	
	����ָʾ��״̬���壺
		1.AT ��Ӧģʽ��DS2������DS1ͬ��׼HC-05�����豸 (2s��˸һ��)
		2.��Գɹ�������ָʾ��DS2����,DS1ͬ��׼HC-05�����豸 (2s��˸����)
		3.δ��ԣ��Զ�������ָʾ��DS2 200ms��˸һ�Σ�DS1ͬ��׼HC-05�����豸 (200ms��˸һ��)

	Fatfs��ֲ    ���У�bsp_driver_sram.c��BSP_SRAM_Init������sram_diskio.c��4������  ���޸�
		 SRAMDISK_status (BYTE);
		 SRAMDISK_read (BYTE, BYTE*, DWORD, UINT);
		 SRAMDISK_write (BYTE, const BYTE*, DWORD, UINT);
		 SRAMDISK_ioctl (BYTE, BYTE, void*);
	
	
	