#ifndef GESTURES_RECOGNIZED_H
#define GESTURES_RECOGNIZED_H

#ifdef __cplusplus
		extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "arm_math.h" 						
			
/*------------����ʶ���������--------------------*/
typedef enum{
	Recognized_TypesofGestures_n=350,
}Recognized_Gestures_n;

//���ݵ��ã�������
void Get_Recognized_Gestures_Data(Recognized_Gestures_n name_num,void*extern_data);//ȡֵ��������������ʶ������,@Recognized_Gestures_n
void Set_Recognized_Gestures_Data(Recognized_Gestures_n name_num,void*extern_data);//�޸�����ʶ������,@Recognized_Gestures_n

#ifdef __cplusplus
}
#endif

#endif


