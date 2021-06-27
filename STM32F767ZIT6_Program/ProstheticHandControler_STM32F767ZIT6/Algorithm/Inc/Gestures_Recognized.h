#ifndef GESTURES_RECOGNIZED_H
#define GESTURES_RECOGNIZED_H

#ifdef __cplusplus
		extern "C" {
#endif

#include "stm32f7xx_hal.h"
#include "arm_math.h" 						
			
/*------------手势识别数据序号--------------------*/
typedef enum{
	Recognized_TypesofGestures_n=350,
}Recognized_Gestures_n;

//数据调用，处理函数
void Get_Recognized_Gestures_Data(Recognized_Gestures_n name_num,void*extern_data);//取值函数，搬运手势识别数据,@Recognized_Gestures_n
void Set_Recognized_Gestures_Data(Recognized_Gestures_n name_num,void*extern_data);//修改手势识别数据,@Recognized_Gestures_n

#ifdef __cplusplus
}
#endif

#endif


