#include "Gestures_Recognized.h"


struct {
	volatile uint16_t Recognized_TypesofGestures;					//识别的手势类型数据
}Recognized_Gestures;//手势识别数据


/*
*****************************************************************************
*@brief		取值函数，搬运手势识别数据
*@param		Recognized_Gestures_n name_num：索取数据对应的枚举变量
*@param		void*extern_data：存放传递出来的数据
*@retval	None
*@par
*****************************************************************************
*/
void Get_Recognized_Gestures_Data(Recognized_Gestures_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	switch(name_num)
	{
		case Recognized_TypesofGestures_n:
			*(uint16_t*)extern_data=Recognized_Gestures.Recognized_TypesofGestures;
			break;
		default:			
			break;
	}
}

/*
*****************************************************************************
*@brief		修改手势识别数据
*@param		Recognized_Gestures_n name_num：修改数据对应的枚举变量
*@param		void*extern_data：传递修改的目标值
*@retval	None
*@par
*****************************************************************************
*/
void Set_Recognized_Gestures_Data(Recognized_Gestures_n name_num,void*extern_data)//*extern_data用来存放get到的值
{
	switch(name_num)
	{
		case Recognized_TypesofGestures_n:
			Recognized_Gestures.Recognized_TypesofGestures=*(uint16_t*)extern_data;
			break;
		default:			
			break;
	}
}



