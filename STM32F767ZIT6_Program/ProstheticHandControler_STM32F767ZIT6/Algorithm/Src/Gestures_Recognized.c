#include "Gestures_Recognized.h"


struct {
	volatile uint16_t Recognized_TypesofGestures;					//ʶ���������������
}Recognized_Gestures;//����ʶ������


/*
*****************************************************************************
*@brief		ȡֵ��������������ʶ������
*@param		Recognized_Gestures_n name_num����ȡ���ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data����Ŵ��ݳ���������
*@retval	None
*@par
*****************************************************************************
*/
void Get_Recognized_Gestures_Data(Recognized_Gestures_n name_num,void*extern_data)//*extern_data�������get����ֵ
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
*@brief		�޸�����ʶ������
*@param		Recognized_Gestures_n name_num���޸����ݶ�Ӧ��ö�ٱ���
*@param		void*extern_data�������޸ĵ�Ŀ��ֵ
*@retval	None
*@par
*****************************************************************************
*/
void Set_Recognized_Gestures_Data(Recognized_Gestures_n name_num,void*extern_data)//*extern_data�������get����ֵ
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



