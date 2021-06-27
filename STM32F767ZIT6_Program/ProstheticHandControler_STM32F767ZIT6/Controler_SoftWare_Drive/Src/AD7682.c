#include "AD7682.h"	


/*
*****************************************************************************
*@brief		������ʱ����
*@param		uint32_t t����ʱ��С������6����
*@retval	None
*@par
*****************************************************************************
*/
static void AD7682_Delay_6ns(uint32_t t){
	while(t--);
}

/*
*****************************************************************************
*@brief		΢����ʱ����
*@param		uint32_t cnt����ʱ��С����λ΢��
*@retval	None
*@par
*****************************************************************************
*/
static void AD7682_Delay_us(uint32_t cnt){
	uint32_t i,j;  
	for(i=0;i<cnt;i++)  
	{  
			for(j=0;j<35;j++);  
	}
}

///*
//*****************************************************************************
//*@brief		����AD7618��ȡ������ע��SPI1��ȡ����AD2�����ݣ���ͨ��AD8-AD15
//					SPI2��ȡ����AD1�����ݣ���ͨ��AD0-AD7
//*@param		SPI_HandleTypeDef *spiHandle��SPI�ṹ����
//*@param		float *data���洢ADģ���������׵�ַ
//*@retval	None
//*@par
//*****************************************************************************
//*/
//void Get_AD7682Data(SPI_HandleTypeDef *spiHandle,float *data){
//	uint16_t CFG=0x3C49;
//	uint8_t txData[2],rxData[2];
//	uint8_t DumbrxData[2];					//�洢�ƶ�����
//	uint8_t i=0,b=0;
//	uint16_t a;
//	/*�ƶ�����*/
//	CFG=(CFG&0x3C7F)|(i<<7);
//	txData[0]=(CFG<<2)>>8;											//�ڲ���׼Դ,REF=4.096,��ֹ������
//	txData[1]=(CFG<<2)&0x00FF;		
//	if(spiHandle->Instance==SPI2){										//ʹ��SPI2��ȡAD1������
//		for(uint8_t j=0;j<2;j++){
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_RESET);
//			AD7682_Delay_6ns(3);	
//			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&DumbrxData[0],1,100);//�����֣�����ͨ��IN0
//			AD7682_Delay_us(2);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_SET);//SOC
//			AD7682_Delay_6ns(3);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_RESET);
//			AD7682_Delay_6ns(3);	
//			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &DumbrxData[1],1,100);
//			AD7682_Delay_6ns(3);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_SET);
//			AD7682_Delay_us(2);													//�����з���EOC
//			a=(DumbrxData[0]<<8)|DumbrxData[1];					//���յ�����
//			if(j==1){
//				data[i]=a/65536.0L*4.096L;										//REF=4.096
//			}
//		}
//		/*��ȡ8��ͨ������*/	
//		for (i=1;i<9;i++){			
//			if(i==8){
//				i=0;
//				b=1;
//			}
//			CFG=(CFG&0x3C7F)|(i<<7);
//			txData[0]=(CFG<<2)>>8;										//�ڲ���׼Դ,REF=4.096,��ֹ������
//			txData[1]=(CFG<<2)&0x00FF;		
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_RESET);
//			AD7682_Delay_6ns(3);	
//			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&rxData[0],1,100);//�����֣�����ͨ��IN   i+2
//			AD7682_Delay_us(2);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_SET);//SOC
//			AD7682_Delay_6ns(3);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_RESET);
//			AD7682_Delay_6ns(3);	
//			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &rxData[1],1,100);
//			AD7682_Delay_6ns(3);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_SET);
//			AD7682_Delay_us(2);											//�����з���EOC
//			a=(rxData[0]<<8)|rxData[1];							//���յ�����
//			if(b==1){
//				i=8;
//			}
//			data[i-1]=a/65536.0L*4.096L;							//REF=4.096
//		}	
//	}
//}
	


