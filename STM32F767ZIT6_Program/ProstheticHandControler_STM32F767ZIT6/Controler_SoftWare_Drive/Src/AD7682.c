#include "AD7682.h"	


/*
*****************************************************************************
*@brief		纳秒延时函数
*@param		uint32_t t，延时大小，步长6纳秒
*@retval	None
*@par
*****************************************************************************
*/
static void AD7682_Delay_6ns(uint32_t t){
	while(t--);
}

/*
*****************************************************************************
*@brief		微秒延时函数
*@param		uint32_t cnt，延时大小，单位微秒
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
//*@brief		两个AD7618读取函数，注意SPI1读取的是AD2的数据，即通道AD8-AD15
//					SPI2读取的是AD1的数据，即通道AD0-AD7
//*@param		SPI_HandleTypeDef *spiHandle：SPI结构体句柄
//*@param		float *data：存储AD模拟量数组首地址
//*@retval	None
//*@par
//*****************************************************************************
//*/
//void Get_AD7682Data(SPI_HandleTypeDef *spiHandle,float *data){
//	uint16_t CFG=0x3C49;
//	uint8_t txData[2],rxData[2];
//	uint8_t DumbrxData[2];					//存储哑读数据
//	uint8_t i=0,b=0;
//	uint16_t a;
//	/*哑读数据*/
//	CFG=(CFG&0x3C7F)|(i<<7);
//	txData[0]=(CFG<<2)>>8;											//内部基准源,REF=4.096,禁止序列器
//	txData[1]=(CFG<<2)&0x00FF;		
//	if(spiHandle->Instance==SPI2){										//使用SPI2读取AD1的数据
//		for(uint8_t j=0;j<2;j++){
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_RESET);
//			AD7682_Delay_6ns(3);	
//			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&DumbrxData[0],1,100);//配置字，输入通道IN0
//			AD7682_Delay_us(2);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_SET);//SOC
//			AD7682_Delay_6ns(3);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_RESET);
//			AD7682_Delay_6ns(3);	
//			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &DumbrxData[1],1,100);
//			AD7682_Delay_6ns(3);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_SET);
//			AD7682_Delay_us(2);													//过程中发生EOC
//			a=(DumbrxData[0]<<8)|DumbrxData[1];					//接收的数据
//			if(j==1){
//				data[i]=a/65536.0L*4.096L;										//REF=4.096
//			}
//		}
//		/*读取8个通道数据*/	
//		for (i=1;i<9;i++){			
//			if(i==8){
//				i=0;
//				b=1;
//			}
//			CFG=(CFG&0x3C7F)|(i<<7);
//			txData[0]=(CFG<<2)>>8;										//内部基准源,REF=4.096,禁止序列器
//			txData[1]=(CFG<<2)&0x00FF;		
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_RESET);
//			AD7682_Delay_6ns(3);	
//			HAL_SPI_TransmitReceive(&hspi1,&txData[0],&rxData[0],1,100);//配置字，输入通道IN   i+2
//			AD7682_Delay_us(2);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_SET);//SOC
//			AD7682_Delay_6ns(3);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_RESET);
//			AD7682_Delay_6ns(3);	
//			HAL_SPI_TransmitReceive(&hspi1,&txData[1], &rxData[1],1,100);
//			AD7682_Delay_6ns(3);
//			HAL_GPIO_WritePin(STM32_SPI1_PSC0_AD7682_GPIO_Port, STM32_SPI1_PSC0_AD7682_Pin, GPIO_PIN_SET);
//			AD7682_Delay_us(2);											//过程中发生EOC
//			a=(rxData[0]<<8)|rxData[1];							//接收的数据
//			if(b==1){
//				i=8;
//			}
//			data[i-1]=a/65536.0L*4.096L;							//REF=4.096
//		}	
//	}
//}
	


