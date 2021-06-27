#include "LTC1867.h"



// Global variables
static uint8_t uni_bi_polar = LTC1867_UNIPOLAR_MODE;    //!< The LTC1867 unipolar/bipolar mode selection
static int32_t LTC1867_offset_unipolar_code = 0;        //!< Ideal unipolar offset for a perfect part
static int32_t LTC1867_offset_bipolar_code = 0;         //!< Ideal bipolar offset for a perfect part

//! Lookup table to build the command for single-ended mode, input with respect to GND
const uint8_t BUILD_COMMAND_SINGLE_ENDED[8] = {LTC1867_CH0, LTC1867_CH1, LTC1867_CH2, LTC1867_CH3,
    LTC1867_CH4, LTC1867_CH5, LTC1867_CH6, LTC1867_CH7
                                              }; 				//!< Builds the command for single-ended mode, input with respect to GND

//! Lookup table to build the command for single-ended mode with channel 7 as common pin
const uint8_t BUILD_COMMAND_SINGLE_ENDED_COM7[7] = {LTC1867_CH0_7COM, LTC1867_CH1_7COM, LTC1867_CH2_7COM, LTC1867_CH3_7COM,
    LTC1867_CH4_7COM, LTC1867_CH5_7COM, LTC1867_CH6_7COM
                                                   };   //!< Builds the command for single-ended mode, input with respect to CH7

//! Lookup table to build the command for differential mode with the selected uni/bipolar mode
const uint8_t BUILD_COMMAND_DIFF[8] = {LTC1867_P0_N1, LTC1867_P2_N3, LTC1867_P4_N5, LTC1867_P6_N7,
                                       LTC1867_P1_N0, LTC1867_P3_N2, LTC1867_P5_N4, LTC1867_P7_N6
                                      }; 								//!< Build the command for differential mode

/*
*****************************************************************************
*@brief		纳秒延时函数
*@param		uint32_t t，延时大小，步长6纳秒
*@retval	None
*@par
*****************************************************************************
*/
static void LTC1867_Delay_6ns(uint32_t t){
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
static void LTC1867_Delay_us(uint32_t cnt){
	uint32_t i,j;  
	for(i=0;i<cnt;i++)  
	{  
			for(j=0;j<35;j++);  
	}
}

// Reads the ADC  and returns 16-bit data
void LTC1867_read(uint8_t adc_command, uint16_t *adc_code)
{
	uint8_t LTC1867_rxdata[2]={0};
	HAL_GPIO_WritePin(STM32_SPI1_PSC0_LTC1867_GPIO_Port, STM32_SPI1_PSC0_LTC1867_Pin, GPIO_PIN_SET);
//	LTC1867_Delay_us(3);
	HAL_GPIO_WritePin(STM32_SPI1_PSC0_LTC1867_GPIO_Port, STM32_SPI1_PSC0_LTC1867_Pin, GPIO_PIN_RESET);	
	HAL_SPI_TransmitReceive(&hspi1,&adc_command,&LTC1867_rxdata[0],2,100);
//	HAL_SPI_TransmitReceive(&hspi1,&adc_command,&LTC1867_rxdata[1],1,100);
	*adc_code=(uint16_t)LTC1867_rxdata[0]<<8|LTC1867_rxdata[1];
}

void LTC1867_setup(void)
{
	uint16_t adc_code_temp[2]={0};
	LTC1867_read(BUILD_COMMAND_SINGLE_ENDED[0], adc_code_temp); 		// Wakes up ADC if it was in sleep mode
}

void single_ended_ch7(uint16_t *data)
{
	uint8_t i=0;
	LTC1867_read(BUILD_COMMAND_SINGLE_ENDED[0], &data[0]); 		// Wakes up ADC if it was in sleep mode
	for (i=0;i<8;i++)
	{
		LTC1867_read(BUILD_COMMAND_SINGLE_ENDED[i], &data[i]); 		// Wakes up ADC if it was in sleep mode
	}
}

void single_ended_com(uint16_t *data)
{
	uint8_t i=0;
	LTC1867_read(BUILD_COMMAND_SINGLE_ENDED_COM7[0], &data[0]); 		// Wakes up ADC if it was in sleep mode
	for (i=0;i<7;i++)
	{
		LTC1867_read(BUILD_COMMAND_SINGLE_ENDED_COM7[i], &data[i]); 		// Wakes up ADC if it was in sleep mode
	}
}

void differencial_ended(uint16_t *data)
{


}

