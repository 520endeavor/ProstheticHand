/*!
LTC1867: 16-bit 8-channel 200ksps ADC

@verbatim

The LTC1863/LTC1867 are pin-compatible, 8-channel 12-/16-bit A/D converters 
with serial I/O, and an internal reference. The ADCs typically draw only 1.3mA 
from a single 5V supply. The 8-channel input multiplexer can be configured for 
either single-ended or differential inputs and unipolar or bipolar conversions 
(or combinations thereof). The automatic nap and sleep modes benefit power 
sensitive applications. 

The LTC1867's DC performance is outstanding with a +/-2LSB INL specification and 
no missing codes over temperature. The signal-to-noise ratio (SNR) for the 
LTC1867 is typically 89dB, with the internal reference. 

SPI DATA FORMAT (MSB First):

            Byte #1                           Byte #2
Data Out :  D15 D14 D13 D12 D11 D10 D9  D8    D7  D6  D5  D4  D3  D2  D1  D0
Data In  :  SD  OS  S1  S0  COM UNI SLP X     X   X   X   X   X   X   X   X

SD   : Single/Differential Bit
OS   : ODD/Sign Bit
Sx   : Address Select Bit
COM  : CH7/COM Configuration Bit
UNI  : Unipolar/Bipolar Bit
SLP  : Sleep Mode Bit
Dx   : Data Bits
X    : Don't care


Example Code:

Read Channel 0 in Single-Ended Unipolar mode when input is with respect to GND

    adc_command = LTC1867_CH0 | LTC1867_UNIPOLAR_MODE;  // Build ADC command for channel 0
    LTC1867_read(LTC1867_CS, adc_command, &adc_code);   // Throws out last reading
    LTC1867_read(LTC1867_CS, adc_command, &adc_code);   // Obtains the current reading and stores to adc_code variable

    // Convert adc_code to voltage
    adc_voltage = LTC1867_unipolar_code_to_voltage(adc_code, LTC1867_lsb, LTC1867_offset_unipolar_code);
*/


#ifndef LTC1867_H
#define LTC1867_H
//#ifdef __cplusplus
// extern "C" {
//#endif


#include "spi.h"
#include "gpio.h"

//! @name LTC1867 Single-Ended Channel Addresses, COM=GND
//! @{
// Single-Ended Channel Address When CH7/COM Pin Is used As CH7
#define LTC1867_CH0             0x80
#define LTC1867_CH1             0xC0
#define LTC1867_CH2             0x90
#define LTC1867_CH3             0xD0
#define LTC1867_CH4             0xA0
#define LTC1867_CH5             0xE0
#define LTC1867_CH6             0xB0
#define LTC1867_CH7             0xF0
//!@}

//! @name LTC1867 Differential Channel Addresses
//! @{
// Differential Channel Address When CH7/COM Pin Is Used As CH7
#define LTC1867_P0_N1           0x00
#define LTC1867_P1_N0           0x40

#define LTC1867_P2_N3           0x10
#define LTC1867_P3_N2           0x50

#define LTC1867_P4_N5           0x20
#define LTC1867_P5_N4           0x60

#define LTC1867_P6_N7           0x30
#define LTC1867_P7_N6           0x70
//!@}

//! @name LTC1867 Single-Ended Channel Addresses, COM=CH7
//! @{
// Channel Address When CH7/COM Pin Is Used As Common
#define LTC1867_CH0_7COM        0x88
#define LTC1867_CH1_7COM        0xC8
#define LTC1867_CH2_7COM        0x98
#define LTC1867_CH3_7COM        0xD8
#define LTC1867_CH4_7COM        0xA8
#define LTC1867_CH5_7COM        0xE8
#define LTC1867_CH6_7COM        0xB8
//!@}

//! @name LTC1867 Sleep / Unipolar/Bipolar config bits
//! @{
// Sleep Mode Command
#define LTC1867_SLEEP_MODE      0x02
#define LTC1867_EXIT_SLEEP_MODE 0x00
#define LTC1867_UNIPOLAR_MODE   0x04  		// Bitwise OR with channel commands for unipolar mode
#define LTC1867_BIPOLAR_MODE    0x00
//!@}

#define LTC1867_LSB							6.25E-5		//!< Ideal LSB voltage for a perfect part


//! Reads the ADC and returns 16-bit data
//! @return void
extern void LTC1867_read(uint8_t adc_command,  	//!< Channel address, config bits ORed together
                  uint16_t *adc_code   		//!< Returns code read from ADC (from previous conversion)
                 );

extern void LTC1867_setup(void);									//Wakes up ADC if it was in sleep mode

extern void single_ended_ch7(uint16_t *data);
extern void single_ended_com(uint16_t *data);
extern void differencial_ended(uint16_t *data);



////! Calculates the LTC1867 input's unipolar voltage given the binary data and lsb weight.
////! @return Floating point voltage
//float LTC1867_unipolar_code_to_voltage(uint16_t adc_code,                   //!< Raw ADC code
//                                       float LTC1867_lsb,                   //!< LSB value (volts)
//                                       int32_t LTC1867_offset_unipolar_code //!< Offset code
//                                      );

////! Calculates the LTC1867 input's bipolar voltage given the two's compliment data and lsb weight
////! @return  Floating point voltage
//float LTC1867_bipolar_code_to_voltage(uint16_t adc_code,                   	//!< Raw ADC code
//                                      float LTC1867_lsb,                   	//!< LSB value (volts)
//                                      int32_t LTC1867_offset_bipolar_code  	//!< Offset code
//                                     );

////! Calibrate the offset and LSB voltage given two measured offset codes, and a full-scale voltage with the corresponding code.
////! @return Void
//void LTC1867_cal_voltage(uint16_t zero_unipolar_code,                     	 //!< Code from a unipolar zero reading
//                         uint16_t zero_bipolar_code,                      	 //!< Code from a bipolar zero reading
//                         uint16_t fs_code,                                	 //!< full-scale code
//                         float zero_voltage,                              	 //!< Measured zero voltage
//                         float fs_voltage,                                	 //!< Measured full-scale voltage
//                         float *LTC1867_lsb,                             	 	 //!< Return LSB value (volts)
//                         int32_t *LTC1867_offset_unipolar_code,          	   //!< Return Unipolar Offset code, in floating point
//                         int32_t *LTC1867_offset_bipolar_code            	   //!< Return Bipolar Offset code, in floating point
//                        );

//#ifdef __cplusplus
//}
//#endif
#endif  //  LTC1867_H
												 
												 