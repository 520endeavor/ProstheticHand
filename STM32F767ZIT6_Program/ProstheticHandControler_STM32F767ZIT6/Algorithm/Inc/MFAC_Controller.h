#ifndef MFAC_CONTROLLER_H
#define MFAC_CONTROLLER_H

#ifdef __cplusplus
		extern "C" {
#endif

#include "stm32f7xx_hal.h"					
			
			
#define eta    		 1.0
#define mu     		 2.0			
#define rho    		 0.6
#define lambda     0.001

extern volatile float debug_data1;
extern volatile float debug_data2;
extern volatile float debug_data3;
extern volatile float debug_data4;
			
float Calculate_PPD_Estimate_k(float PPD_Estimate_ksub1,float Delta_U_ksub1,float Delta_Y_k);		//计算PPD估计值
float Calculate_U_k(float U_ksub1,float PPD_Estimate_k,float Expected_Y_kadd1,float Y_k);				//计算控制变量输入值			
			

#ifdef __cplusplus
}
#endif

#endif

