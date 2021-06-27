#include "MFAC_Controller.h"

volatile float debug_data1=0;
volatile float debug_data2=0;
volatile float debug_data3=0;
volatile float debug_data4=0;
/*
*****************************************************************************
*@brief		计算PPD估计值
*@param		float PPD_Estimate_ksub1：第k-1个PPD估计值phi(k-1)
*@param		float Delta_U_ksub1：第k-1个输入值与第k-2个输入值之差U(k-1)-U(k-2)
*@param		float Delta_Y_k：第k个输出值与第k-1个输出值之差Y(k)-Y(k-1)
*@retval	float PPD_Estimate_k：计算第K个PPD估计值phi(k)
*@par
*****************************************************************************
*/
float Calculate_PPD_Estimate_k(float PPD_Estimate_ksub1,float Delta_U_ksub1,float Delta_Y_k)
{
	float PPD_Estimate_k=0;
	PPD_Estimate_k=PPD_Estimate_ksub1+eta*Delta_U_ksub1/(mu+Delta_U_ksub1*Delta_U_ksub1)*(Delta_Y_k-PPD_Estimate_ksub1*Delta_U_ksub1);
	return PPD_Estimate_k;
}

/*
*****************************************************************************
*@brief		计算控制变量输入值
*@param		float U_ksub1：第k-1个输入值U(k-1)
*@param		float PPD_Estimate_k：第K个PPD估计值phi(k)
*@param		float Expected_Y_kadd1：第k+1个期望输出值Expected_Y(k+1)
*@param		float Y_k：第k个输出值Y(k)
*@retval	float U_k：计算第K个输入值U(k)
*@par
*****************************************************************************
*/
float Calculate_U_k(float U_ksub1,float PPD_Estimate_k,float Expected_Y_kadd1,float Y_k)
{
	float U_k=0;
	U_k=U_ksub1+rho*PPD_Estimate_k/(lambda+PPD_Estimate_k*PPD_Estimate_k)*(Expected_Y_kadd1-Y_k);
	return U_k;
}




