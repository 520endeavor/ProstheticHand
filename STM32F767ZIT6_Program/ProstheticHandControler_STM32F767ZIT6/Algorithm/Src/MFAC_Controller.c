#include "MFAC_Controller.h"

volatile float debug_data1=0;
volatile float debug_data2=0;
volatile float debug_data3=0;
volatile float debug_data4=0;
/*
*****************************************************************************
*@brief		����PPD����ֵ
*@param		float PPD_Estimate_ksub1����k-1��PPD����ֵphi(k-1)
*@param		float Delta_U_ksub1����k-1������ֵ���k-2������ֵ֮��U(k-1)-U(k-2)
*@param		float Delta_Y_k����k�����ֵ���k-1�����ֵ֮��Y(k)-Y(k-1)
*@retval	float PPD_Estimate_k�������K��PPD����ֵphi(k)
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
*@brief		������Ʊ�������ֵ
*@param		float U_ksub1����k-1������ֵU(k-1)
*@param		float PPD_Estimate_k����K��PPD����ֵphi(k)
*@param		float Expected_Y_kadd1����k+1���������ֵExpected_Y(k+1)
*@param		float Y_k����k�����ֵY(k)
*@retval	float U_k�������K������ֵU(k)
*@par
*****************************************************************************
*/
float Calculate_U_k(float U_ksub1,float PPD_Estimate_k,float Expected_Y_kadd1,float Y_k)
{
	float U_k=0;
	U_k=U_ksub1+rho*PPD_Estimate_k/(lambda+PPD_Estimate_k*PPD_Estimate_k)*(Expected_Y_kadd1-Y_k);
	return U_k;
}




