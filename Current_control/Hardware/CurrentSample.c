#include "MyProject.h"

extern ADC_HandleTypeDef hadc1;
extern uint16_t ad_value[2];//���������洢
extern int16_t current_a_count;
extern int16_t current_b_count;

float getDCCurrent(float motor_electrical_angle)
{
	PhaseCurrent_s current;
	float sign=1;       // currnet sign - if motor angle not provided the magnitude is always positive
	float i_alpha, i_beta;
	
	// read current phase currents
	current = getPhaseCurrents();
	
	// calculate clarke transform
	if(!current.c)
	{
		// if only two measured currents
		i_alpha = current.a;
		i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
	}
	else
	{
		// signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
		float mid = (1.f/3) * (current.a + current.b + current.c);
		float a = current.a - mid;
		float b = current.b - mid;
		i_alpha = a;
		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
	}
	
	// if motor angle provided function returns signed value of the current
	// determine the sign of the current
	// sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1  
	if(motor_electrical_angle)sign = (i_beta * _cos(motor_electrical_angle) - i_alpha*_sin(motor_electrical_angle)) > 0 ? 1 : -1;  
	// return current magnitude
	return sign*_sqrt(i_alpha*i_alpha + i_beta*i_beta);
}
/******************************************************************************/
// function used with the foc algorihtm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents 
//   - using getPhaseCurrents internally
DQCurrent_s getFOCCurrents(float angle_el)
{
	PhaseCurrent_s current;
	float i_alpha, i_beta;
	float ct,st;
	DQCurrent_s ret;
	
	// read current phase currents
	current = getPhaseCurrents();
	
	// calculate clarke transform
	if(!current.c)
	{
		// if only two measured currents
		i_alpha = current.a;  
		i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
	}
	else
	{
		// signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
		float mid = (1.f/3) * (current.a + current.b + current.c);
		float a = current.a - mid;
		float b = current.b - mid;
		i_alpha = a;
		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
	}
	
	// calculate park transform
	ct = _cos(angle_el);
	st = _sin(angle_el);
	ret.d = i_alpha * ct + i_beta * st;
	ret.q = i_beta * ct - i_alpha * st;
	return ret;
}

PhaseCurrent_s getPhaseCurrents(void)
{
	PhaseCurrent_s current;
	
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)(ad_value),2);
	current.a = current_a_count*3.3/(4096*1.528*0.33); //current.a = (current_a_count*3.3/4096 - 1.558)/(1.528*0.33);
	current.b = current_b_count*3.3/(4096*1.528*0.33);
	current.c = 0;	//C���������δ����
	
	return current;
}
