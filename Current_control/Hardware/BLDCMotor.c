#include "Myproject.h"

//extern TorqueControlType torque_controller;
//extern MotionControlType controller;
//extern float shaft_velocity_sp;
//extern float shaft_angle_sp;
//extern DQVoltage_s voltage;
//extern float shaft_angle;
//extern float shaft_velocity;
//extern float current_sp;
//extern float electrical_angle;
//extern float f3[3];

float voltage_power_supply;
unsigned long open_loop_timestamp;
float voltage_limit;
float velocity_limit;
float current_limit;
float voltage_sensor_align;
int pole_pairs;
int PWM_Period = 300;

void move(float new_target)
{
	shaft_velocity = shaftVelocity();//获取当前角速度
	
	switch(controller)
	{
		case Type_torque:
			if(torque_controller==Type_voltage)voltage.q = new_target;  // if voltage torque control
			else
				current_sp = new_target; // if current/foc_current torque control
		break;
			
		case Type_angle:
			// angle set point
			shaft_angle_sp = new_target;
			// calculate velocity set point
			shaft_velocity_sp = PIDoperator(&P_angle,(shaft_angle_sp - shaft_angle));
			// calculate the torque command
			current_sp = PIDoperator(&PID_velocity,(shaft_velocity_sp - shaft_velocity)); // if voltage torque control
			// if torque controlled through voltage  
			if(torque_controller == Type_voltage)
			{
				voltage.q = current_sp;
				voltage.d = 0;
			}
		break;
			
		case Type_velocity:
			// velocity set point
			shaft_velocity_sp = new_target;
			// calculate the torque command
			current_sp = PIDoperator(&PID_velocity,(shaft_velocity_sp - shaft_velocity)); // if current/foc_current torque control
			// if torque controlled through voltage control 
			if(torque_controller == Type_voltage)
			{
				voltage.q = current_sp;  // use voltage if phase-resistance not provided
				voltage.d = 0;
			}
		break;
			
		case Type_velocity_openloop:
			shaft_velocity_sp = new_target;
			voltage.q = velocityOpenloop(shaft_velocity_sp);
			voltage.d = 0;
			break;
		
		case Type_angle_openloop:
			shaft_angle_sp = new_target;
			voltage.q = angleOpenloop(shaft_angle_sp);
			voltage.d = 0;
			break;
	}
}

void loopFOC(void)
{
	if( controller==Type_angle_openloop || controller==Type_velocity_openloop ) return;
	
	shaft_angle = getAngle();// shaft angle
	electrical_angle = _normalizeAngle(shaft_angle*pole_pairs);// electrical angle
	
	switch(torque_controller)
	{
		case Type_voltage:  // no need to do anything really
			break;
		case Type_dc_current:
			current.q = getDCCurrent(electrical_angle);
			// filter the value values
			current.q = LPFoperator(&LPF_current_q,current.q);
			// calculate the phase voltage
			voltage.q = PIDoperator(&PID_current_q,(current_sp - current.q)); 
			voltage.d = 0;
			break;
		case Type_foc_current:
			current = getFOCCurrents(electrical_angle);
			// filter values
			current.q = LPFoperator(&LPF_current_q,current.q);
			current.d = LPFoperator(&LPF_current_d,current.d);
			// calculate the phase voltages
			voltage.q = PIDoperator(&PID_current_q,(current_sp - current.q)); 
			voltage.d = PIDoperator(&PID_current_d, -current.d);
			break;
		default:
			printf("MOT: no torque control selected!");
			break;
	}
	// set the phase voltage - FOC heart function
	setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

float velocityOpenloop(float target_velocity)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = SysTick->VAL; //读寄存器的值
	if(now_us<open_loop_timestamp)Ts = (float)(open_loop_timestamp - now_us)/12*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp)/12*1e-6;
	open_loop_timestamp=now_us;  //save timestamp for next call
	// quick fix for strange cases (micros overflow)
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	// calculate the necessary angle to achieve target velocity
	shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
	
	Uq = voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
	
	return Uq;
}

float angleOpenloop(float target_angle)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = SysTick->VAL; //_micros();
	if(now_us<open_loop_timestamp)Ts = (float)(open_loop_timestamp - now_us)/12*1e-6;
	else
		Ts = (float)(0xFFFFFF - now_us + open_loop_timestamp)/12*1e-6;
	open_loop_timestamp = now_us;  //save timestamp for next call
	// quick fix for strange cases (micros overflow)
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3; 
	
	// calculate the necessary angle to move from current position towards target angle
	// with maximal velocity (velocity_limit)
	if(fabs( target_angle - shaft_angle ) > velocity_limit*Ts)
	{
		shaft_angle += _sign(target_angle - shaft_angle) * velocity_limit * Ts;
		//shaft_velocity = velocity_limit;
	}
	else
	{
		shaft_angle = target_angle;
		//shaft_velocity = 0;
	}
	
	Uq = voltage_limit;
	// set the maximal allowed voltage (voltage_limit) with the necessary angle
	setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
	
	return Uq;
}

void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	
	if(Ud) // only if Ud and Uq set 
	{// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud*Ud + Uq*Uq) / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	if(Uout> 0.577)Uout= 0.577;
	if(Uout<-0.577)Uout=-0.577;
	
	sector = (angle_el / _PI_3) + 1;
	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;
	
	// calculate the duty cycles(times)
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,Ta*PWM_Period);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,Tb*PWM_Period);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,Tc*PWM_Period);
}
