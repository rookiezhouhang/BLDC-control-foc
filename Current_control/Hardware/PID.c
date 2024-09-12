#include "MyProject.h"

PIDController  PID_current_q,PID_current_d,PID_velocity,P_angle;
LowPassFilter  LPF_current_q,LPF_current_d,LPF_velocity;
float y_vel_prev=0;

void PID_init(void)
{
	PID_velocity.P=0.1;  
	PID_velocity.I=2;    
	PID_velocity.D=0;
	PID_velocity.output_ramp=50;    //限制转速变化速率，
	if(torque_controller == Type_voltage)PID_velocity.limit = voltage_limit;  //速度模式的电流限制
	else  PID_velocity.limit = current_limit;       
	PID_velocity.error_prev=0;
	PID_velocity.output_prev=0;
	PID_velocity.integral_prev=0;
	PID_velocity.timestamp_prev=0;
	
	P_angle.P=20;
	P_angle.I=0;
	P_angle.D=0;
	P_angle.output_ramp=0;
	P_angle.limit=velocity_limit;
	P_angle.error_prev=0;
	P_angle.output_prev=0;
	P_angle.integral_prev=0;
	P_angle.timestamp_prev=0;
	
	PID_current_q.P=0.5;  //航模电机，速度闭环，不能大于1，否则容易失控
	PID_current_q.I=0;    //电流环I参数不太好调试，只用P参数也可以
	PID_current_q.D=0;
	PID_current_q.output_ramp=0;
	PID_current_q.limit=voltage_limit;
	PID_current_q.error_prev=0;
	PID_current_q.output_prev=0;
	PID_current_q.integral_prev=0;
	PID_current_q.timestamp_prev=0;
	
	PID_current_d.P=0.5; 
	PID_current_d.I=0;
	PID_current_d.D=0;
	PID_current_d.output_ramp=0;
	PID_current_d.limit=voltage_limit;
	PID_current_d.error_prev=0;
	PID_current_d.output_prev=0;
	PID_current_d.integral_prev=0;
	PID_current_d.timestamp_prev=0;
}
/******************************************************************************/
//P&I for velocity_PID
float PIDoperator(PIDController* PID,float error)
{
	unsigned long now_us;
	float Ts;
	float proportional,integral,derivative,output;
	float output_rate;
	
	now_us = SysTick->VAL;
	if(now_us<PID->timestamp_prev)Ts = (float)(PID->timestamp_prev - now_us)/12*1e-6f;
	else
		Ts = (float)(0xFFFFFF - now_us + PID->timestamp_prev)/12*1e-6f;
	PID->timestamp_prev = now_us;
	if(Ts == 0 || Ts > 0.5) Ts = 1e-3f;
	
	// u(s) = (P + I/s + Ds)e(s)
	// Discrete implementations
	// proportional part
	// u_p  = P *e(k)
	proportional = PID->P * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	integral = PID->integral_prev + PID->I*Ts*0.5f*(error + PID->error_prev);
	// antiwindup - limit the output
	integral = _constrain(integral, -PID->limit, PID->limit);
	// Discrete derivation
	// u_dk = D(ek - ek_1)/Ts
	derivative = PID->D*(error - PID->error_prev)/Ts;
	
	// sum all the components
	output = proportional + integral + derivative;
	// antiwindup - limit the output variable
	output = _constrain(output, -PID->limit, PID->limit);
	
	// if output ramp defined
	if(PID->output_ramp > 0)
	{
		// limit the acceleration by ramping the output
		output_rate = (output - PID->output_prev)/Ts;
		if(output_rate > PID->output_ramp)output = PID->output_prev + PID->output_ramp*Ts;
		else if(output_rate < -PID->output_ramp)output = PID->output_prev - PID->output_ramp*Ts;
	}
	
	// saving for the next pass
	PID->integral_prev = integral;
	PID->output_prev = output;
	PID->error_prev = error;
	
	return output;
}

void LPF_init(void)
{
	LPF_current_q.Tf=0.05;    
	LPF_current_q.y_prev=0;
	LPF_current_q.timestamp_prev=0;  //SysTick->VAL;
	
	LPF_current_d.Tf=0.05;
	LPF_current_d.y_prev=0;
	LPF_current_d.timestamp_prev=0;
	
	LPF_velocity.Tf=0.0001;   //Tf设置小一点，配合爬升斜率设置PID_velocity.output_ramp，速度切换更平稳；如果没有爬升模式的斜率限制，Tf太小电机容易抖动。
	LPF_velocity.y_prev=0;
	LPF_velocity.timestamp_prev=0;
}

float LPFoperator(LowPassFilter* LPF,float x)
{
	unsigned long now_us;
	float dt, alpha, y;
	
	now_us = SysTick->VAL;
	if(now_us < LPF->timestamp_prev)dt = (float)(LPF->timestamp_prev - now_us)/12*1e-6f;
	else
		dt = (float)(0xFFFFFF - now_us + LPF->timestamp_prev)/12*1e-6f;
	LPF->timestamp_prev = now_us;
	if(dt > 0.3)   //时间过长，大概是程序刚启动初始化，直接返回
	{
		LPF->y_prev = x;
		return x;
	}
	
	alpha = LPF->Tf/(LPF->Tf + dt);
	y = alpha*LPF->y_prev + (1.0f - alpha)*x;
	LPF->y_prev = y;
	
	return y;
}
