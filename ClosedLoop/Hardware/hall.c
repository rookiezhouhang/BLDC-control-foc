#include "Myproject.h"
#include <cmath>

volatile int8_t electric_sector;
volatile long electric_rotations;
volatile int8_t hall_state;//hall状态
volatile int direction;//转动方向
volatile long pulse_diff;//每次CC1中断的时间间隔
volatile unsigned long pulse_timestamp;

const int8_t ELECTRIC_SECTORS[8] = { -1,  0,  4,  5,  2,  1,  3 , -1 };
int cpr = 7*6;//极对数*6

uint8_t GetBldcHall(void)
{
	uint8_t ret = 0;
	
	ret = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15);
	ret <<=1;
	ret |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3);
	ret <<=1;
	ret |= HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10);
	
	return ret;
}

//每次CC1中断时进入该函数更新HALL相关参数
void updateState(void)
{
	long new_pulse_timestamp = SysTick->VAL;
	
	int8_t new_hall_state = GetBldcHall();
	if (new_hall_state == hall_state) {
		return;
	}
	hall_state = new_hall_state;
	
	int8_t new_electric_sector = ELECTRIC_SECTORS[hall_state];
	static int old_direction;
	if (new_electric_sector - electric_sector > 3) {
		//underflow
		direction = -1;//CCW
		electric_rotations += direction;
	} else if (new_electric_sector - electric_sector < (-3)) {
		//overflow
		direction = 1;//CW;
		electric_rotations += direction;
	} else {
		direction = (new_electric_sector > electric_sector)? 1 : -1;
	}
	electric_sector = new_electric_sector;
	
	if (direction == old_direction) {
		// not oscilating or just changed direction
		if(new_pulse_timestamp<pulse_timestamp){
			pulse_diff = pulse_timestamp - new_pulse_timestamp;
		}
		else{
			pulse_diff = 0xFFFFFF - new_pulse_timestamp + pulse_timestamp;
		}
		
	} else {
		pulse_diff = 0;
	}
	
	pulse_timestamp = new_pulse_timestamp;
	old_direction = direction;
}

//计算得到电角度
float getAngle(void)
{
	float Angle = ((float)(electric_rotations * 6 + electric_sector) / (float)cpr) * _2PI ;
	return Angle;
}

float getVelocity(void)
{
	
	long last_pulse_timestamp = pulse_timestamp;
	long last_pulse_diff = pulse_diff;
	
	// last velocity isn't accurate if too old
	if (last_pulse_diff == 0 || ((long)(SysTick->VAL - last_pulse_timestamp) > last_pulse_diff*2) ) { 
		return 0;
	} 
	else {
		return (direction * (_2PI / (float)cpr) / (last_pulse_diff / 12000000.0f));
	}
	
}
