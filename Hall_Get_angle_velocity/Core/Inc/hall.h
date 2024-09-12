#ifndef _HALL_H_
#define _HALL_H_

#include "main.h"

uint8_t GetBldcHall(void);
void updateState(void);
float _normalizeAngle(float angle);
float getAngle(void);
float getVelocity(void);
	
#endif
