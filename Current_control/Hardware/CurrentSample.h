#ifndef CURRENTSAMPLE_H
#define CURRENTSAMPLE_H

#include "foc_utils.h" 

float getDCCurrent(float motor_electrical_angle);
DQCurrent_s getFOCCurrents(float angle_el);
PhaseCurrent_s getPhaseCurrents(void);

#endif
