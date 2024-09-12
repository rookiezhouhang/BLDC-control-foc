#ifndef BLDCMotor_H
#define BLDCMotor_H

void move(float new_target);
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
void setPhaseVoltage(float Uq, float Ud, float angle_el);

#endif
