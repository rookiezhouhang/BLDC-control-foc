#ifndef BLDCMotor_H
#define BLDCMotor_H

extern long sensor_direction;
extern float voltage_power_supply;
extern float voltage_limit;
extern float voltage_sensor_align;
extern int  pole_pairs;
extern unsigned long open_loop_timestamp;
extern float velocity_limit;
extern float current_limit;

void move(float new_target);
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);
void setPhaseVoltage(float Uq, float Ud, float angle_el);
void loopFOC(void);

#endif
