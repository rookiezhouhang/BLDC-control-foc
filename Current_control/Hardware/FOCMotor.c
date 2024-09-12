#include "Myproject.h"

float shaft_angle; //»úÐµ½Ç¶È
float electrical_angle;
float shaft_velocity;
float current_sp;
float shaft_velocity_sp;
float shaft_angle_sp;
DQVoltage_s voltage;
DQCurrent_s current;

//extern int direction;

TorqueControlType torque_controller;
MotionControlType controller;

float shaftVelocity(void)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_velocity;
  return LPFoperator(&LPF_velocity,getVelocity());
}
