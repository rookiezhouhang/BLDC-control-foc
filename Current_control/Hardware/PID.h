#ifndef PID_H
#define PID_H


typedef struct 
{
    float P; //!< Proportional gain 
    float I; //!< Integral gain 
    float D; //!< Derivative gain 
    float output_ramp; //!< Maximum speed of change of the output value
    float limit; //!< Maximum output value
    float error_prev; //!< last tracking error value
    float output_prev;  //!< last pid output value
    float integral_prev; //!< last integral component value
    unsigned long timestamp_prev; //!< Last execution timestamp
} PIDController;

typedef struct 
{
	float Tf; //!< Low pass filter time constant
	float y_prev; //!< filtered value in previous execution step 
	unsigned long timestamp_prev;  //!< Last execution timestamp
} LowPassFilter;

extern LowPassFilter  LPF_current_q,LPF_current_d,LPF_velocity;
extern PIDController  PID_current_q,PID_current_d,PID_velocity,P_angle;

void PID_init(void);
void LPF_init(void);
float PIDoperator(PIDController* PID,float error);
float LPFoperator(LowPassFilter* LPF,float x);

#endif
