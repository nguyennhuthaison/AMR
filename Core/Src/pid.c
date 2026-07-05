#include "pid.h"
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
}
float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float current_value) {
    float error = setpoint - current_value;
    float p_out = pid->Kp * error;
    pid->integral += error * 0.01f;
    if (pid->integral > 500.0f)  pid->integral = 500.0f;
    if (pid->integral < -500.0f) pid->integral = -500.0f;
    float i_out = pid->Ki * pid->integral;
    pid->previous_error = error;
    return p_out+i_out;
}