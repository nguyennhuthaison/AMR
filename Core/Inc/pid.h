#ifndef PID_H
#define PID_H
#include "main.h"
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float previous_error;
    float integral;
} PID_HandleTypeDef;
void PID_Init(PID_HandleTypeDef *pid, float Kp, float Ki, float Kd);
float PID_Compute(PID_HandleTypeDef *pid, float setpoint, float current_value);
#endif // PID_H