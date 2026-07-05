#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "main.h"
#include "encoder.h"
#define WHEEL_BASE 0.15f // Khoảng cách giữa hai bánh xe (m)
typedef struct {
    float x;//toa do x
    float y;//toa do y
    float theta;//goc xoay(radian)
    float yaw_imu;//goc xoay tu mpu6050
} Odometry_HandleTypeDef;
void Odometry_Init(Odometry_HandleTypeDef *odom);
void Odometry_Update(Odometry_HandleTypeDef *odom, Encoder_HandleTypeDef *encL, Encoder_HandleTypeDef *encR);
#endif /* ODOMETRY_H */ 