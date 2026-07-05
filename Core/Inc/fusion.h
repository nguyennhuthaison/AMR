#ifndef _FUSION_H
#define _FUSION_H
#include "main.h"
#include "odometry.h"
#include "mpu6050.h"
#define dt 0.01f //vi phan thoi gian
void Fusion_Update(Odometry_HandleTypeDef *odom, MPU6050_Data *imu);
#endif /* _FUSION_H */