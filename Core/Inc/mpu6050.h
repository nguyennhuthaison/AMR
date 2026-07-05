#ifndef MPU6050_H
#define MPU6050_H
#include "main.h"
#include <stdbool.h>
typedef struct {
    //gia toc tuyen tinh, m/s^2
    float accel_x;
    float accel_y;
    float accel_z;
    //toc do goc, rad/s
    float gyro_x;
    float gyro_y;
    float gyro_z;
} MPU6050_Data;
bool MPU6050_Init(I2C_HandleTypeDef *hi2c);
bool MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
#endif