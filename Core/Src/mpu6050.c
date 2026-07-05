#include "stdint.h"
#include "mpu6050.h"
static int16_t accel_x_raw =0;
static int16_t accel_y_raw =0;
static int16_t accel_z_raw =0;
static int16_t gyro_x_raw =0;
static int16_t gyro_y_raw =0;
static int16_t gyro_z_raw =0;
bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{  
    HAL_StatusTypeDef i2c_status;
    uint8_t wake_up_data = 0x00;
    i2c_status = HAL_I2C_Mem_Write(hi2c, 0x68 << 1, 0x6B, I2C_MEMADD_SIZE_8BIT, &wake_up_data, 1, 100);

     if (i2c_status != HAL_OK) {
      return false;
     }
    return true;
}
bool MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data)
{
    uint8_t accel_buffer[6];
    uint8_t gyro_buffer[6];
    if (HAL_I2C_Mem_Read(hi2c, 0x68 << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, accel_buffer, 6, 100) != HAL_OK) {
        return false;
    }
    accel_x_raw = (int16_t)((accel_buffer[0] << 8) | accel_buffer[1]);
    accel_y_raw = (int16_t)((accel_buffer[2] << 8) | accel_buffer[3]);
    accel_z_raw = (int16_t)((accel_buffer[4] << 8) | accel_buffer[5]);

    if (HAL_I2C_Mem_Read(hi2c, 0x68 << 1, 0x43, I2C_MEMADD_SIZE_8BIT, gyro_buffer, 6, 100) != HAL_OK) {
        return false;
    }
    gyro_x_raw = (int16_t)((gyro_buffer[0] << 8) | gyro_buffer[1]);
    gyro_y_raw = (int16_t)((gyro_buffer[2] << 8) | gyro_buffer[3]);
    gyro_z_raw = (int16_t)((gyro_buffer[4] << 8)  | gyro_buffer[5]);


    data->accel_x = ((float)accel_x_raw / 16384.0f) * 9.80665f;
    data->accel_y = ((float)accel_y_raw / 16384.0f) * 9.80665f;
    data->accel_z = ((float)accel_z_raw / 16384.0f) * 9.80665f;


    data->gyro_x = ((float)gyro_x_raw / 131.0f) * PI / 180.0f;
    data->gyro_y = ((float)gyro_y_raw / 131.0f) * PI / 180.0f;
    data->gyro_z = ((float)gyro_z_raw / 131.0f) * PI / 180.0f;
    return true;
}