#include "fusion.h"
void Fusion_Update(Odometry_HandleTypeDef *odom, MPU6050_Data *imu)
{
    odom->yaw_imu += imu->gyro_z * dt;
    odom->theta = odom->yaw_imu;
    while (odom->theta > PI)  odom->theta -= 2.0f * PI;
    while (odom->theta < -PI) odom->theta += 2.0f * PI;
}