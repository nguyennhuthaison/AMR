#include "odometry.h"
#include <math.h>
void Odometry_Init(Odometry_HandleTypeDef *odom) 
{
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
    odom->v = 0.0f;
    odom->w = 0.0f;
}
void Odometry_Update(Odometry_HandleTypeDef *odom, Encoder_HandleTypeDef *encL, Encoder_HandleTypeDef *encR) 
{

    float delta_s_L = 0.0f;
    float delta_s_R = 0.0f;
    delta_s_L = encL->delta_pulses * (2.0f * PI * WHEEL_BASE) / ENCODER_PPR;
    delta_s_R = encR->delta_pulses * (2.0f * PI * WHEEL_BASE) / ENCODER_PPR;
    //2. Tinh toan quang duong va goc xoay tuc thoi
    float delta_s = (delta_s_R + delta_s_L) / 2.0f;
    float delta_theta = (delta_s_R - delta_s_L) / WHEEL_BASE;
    odom->v = delta_s / DT;
    odom->w = delta_theta / DT;
    //3. Cap nhat toa do va goc xoay
    float theta_mid = odom->theta + delta_theta/2.0f;

    odom->x += delta_s * cosf(theta_mid);
    odom->y += delta_s * sinf(theta_mid);
    odom->theta += delta_theta;
    //4. Chuan hoa goc xoay ve khoang -pi den pi
    while (odom->theta > PI)
    {
        odom->theta -= 2.0f * PI;
    }

    while (odom->theta < -PI)
    {
        odom->theta += 2.0f * PI;
    }
}
