#include "ekf.h"
#include "main.h"
#include "math.h"
void equal_matrix(float A[3][3], float result[3][3]){
    
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            result[i][j] = A[i][j];
        }
    }
}
void plus_matrix(float A[3][3], float B[3][3], float result[3][3]){
    
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}
void minus_matrix(float A[3][3], float B[3][3], float result[3][3]){
    
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}
void multiply_matrix(float A[3][3], float B[3][3], float result[3][3])
{
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            result[i][j] = 0;
            for(int k=0;k<3;k++)
            {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}
void transpose_matrix(float A[3][3], float result[3][3])
{
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            result[j][i] = A[i][j];
        }
    }
}
void EKF_Init(EKF *ekf)
{
    ekf->x[0] = 0.0f;
    ekf->x[1] = 0.0f;
    ekf->x[2] = 0.0f;
    ekf->F[0][0] = 1.0f;ekf->F[0][1] = 0.0f;ekf->F[0][2] = 0.0f;
    ekf->F[1][0] = 0.0f;ekf->F[1][1] = 1.0f;ekf->F[1][2] = 0.0f;
    ekf->F[2][0] = 0.0f;ekf->F[2][1] = 0.0f;ekf->F[2][2] = 1.0f;

    ekf->P[0][0] = 0.01f;ekf->P[0][1] = 0.0f;ekf->P[0][2] = 0.0f;
    ekf->P[1][0] = 0.0f;ekf->P[1][1] = 0.01f;ekf->P[1][2] = 0.0f;
    ekf->P[2][0] = 0.0f;ekf->P[2][1] = 0.0f;ekf->P[2][2] = 0.001f;

    ekf->Q[0][0] = 0.001f;ekf->Q[0][1] = 0.0f;ekf->Q[0][2] = 0.0f;
    ekf->Q[1][0] = 0.0f;ekf->Q[1][1] = 0.001f;ekf->Q[1][2] = 0.0f;
    ekf->Q[2][0] = 0.0f;ekf->Q[2][1] = 0.0f;ekf->Q[2][2] = 0.02f;
}
void EKF_predict_state(EKF *ekf, float vL, float vR, float dt)
{
    float v = (vL + vR) * 0.5f;//van toc robot
    float w = (vR - vL) / WHEEL_BASE;//goc xoay robot

    ekf->x[0] += v * cosf(ekf->x[2]) * dt;//cap nhat vi tri x
    ekf->x[1] += v * sinf(ekf->x[2]) * dt;//cap nhat vi tri y
    ekf->x[2] += w * dt;//cap nhat goc xoay theta
    while (ekf->x[2] > PI)
    {
        ekf->x[2] -= 2.0f * PI;
    }

    while (ekf->x[2] < -PI)
    {
        ekf->x[2] += 2.0f * PI;
    }
}
void EKF_predict_covariance(EKF *ekf, float vL, float vR, float dt){
    float v = (vL + vR) * 0.5f; // Van toc dai hien tai của robot
    ekf->F[0][2] = -v * sinf(ekf->x[2]) * dt; // dao ham x theo theta
    ekf->F[1][2] =  v * cosf(ekf->x[2]) * dt; //dao ham y theo theta
    float result[3][3];//ma tran luu tam ket qua F.P
    multiply_matrix(ekf->F, ekf->P, result);//result = F.P
    float F_transpose[3][3];
    transpose_matrix(ekf->F, F_transpose);
    float result2[3][3];//ma tran luu tam ket qua F.P.F^T
    multiply_matrix(result, F_transpose, result2);//result2 = F.P.F^T
    float result3[3][3];//ma tran luu tam ket qua F.P.F^T + Q
    plus_matrix(result2, ekf->Q, result3);//result3 = F.P.F^T + Q
    equal_matrix(result3, ekf->P);//P = F.P.F^T + Q
}
void EKF_update_state(EKF *ekf, float yaw_imu){
    float yk = yaw_imu - ekf->x[2];//sai so giua goc do tu mpu6050 va goc do du doan
    while (yk > PI)
    {
        yk -= 2.0f * PI;
    }
    while (yk < -PI)
    {
        yk += 2.0f * PI;
    }
    float S = ekf->P[2][2] + R_imu;
    float K[3];//ma tran Kalman gain
    K[0] = ekf->P[0][2] / S;
    K[1] = ekf->P[1][2] / S;
    K[2] = ekf->P[2][2] / S;
    //cap nhat trang thai
    ekf->x[0] += K[0] * yk;
    ekf->x[1] += K[1] * yk;
    ekf->x[2] += K[2] * yk;
    
    //cap nhat ma tran P
    float tempt[3][3];
    tempt[0][0] = 1.0f;tempt[0][1] =0;    tempt[0][2] = -K[0];
    tempt[1][0] = 0.0f;tempt[1][1] = 1.0f;tempt[1][2] = -K[1];
    tempt[2][0] = 0.0f;tempt[2][1] = 0.0f;tempt[2][2] = 1.0f - K[2];
    float result[3][3];
    multiply_matrix(tempt, ekf->P, result);
    equal_matrix(result, ekf->P);
}
