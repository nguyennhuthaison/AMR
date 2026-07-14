#ifndef __EKF_H
#define __EKF_H
#define R_imu 0.001f //do lech giua goc do tu mpu6050 va goc do du doan
typedef struct {
    float x[3]; // State vector [x, y, theta]
    float F[3][3]; // State transition matrix
    float P[3][3]; // Covariance matrix
    float Q[3][3]; // Process noise covariance
    float R[3][3]; // Measurement noise covariance
}EKF;
// Cac ham tinh toan ma tran cho EKF
void equal_matrix(float A[3][3], float result[3][3]);//result = A
void plus_matrix(float A[3][3], float B[3][3], float result[3][3]);//result = A + B
void minus_matrix(float A[3][3], float B[3][3], float result[3][3]);//result = A - B
void multiply_matrix(float A[3][3], float B[3][3], float result[3][3]);//result = A * B
void transpose_matrix(float A[3][3], float result[3][3]);//result = A^T


//cac ham cua EKF
void EKF_Init(EKF *ekf);
//cac ham du doan
void EKF_predict_state(EKF *ekf, float vL, float vR, float dt);
void EKF_predict_covariance(EKF *ekf, float vL, float vR, float dt);
//cac ham cap nhat
void EKF_update_state(EKF *ekf, float yaw_imu);
#endif // __EKF_H