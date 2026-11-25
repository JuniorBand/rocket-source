/*
  ******************************************************************************
  * @file    lkf.cpp
  * @brief   Implementação do Linear Kalman Filter para STM32
  ******************************************************************************
*/

#include "lkf.h"

void LKF_Construtor(LKF_t *f) {
    f->init      = initPrototipo;
    f->reiniciar = reiniciarPrototipo;
    f->update    = updatePrototipo;
}

void initPrototipo(LKF_t *f, float alt_inicial, float *vel_out) {
    f->X[0] = alt_inicial;
    f->X[1] = 0.0f;
    if (vel_out) *vel_out = 0.0f;

    f->P[0][0] = 1.0f; f->P[0][1] = 0.0f;
    f->P[1][0] = 0.0f; f->P[1][1] = 1.0f;

    f->Q[0][0] = LKF_Q_ALT; f->Q[0][1] = 0.0f;
    f->Q[1][0] = 0.0f;      f->Q[1][1] = LKF_Q_VEL;

    f->R = LKF_R_ALT;
}

void reiniciarPrototipo(LKF_t *f, float alt_inicial, float *vel_out) {
    f->X[0] = alt_inicial;
    f->X[1] = 0.0f;
    if (vel_out) *vel_out = 0.0f;

    f->P[0][0] = 1.0f; f->P[0][1] = 0.0f;
    f->P[1][0] = 0.0f; f->P[1][1] = 1.0f;
}

void updatePrototipo(LKF_t *f, float z_meas, float dt, float *alt_out, float *vel_out) {
    // 1. Predição do Estado (X_pred = F * X)
    const float X_pred_0 = f->X[0] + f->X[1] * dt;
    const float X_pred_1 = f->X[1];

    // 2. Predição da Covariância (P_pred = F * P * F' + Q)
    const float P_pred_00 = f->P[0][0] + dt * (f->P[1][0] + f->P[0][1]) + dt * dt * f->P[1][1] + f->Q[0][0];
    const float P_pred_01 = f->P[0][1] + dt * f->P[1][1] + f->Q[0][1];
    const float P_pred_10 = f->P[1][0] + dt * f->P[1][1] + f->Q[1][0];
    const float P_pred_11 = f->P[1][1] + f->Q[1][1];

    // 3. Ganho de Kalman (K = P_pred * H' * inv(S))
    const float S   = P_pred_00 + f->R;
    const float K_0 = P_pred_00 / S;
    const float K_1 = P_pred_10 / S;

    // 4. Atualização do Estado (X = X_pred + K * Y)
    const float Y = z_meas - X_pred_0;
    f->X[0] = X_pred_0 + K_0 * Y;
    f->X[1] = X_pred_1 + K_1 * Y;

    // 5. Atualização da Covariância (P_new = (I - K * H) * P_pred)
    f->P[0][0] = (1.0f - K_0) * P_pred_00;
    f->P[0][1] = (1.0f - K_0) * P_pred_01;
    f->P[1][0] = -K_1 * P_pred_00 + P_pred_10;
    f->P[1][1] = -K_1 * P_pred_01 + P_pred_11;

    // 6. Saídas filtradas
    if (alt_out) *alt_out = f->X[0];
    if (vel_out) *vel_out = f->X[1];
}