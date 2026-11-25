/*
  ******************************************************************************
  * @file    lkf.c
  * @date 	 3 de mar. de 2026
  * @authors Levi e Junior Bandeira
  * @brief   Source code do Linear Kalman Filter para STM32.
  ******************************************************************************
*/

#include <lkf.h>
#include <utils.h>

// Não usando extern LKF_t* f para caso use mais de um filtro por vez.

void LKF_Construtor(LKF_t* f){
    f->init = initPrototipo;
    f->reiniciar = reiniciarPrototipo;
    f->update = updatePrototipo;
}

void initPrototipo(LKF_t* f, float *alt_inicial, float *vel_inicial) {
    f->X[0] = *alt_inicial;
    f->X[1] = 0.0f;
    *vel_inicial = f->X[1];

    f->P[0][0] = 1.0f; f->P[0][1] = 0.0f;
    f->P[1][0] = 0.0f; f->P[1][1] = 1.0f;

    // Matriz Q (Process Noise)
    f->Q[0][0] = 0.01f; f->Q[0][1] = 0.0f;
    f->Q[1][0] = 0.0f;  f->Q[1][1] = 0.1f;

    // Matriz R (Measurement Noise do MS5611)
    f->R = 0.20f;
}

// Reseta o filtro para uma altitude conhecida (usado após a calibração no solo)
// O foguete já subiu 5 metros. O filtro pode estar "viciado" achando que está parado.
// Vamos dizer pra ele: "Eu sei que estou em 5m, mas ABRA A INCERTEZA sobre a velocidade agora!"
void reiniciarPrototipo(LKF_t* f, float *alt_inicial, float *vel_inicial){
    f->X[0] = *alt_inicial;  // Ponto de chute inicial = leitura_inicial_fixa do sensor
    f->X[1] = 0.0f;
    *vel_inicial = f->X[1];

    // Reseta a incerteza para o padrão (1.0)
    f->P[0][0] = 1.0f; f->P[0][1] = 0.0f;
    f->P[1][0] = 0.0f; f->P[1][1] = 1.0f;
}

void updatePrototipo(LKF_t* f, float z_meas, float dt, float *alt_out, float *vel_out) {

    // 1. Predição do Estado (X_pred = F * X)
    float X_pred_0 = f->X[0] + f->X[1] * dt;
    float X_pred_1 = f->X[1];

    // 2. Predição da Covariância (P_pred = F * P * F' + Q)
    float P_pred_00 = f->P[0][0] + dt * (f->P[1][0] + f->P[0][1]) + dt * dt * f->P[1][1] + f->Q[0][0];
    float P_pred_01 = f->P[0][1] + dt * f->P[1][1] + f->Q[0][1];
    float P_pred_10 = f->P[1][0] + dt * f->P[1][1] + f->Q[1][0];
    float P_pred_11 = f->P[1][1] + f->Q[1][1];

    // 3. Ganho de Kalman (K = P_pred * H' * inv(S))
    float S = P_pred_00 + f->R;
    float K_0 = P_pred_00 / S;
    float K_1 = P_pred_10 / S;

    // 4. Atualização do Estado (X = X_pred + K * Y)
    float Y = z_meas - X_pred_0;
    f->X[0] = X_pred_0 + K_0 * Y;
    f->X[1] = X_pred_1 + K_1 * Y;

    // 5. Atualização da Covariância (P_new = (I - K * H) * P_pred)
    // Atribuição direta (mais rápido que memcpy)
    f->P[0][0] = (1.0f - K_0) * P_pred_00;
    f->P[0][1] = (1.0f - K_0) * P_pred_01;
    f->P[1][0] = -K_1 * P_pred_00 + P_pred_10;
    f->P[1][1] = -K_1 * P_pred_01 + P_pred_11;

    // 6. Atualiza as variáveis de saída
    *alt_out = f->X[0];
    *vel_out = f->X[1];
}
