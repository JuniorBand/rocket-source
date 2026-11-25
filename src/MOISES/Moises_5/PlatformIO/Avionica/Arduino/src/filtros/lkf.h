/*
  ******************************************************************************
  * @file    lkf.h
  * @brief   Header do Linear Kalman Filter (Altitude e Velocidade Vertical)
  ******************************************************************************
*/

#ifndef LKF_H
#define LKF_H

#include "../macros_config.h"

typedef struct LKF {
    float X[2];      // Estado: [0] = Altitude (m), [1] = Velocidade Vertical (m/s)
    float P[2][2];   // Matriz de Covariância do Erro
    float Q[2][2];   // Matriz de Ruído do Processo
    float R;         // Variância do Ruído de Medida (Barômetro)

    void (*init)(struct LKF *f, float alt_inicial, float *vel_out);
    void (*update)(struct LKF *f, float z_meas, float dt, float *alt_out, float *vel_out);
    void (*reiniciar)(struct LKF *f, float alt_inicial, float *vel_out);
} LKF_t;

void LKF_Construtor(LKF_t *f);
void initPrototipo(LKF_t *f, float alt_inicial, float *vel_out);
void reiniciarPrototipo(LKF_t *f, float alt_inicial, float *vel_out);
void updatePrototipo(LKF_t *f, float z_meas, float dt, float *alt_out, float *vel_out);

#endif /* LKF_H */