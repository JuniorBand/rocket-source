/*
  ******************************************************************************
  * @file    lkf.h
  * @date 	 3 de mar. de 2026
  * @authors  Levi e Júnior Bandeira
  * @brief   Header do Linear Kalman Filter para STM32.
  ******************************************************************************
*/

#ifndef LKF_H
#define LKF_H


// Estrutura do Filtro Kalman (LKF)
typedef struct LKF {
    float X[2];      // Estado: [0]=Altitude, [1]=Velocidade
    float P[2][2];   // Matriz de Covari�ncia
    float Q[2][2];   // Ru�do do Processo
    float R;         // Ru�do da Medida
    void (*init)(struct LKF* f, float *alt_inicial, float *vel_inicial);
    void (*update)(struct LKF* f, float z_meas, float dt, float *alt_out, float *vel_out);
    void (*reiniciar)(struct LKF* f, float *alt_inicial, float *vel_inicial);

} LKF_t;

void LKF_Construtor(LKF_t* f);
void initPrototipo(LKF_t* f, float *alt_inicial, float *vel_inicial);
void reiniciarPrototipo(LKF_t* f, float *alt_inicial, float *vel_inicial);
void updatePrototipo(LKF_t* f, float z_meas, float dt, float *alt_out, float *vel_out);


#endif /* LKF_H */
