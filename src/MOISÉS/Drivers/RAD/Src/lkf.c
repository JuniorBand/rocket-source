/*
  ******************************************************************************
  * @file    lkf.c
  * @date 	 3 de mar. de 2026
  * @authors  Levi e Júnior Bandeira
  * @brief   Source code do Linear Kalman Filter para STM32.
  ******************************************************************************
*/

#include "lkf.h"
#include <string.h>
#include "utils.h"

// Não usando extern LKF_t* f para caso use mais de um filtro por vez.


void LKF_Construtor(LKF_t* f){
	f->init = initPrototipo;
	f->reiniciar = reiniciarPrototipo;
	f->update = updatePrototipo;
}



void initPrototipo(LKF_t* f, float *alt_inicial, float *vel_inicial) {
    f->X[0] = *alt_inicial; f->X[1] = 0.0f; *vel_inicial = f->X[1];
    f->P[0][0] = 1.0f; f->P[0][1] = 0.0f; f->P[1][0] = 0.0f; f->P[1][1] = 1.0f;
    f->Q[0][0] = 0.01f; f->Q[0][1] = 0.0f; f->Q[1][0] = 0.0f; f->Q[1][1] = 0.1f;
    f->R = 0.20f;
}

// Reseta o filtro para uma altitude conhecida (usado após a calibração no solo)
// O foguete já subiu 5 metros. O filtro pode estar "viciado" achando que está parado.
// Vamos dizer pra ele: "Eu sei que estou em 5m, mas ABRA A INCERTEZA sobre a velocidade agora!"
void reiniciarPrototipo(LKF_t* f, float *alt_inicial, float *vel_inicial){

	f->X[0] = *alt_inicial; // Ponto de chute inicial = leitura_inicial_fiXa do sensor
	f->X[1] = 0.0f;
	*vel_inicial = f->X[1];

    // Reseta a incerteza para o padrão (1.0)
    f->P[0][0] = 1.0f; f->P[0][1] = 0.0f;
    f->P[1][0] = 0.0f; f->P[1][1] = 1.0f;

}

void updatePrototipo(LKF_t* f, float z_meas, float dt, float *alt_out, float *vel_out) {
    // Predi��o
    float X_pred[2];
    // Xpred = F * X
    X_pred[0] = f->X[0] + f->X[1] * dt;
    X_pred[1] = f->X[1];

    float P_pred[2][2];
	// Ppred = F * P * F' + Q, onde F � a matriz de transi��o do estado linearizado
    P_pred[0][0] = f->P[0][0] + dt * (f->P[1][0] + f->P[0][1]) + dt * dt * f->P[1][1] + f->Q[0][0];
    P_pred[0][1] = f->P[0][1] + dt * f->P[1][1] + f->Q[0][1];
    P_pred[1][0] = f->P[1][0] + dt * f->P[1][1] + f->Q[1][0];
    P_pred[1][1] = f->P[1][1] + f->Q[1][1];

    // Atualiza��o
    float K[2], S = P_pred[0][0] + f->R;
	// S = H * P_pred * H' + R, onde H = [1 0] (medida de altitude)
	// K = P_pred * H' * inv(S), onde H = [1 0] (medida de altitude)
    K[0] = P_pred[0][0] / S; K[1] = P_pred[1][0] / S;

	// Z = Valor recebido pelo sensor (medida de altitude)
	// Y = Z - H * Xpred, onde H = [1 0] (medida de altitude)
	// X = Xpred + K * Y
    f->X[0] = X_pred[0] + K[0] * (z_meas - X_pred[0]);
    f->X[1] = X_pred[1] + K[1] * (z_meas - X_pred[0]);

    float P_new[2][2];
	// P_new = (I - K * H) * P_pred, onde H = [1 0] (medida de altitude)
    P_new[0][0] = (1.0f - K[0]) * P_pred[0][0];
    P_new[0][1] = (1.0f - K[0]) * P_pred[0][1];
    P_new[1][0] = -K[1] * P_pred[0][0] + P_pred[1][0];
    P_new[1][1] = -K[1] * P_pred[0][1] + P_pred[1][1];
	memcpy(f->P, P_new, sizeof(P_new)); // Atualiza a covari�ncia do filtro

    *vel_out = f->X[1];
    *alt_out = f->X[0];
}
