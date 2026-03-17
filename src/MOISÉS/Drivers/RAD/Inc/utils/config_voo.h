/*
  ******************************************************************************
  * @file    config_voo.h
  * @date 	 14 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Header do arquivo que configura a parte complexa do código do foguete.
  ******************************************************************************
*/

#ifndef CONFIG_VOO_H
#define CONFIG_VOO_H

#include "stm32f4xx_hal.h"
#include "utils.h"
#include "lkf.h"

//typedef struct MS5611_s MS5611_t;

// Máquina de Estados do Voo
typedef enum EstadoSistema_s {
    ESTADO_CALIBRACAO,      // Inicialização = 0
    ESTADO_PRONTO,          // Na Rampa (Modo Silencioso) = 1
    ESTADO_EM_VOO,          // Subindo (Gravando) = 2
	ESTADO_APOGEU,			// Subindo (Gravando) = 3
    ESTADO_RECUPERACAO,     // Caindo (Gravando) = 4
    ESTADO_POUSADO,         // No Chão (Finalizado) = 5
	ESTADO_ERRO				// Erro = 6
} EstadoSistema;

// Dados Estat�sticos do Voo
typedef struct SeguroVoo_s {
    float altitude_maxima;
    uint32_t tempo_inicio_ms;
} SeguroVoo_t;

typedef struct CaixaPreta_s {
    float pressao_solo;         // Guarda a pressao do chao para nao perder o "zero" da altitude
    uint32_t status_voo;        // Flag magica: 0xAA55AA55 (Voando) ou 0 (Parado)
} CaixaPreta_t;

typedef struct DadosVoo_s {
	uint32_t magicNumber;
    float pressaoAtual;          // Press�o em hPa
    float temperaturaAtual;
    float altitudeAtual;         // Altitude em Metros
    float velocidadeAtual;
    EstadoSistema estadoAtual;
} __attribute__((aligned(4))) DadosVoo_t;

extern const char* PRINT_ESTADO[];
extern SeguroVoo_t seguroVoo;
extern DadosVoo_t dadosVoo;
extern CaixaPreta_t caixaPreta;
extern RTC_HandleTypeDef hrtc;
extern volatile uint8_t flagTickVoo;

void beep(int duracao, int vezes);
void setupVoo(SPI_HandleTypeDef *hspi_sensor, TIM_HandleTypeDef *htim);
void processarLogicaVoo(void);


#endif /* CONFIG_VOO_H */
