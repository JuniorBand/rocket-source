/*
  ******************************************************************************
  * @file    config_voo.h
  * @date 	 14 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Header do arquivo que configura a parte complexa do código do foguete.
  * @brief   PAINEL DE CONTROLE DA MISSÃO
  ******************************************************************************
*/

#ifndef CONFIG_VOO_H
#define CONFIG_VOO_H

#include <stm32f4xx_hal.h>
#include <macros_config.h>
#include <utils.h>
#include <lkf.h>
#include <main.h>

// ==============================================================================
// PARÂMETROS FÍSICOS DA MISSÃO
// ==============================================================================
// ATENÇÃO: Use o f para forçar o compilador entender que é float.
// O Tempo de amostragem do sistema (10 ms = 0.01 segundos). Fundamental para o Kalman.
#define DT 0.01f

#ifdef EM_VOO

	#define ALTITUDE_LANCAMENTO  5.0f
	#define VELOCIDADE_LANCAMENTO 8.0f // playing it safe aqui
	#define ALTITUDE_POUSO       3.0f
	#define VELOCIDADE_POUSO     1.5f
	#define DESCIDA_MINIMA       10.0f
	#define META_APOGEU          500.0f // Defina a meta em metros aqui
	#define DESVIO_MIN           1.0f
	#define ALTITUDE_MIN_EJECAO  20.0f
	#define TEMPO_MAX_VOO_MS     240000UL  // (4 minutos) Tempo maximo de voo para parar de gravar

	#pragma message("Parâmetros do EM_VOO definidos.")

#else // EM_SOLO: testes em bancada
	#define ALTITUDE_LANCAMENTO  1.5f
	#define VELOCIDADE_LANCAMENTO 0.7f
	#define ALTITUDE_POUSO       0.5f
	#define VELOCIDADE_POUSO     0.8f
	#define DESCIDA_MINIMA       2.0f
	#define META_APOGEU          9.0f  // Meta menor para testes
	#define DESVIO_MIN           0.0f
	#define ALTITUDE_MIN_EJECAO  1.0f
	#define TEMPO_MAX_VOO_MS     1000000UL

	#pragma message("Parâmetros do EM_SOLO definidos.")
#endif


#ifndef USE_FILTER
	#define VELOCIDADE_LANCAMENTO 0.0f
#endif


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
    u32 tempo_inicio_ms;
} SeguroVoo_t;

typedef struct CaixaPreta_s {
    float pressao_solo;         // Guarda a pressao do chao para nao perder o "zero" da altitude
    u32 status_voo;        // Flag magica: 0xAA55AA55 (Voando) ou 0 (Parado)
} CaixaPreta_t;

typedef struct DadosVoo_s {
	u32 magicNumber;
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
extern volatile u8 flagTickVoo;
extern u8 flagGravacaoParada;
extern u8 flagFimDeVoo;

void beep(u32 duracao, u8 vezes);
void setupVoo(SPI_HandleTypeDef *hspi_mem, SPI_HandleTypeDef *hspi_sensor, TIM_HandleTypeDef *htim_ms, RTC_HandleTypeDef *hrtc_sys);
void processarLogicaVoo(void);
void simularVooAoVivoUSB(void);

#if defined(BUZZER_PORT) && defined(BUZZER_PIN) && defined(USE_BUZZER)
	// Se o seu buzzer liga com LOW (Lógica Invertida / Active-Low)
	#pragma message("BUZZER_PORT e BUZZER_PIN foram definidos.")
	#define BUZZER_ON()  writePinLow(BUZZER_PORT, BUZZER_PIN)
	#define BUZZER_OFF() writePinHigh(BUZZER_PORT, BUZZER_PIN)
#else
	#warning "BUZZER_PORT e BUZZER_PIN não foram definidos no main.h!"
	#define BUZZER_ON()  do { } while(0)
	#define BUZZER_OFF() do { } while(0)
#endif



#endif /* CONFIG_VOO_H */
