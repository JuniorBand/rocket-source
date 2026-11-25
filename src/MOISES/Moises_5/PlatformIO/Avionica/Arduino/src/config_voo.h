/*
  ******************************************************************************
  * @file    config_voo.h
  * @brief   Painel de Controle da Missão e Máquina de Estados do Foguete (RAD)
  ******************************************************************************
*/

#ifndef CONFIG_VOO_H
#define CONFIG_VOO_H

#include <macros_config.h>
#include <utils/utils.h>
#include <sensores/bmp280.h>
#include <sensores/mpu6050.h>
#include <filtros/lkf.h>
#include <flash/w25q.h>

// ==============================================================================
// 1. PARÂMETROS FÍSICOS DA MISSÃO (20 Hz -> DT = 0.05 s)
// ==============================================================================
#ifdef EM_VOO
    constexpr float ALTITUDE_LANCAMENTO      = 10.0f;
    constexpr float VELOCIDADE_LANCAMENTO    = 6.0f;
    constexpr float ALTITUDE_POUSO           = 30.0f;
    constexpr float VELOCIDADE_POUSO         = 2.5f;
    constexpr float DESCIDA_MINIMA           = 20.0f;
    constexpr float META_APOGEU              = 500.0f;
    constexpr float DESVIO_MIN               = 15.0f;
    constexpr float ALTITUDE_MIN_EJECAO      = 20.0f;
    constexpr float ALTITUDE_MAIN_PARACHUTE  = 120.0f;
    constexpr u32   TEMPO_MAX_VOO_MS         = 1500000UL; // 25 min
    constexpr u32   TEMPO_MAX_PARAQUEDAS_MS  = 60000UL;   // 60 s
    constexpr u32 TEMPO_MAX_MAIN_MS         = 120000UL;   // 120 s
    #pragma message("Parametros de EM_VOO carregados.")
#else
    constexpr float ALTITUDE_LANCAMENTO      = 2.5f;
    constexpr float VELOCIDADE_LANCAMENTO    = 0.7f;
    constexpr float ALTITUDE_POUSO           = 1.5f;
    constexpr float VELOCIDADE_POUSO         = 1.0f;
    constexpr float DESCIDA_MINIMA           = 3.0f;
    constexpr float META_APOGEU              = 9.0f;
    constexpr float DESVIO_MIN               = 1.0f;
    constexpr float ALTITUDE_MIN_EJECAO      = 3.0f;
    constexpr float ALTITUDE_MAIN_PARACHUTE  = 4.0f;
    constexpr u32   TEMPO_MAX_VOO_MS         = 600000UL;  // 10 min
    constexpr u32   TEMPO_MAX_PARAQUEDAS_MS  = 500000UL;   //
    constexpr u32 TEMPO_MAX_MAIN_MS         = 550000UL;   //
    #pragma message("Parametros de EM_SOLO (bancada) carregados.")
#endif

#ifndef USE_FILTER
    #undef VELOCIDADE_LANCAMENTO
    constexpr float VELOCIDADE_LANCAMENTO    = 0.0f;
#endif

constexpr u8  LAUNCH_CONFIRM_SAMPLES  = 3;    // 3 amostras seguidas (150 ms) para confirmar decolagem
constexpr u8  LANDING_CONFIRM_SAMPLES = 40;   // 40 amostras seguidas (2.0 s) para confirmar pouso
constexpr u32 PYRO_PULSE_DURATION_MS  = 2000; // Duração do pulso nos MOSFETs IRLZ44N (2 s)

// ==============================================================================
// 2. MÁQUINA DE ESTADOS E ESTRUTURAS DE DADOS
// ==============================================================================
typedef enum {
    ESTADO_CALIBRACAO = 0,
    ESTADO_PRONTO,
    ESTADO_EM_VOO,
    ESTADO_APOGEU,
    ESTADO_RECUPERACAO,
    ESTADO_POUSADO,
    ESTADO_ERRO
} EstadoSistema;

typedef struct {
    float altitudeMaxima;
    u32   tempoInicioMs;
    bool  deployAcionado;
    bool  mainAcionado;
    u32   deployTimerMs;
    u32   mainTimerMs;
} SeguroVoo_t;

typedef struct {
    u32           uptimeMs;
    float         pressaoPa;
    float         temperaturaC;
    float         altitudeM;
    float         velocidadeMs;
    i16           accelRaw[3];
    i16           gyroRaw[3];
    u8            imuStatus;
    EstadoSistema estadoAtual;
} DadosVoo_t;

// ==============================================================================
// 3. VARIÁVEIS GLOBAIS EXPORTADAS
// ==============================================================================
extern const char* PRINT_ESTADO[];
extern SeguroVoo_t   seguroVoo;
extern DadosVoo_t    dadosVoo;
extern BMP280_t      bmpSensor;
extern MPU6050_t     mpuSensor;
extern LKF_t         filtroKalman;

extern volatile u8   flagTickVoo;
extern bool          flagGravacaoParada;
extern bool          flagFimDeVoo;
extern bool          sistemaBloqueadoPorLog;

// ==============================================================================
// 4. API PÚBLICA DO CONTROLE DE VOO
// ==============================================================================
void setupVoo(bool watchdogReset);
void processarLogicaVoo(void);
bool recalibrarERearmarSistema(void);
void forcarErroSistema(const char *motivo);
void simularVooAoVivoUSB(void);

// Controle sonoro (Buzzer ativo em HIGH no pino PB1)
void beeperClear(void);
void beeperUpdate(void);
void playBeeps(u8 count, u16 onMs, u16 offMs);
void beepBlocking(u16 durationMs, u8 repetitions = 1);
//void feedWatchdog(void);

#ifdef USE_BUZZER
    #define BUZZER_ON()  digitalWrite(PIN_BUZZER, HIGH)
    #define BUZZER_OFF() digitalWrite(PIN_BUZZER, LOW)
#else
    #define BUZZER_ON()  do { } while(0)
    #define BUZZER_OFF() do { } while(0)
#endif

#endif /* CONFIG_VOO_H */