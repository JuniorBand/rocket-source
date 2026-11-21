/*
  ******************************************************************************
  * @file    config_voo.h
  * @date 	 14 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Header do arquivo que configura a parte complexa do codigo do foguete.
  * @brief   PAINEL DE CONTROLE DA MISSÃO
  ******************************************************************************
*/

#ifndef CONFIG_VOO_H
#define CONFIG_VOO_H

#include <macros_config.h>
#include <utils.h>
#include <prints.h>
#include <config_voo.h>
#include <bmp280.h>
#include <mpu6050.h>
#include <w25q.h>
#include <telemetria_com.h>
#include <usb_com.h>

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SPIFlash.h>
#include <IWatchdog.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================
// CONFIGURAÇÕES
// ============================================================
#define DT 0.05f // Intervalo para o filtro de Kalman (50 ms = 20 Hz)

#ifdef EM_VOO
    static const u32 TEMPO_MAX_VOO_MS = 600000UL; // 10 minutos
    static const u32 TEMPO_MAX_PARAQUEDAS_MS= 60000UL; // 1 minuto
#else // EM_SOLO: testes em bancada
    static const u32 TEMPO_MAX_VOO_MS = 600000UL; // 10 minutos
    static const u32 TEMPO_MAX_PARAQUEDAS_MS= 60000UL; // 1 minuto
#endif


static const u32 SAMPLE_INTERVAL_MS      = 50;     // 20 Hz de amostragem e gravação
static const u32 TELEMETRY_INTERVAL_MS   = 200;    // 5 Hz de telemetria na serial
static const u32 PRETRIGGER_MS           = 2000;   // histórico gravado antes do lançamento
static const u16 PRETRIGGER_SAMPLES      = PRETRIGGER_MS / SAMPLE_INTERVAL_MS;

static const float    ALTITUDE_MIN_EJECAO      = 3.0f;
static const u8  QUANTIDADE_MIN_DADOS_EJECAO  = 3;      // amostras seguidas acima do limiar

static const u8  CALIB_SAMPLES           = 32;
static const u8  CALIB_MIN_VALID         = 24;
static const u32 CALIB_SETTLE_MS         = 300;


static const u32 WATCHDOG_TIMEOUT_US     = 4000000; // 4 s
static const u32 USB_WAIT_MS             = 1500;

static const u32 READY_CHIRP_INTERVAL_MS = 5000;   // 0 desativa o bipe de "armado"
static const float    VZ_FILTER_ALPHA         = 0.2f;

static const float    PRESSURE_MIN_PA         = 30000.0f;
static const float    PRESSURE_MAX_PA         = 120000.0f;
static const u32 MPU_RETRY_MS            = 3000;
static const u32 UART_TTL_BAUD           = 115200;  // UART binaria 8N1
static const u16 MPU_CAL_SAMPLES         = 96;
static const u8  MPU_FAIL_LIMIT          = 5;

// Configuracao do MPU6050: +/-16g (2048 LSB/g); +/-2000 dps (16.4 LSB/dps).
// Valores RAW sao gravados, sem subtrair 1g da gravidade nem o offset do giroscopio.
static const float MPU_ACC_TO_M_S2 = 9.80665f / 2048.0f;
static const float MPU_GYR_TO_DPS  = 1.0f / 16.4f;

// ============================================================
// LAYOUT DA FLASH - v3 (32 B por registro) + leitura de v2 (16 B)
// ============================================================
// 0x000000..0x0000FF: cabecalho de 32 bytes, restante reservado na pagina.
// 0x000100..fim: registros contiguos de 32 bytes (8 por pagina SPI Flash).
// Versao v2 anterior permanece legivel com V, mas NAO eh mais gravavel.
// O comando A apaga a memoria externa W25Q64 e cria um novo voo v3.

static const u32 HEADER_ADDR      = 0x000000;
static const u32 DATA_START_ADDR  = 0x000100;
static const u32 FLASH_PAGE_SIZE  = 256;
static const u32 FLASH_BLOCK_SIZE = 65536;

static const u32 HEADER_MAGIC     = 0x31544C46; // "FLT1"
static const u16 HEADER_VERSION   = 3;
static const u16 LEGACY_VERSION   = 2;

typedef struct CaixaPreta_s { // Comeco da flash (BlackBox)
    u32 magic;
    u16 version;
    u16 recordSize;
    u16 sampleIntervalMs;
    u16 pretriggerDadosVoo_ts;
    float    basePressurePa;
    float    baseTemperatureC;
    float    launchThresholdM;
    i16  gyroBiasRaw[3];  // novo v3; ocupava reserved[0..5] no v2
    u8  mpuCalibrated;   // novo v3; ocupava reserved[6] no v2
    u8  crc8;
} CaixaPreta_t;


// Checking
static_assert(sizeof(CaixaPreta_t) == 32, "Cabecalho deve ter 32 bytes");
static_assert(sizeof(LogRecordV2) == 16, "Registro v2 deve ter 16 bytes");
static_assert(sizeof(LogRecord) == 32, "Registro v3 deve ter 32 bytes");
static_assert(FLASH_PAGE_SIZE % sizeof(LogRecord) == 0, "Registro deve caber na pagina");

static const u32 RECORD_SIZE = sizeof(LogRecord);
static const u32 LEGACY_RECORD_SIZE = sizeof(LogRecordV2);
static const u8 MPU_STATUS_VALID = 0x01;
static const u8 MPU_STATUS_GYRO_CALIBRATED = 0x02;

enum RecordFlag : u8 {
    FLAG_PRETRIGGER = 0x01,
    FLAG_LAUNCH     = 0x02,
    FLAG_RESUMED    = 0x04,
    FLAG_END_LANDED = 0x08,
    FLAG_END_MANUAL = 0x10,
    FLAG_END_TIMEOUT = 0x16,
    FLAG_END_FULL   = 0x20,
};
static const u8 FLAG_END_MASK = FLAG_END_LANDED | FLAG_END_MANUAL | FLAG_END_FULL;

// ============================================================
// ESTADOS
// ============================================================

typedef enum EstadoSistema_s : u8 {
    ESTADO_CALIBRACAO,      // Inicialização = 0
    ESTADO_PRONTO,          // Na Rampa (Modo Silencioso) = 1
    ESTADO_EM_VOO,          // Subindo (Gravando) = 2
	ESTADO_APOGEU,			// Subindo (Gravando) = 3
    ESTADO_RECUPERACAO,     // Caindo (Gravando) = 4
    ESTADO_POUSADO,         // No Chão (Finalizado) = 5
	ESTADO_ERRO				// Erro = 6
} EstadoSistema_t;

typedef struct DadosVoo_s {
    u32 uptimeMs;
    float    pressurePa;
    float    temperatureC;
    float    altitudeM;
    i16  accelRaw[3];
    i16  gyroRaw[3];
    u8  imuStatus;
    EstadoSistema estadoAtual;
} DadosVoo_t;




FlashStringPtr stateName(EstadoSistema_t state);
float relativeAltitude(float pressurePa);


// ============================================================
// SENSOR
// ============================================================


void resetFlightTracking(void);

bool calibrateBase(void);

// ============================================================
// CICLO DE VOO
// ============================================================

void preRingPush(const DadosVoo_t &dadosVoo);

void stopWithError(FlashStringPtr message);

void startFlight(u32 launchTime);
void handleIdleDadosVoo(const DadosVoo_t &dadosVoo);
void handleRecordingDadosVoo(const DadosVoo_t &dadosVoo);

void processarDadosVoo(u32 now);



// ============================================================
// INICIALIZAÇÃO
// ============================================================

void waitForSerial(u32 timeoutMs);

void resumeFlight(void);


#if defined(BUZZER_PIN) && defined(USE_BUZZER)
	// Se o seu buzzer liga com LOW (Logica Invertida / Active-Low)
	#pragma message("BUZZER_PIN foram definidos.")
	#define BUZZER_ON()  digitalWrite(BUZZER_PIN, LOW)
	#define BUZZER_OFF() digitalWrite(BUZZER_PIN, HIGH)
#else
	#warning "BUZZER_PIN não foram definidos!"
	#define BUZZER_ON()  do { } while(0)
	#define BUZZER_OFF() do { } while(0)
#endif