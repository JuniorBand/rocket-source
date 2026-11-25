/*
  ******************************************************************************
  * @file    macros_config.h
  * @brief   PAINEL DE CONTROLE DE HARDWARE E MISSÃO (STM32F411 + ARDUINO)
  ******************************************************************************
*/

#ifndef MACROS_CONFIG_H_
#define MACROS_CONFIG_H_

// ==============================================================================
// 1. CHAVES DE COMPILAÇÃO DA MISSÃO
// ==============================================================================
//#define EM_VOO            // Descomente para voo real; comente para modo bancada
#define USE_W25Q            // Usa memória Flash SPI externa W25Q64
#define USE_FILTER          // Ativa o Filtro de Kalman (altitude + velocidade)
#define USE_BUZZER          // Ativa feedback sonoro no buzzer
#define VERBOSE             // Imprime telemetria formatada na USB (5 Hz)
#define GRAVAR_LOGS         // Habilita gravação na Flash
#define BMP_USE_HW_SPI 1    // 1 = SPI3 em hardware (PB3/PB4/PB5); 0 = software

// Permite enviar dados de telemetria via UART2 (GPS/Rádio) para estação remota
//#define MANDAR_DADOS_TELEMETRIA

// Permite receber comandos de telemetria via UART2 (GPS/Rádio) da estação remota
//#define RECEBER_DADOS_TELEMETRIA

// Permite colocar cores ou tirar as cores dos prints (Caso nao queira cores: comente)
#define PRINT_COLORS

// Permite usar prints no código (Caso nao queira prints: comente)
#define USE_PRINTS


// // Comente esta linha quando compilar no STM32CubeIDE (HAL)
// #define USE_ARDUINO_UTILS

// #ifdef USE_ARDUINO_UTILS
//     #include <Arduino.h>
// #endif

// ==============================================================================
// 2. PINAGEM DA PCB (STM32F411 BLACK PILL)
// ==============================================================================
// Sinalização
#define PIN_BUZZER              PB1
#define PIN_LED_ONBOARD         PC13 // Ativo em LOW

// Memória Flash W25Q64 (SPI1: PA5 SCK, PA6 MISO, PA7 MOSI)
#define PIN_FLASH_CS            PA4

// Barômetro BMP280 (SPI3 via funções alternativas ALT1)
#define PIN_BMP_CS              PA15
#define PIN_BMP_SCK             PB3
#define PIN_BMP_MISO            PB4
#define PIN_BMP_MOSI            PB5

// IMU MPU6050 (I2C por software open-drain nas trilhas PB14/PB15)
#define PIN_MPU_SCL             PB14
#define PIN_MPU_SDA             PB15
#define PIN_MPU_AD0             PB2  // Mantido LOW (também aterrar ADD na PCB)
#define PIN_MPU_AUX_XDA         PB13 // Roteado ao XDA: manter em INPUT
#define MPU_I2C_ADDR            0x68

// Canais Pirotécnicos / Ejeção (Optoacopladores -> IRLZ44N)
#define PIN_PYRO_MAIN           PB0 // Canal J1 (Paraquedas Principal)
#define PIN_PYRO_DEPLOY         PA0  // Canal J2 (Drogue / Apogeu)

// Telemetria / Placa GPS + Rádio (UART2 no conector OBC)
#define PIN_UART_TTL_RX         PA3
#define PIN_UART_TTL_TX         PA2
#define PIN_OBC_AUX             PA1  // Pino auxiliar no conector OBC

// ==============================================================================
// 3. TAXAS, TEMPOS E CONSTANTES DOS SENSORES
// ==============================================================================
#define SAMPLE_INTERVAL_MS      50UL      // 20 Hz de amostragem
#define DT                      0.05f     // Passo de tempo do Kalman (50 ms)
#define USB_TELEM_INTERVAL_MS   200UL     // 5 Hz de telemetria na USB
#define UART_TTL_BAUD           115200UL  // Baudrate da UART2 (GPS/Rádio)
#define WATCHDOG_TIMEOUT_US     4000000UL // 4 segundos
#define USB_WAIT_MS             1500UL    // Espera inicial pelo monitor USB

// Pré-trigger na RAM (20 amostras = 1 segundo antes da decolagem a 20 Hz)
#define PRETRIGGER_SAMPLES      20

// Limites físicos e calibração do BMP280
#define PRESSURE_MIN_PA         30000.0f
#define PRESSURE_MAX_PA         120000.0f
#define BMP_CALIB_SAMPLES       32
#define BMP_CALIB_MIN_VALID     24

// Limites e fatores de escala do MPU6050 (+/-16g e +/-2000 dps)
#define MPU_CAL_SAMPLES         96
#define MPU_FAIL_LIMIT          5
#define MPU_RETRY_MS            3000UL
#define MPU_ACC_TO_MS2          (9.80665f / 2048.0f)
#define MPU_GYR_TO_DPS          (1.0f / 16.4f)

// Sintonização do Filtro de Kalman (ajustado para ruído do BMP280)
#define LKF_Q_ALT               0.01f
#define LKF_Q_VEL               0.15f
#define LKF_R_ALT               0.65f

#endif /* MACROS_CONFIG_H_ */