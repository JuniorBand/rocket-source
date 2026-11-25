/*
  ******************************************************************************
  * @file    w25q.h
  * @brief   Driver de armazenamento SPI para a memória Flash W25Q (Caixa Preta)
  ******************************************************************************
*/

#ifndef W25Q_H
#define W25Q_H

#include <macros_config.h>
#include <utils/utils.h>

// ==============================================================================
// LAYOUT DE ENDEREÇOS DA W25Q (8 MB = 0x800000)
// ==============================================================================
constexpr u32 W25Q_HEADER_ADDR      = 0x000000; // Bytes 0..31: FlightHeader_t
constexpr u32 W25Q_STATUS_VOA_ADDR  = 0x000020; // Bytes 32..35: Flag de voo ativo (Caixa Preta)
constexpr u32 W25Q_DATA_START_ADDR  = 0x001000; // Início do 2º setor (4 KB) para os logs

// Valores Padroes da W25Q (datasheet)
constexpr u32 W25Q_PAGE_SIZE        = 256;
constexpr u32 W25Q_SECTOR_SIZE      = 4096;
constexpr u32 W25Q_BLOCK_SIZE       = 65536;
constexpr u32 W25Q_DEFAULT_LIMIT    = 0x800000; // 8 MB (W25Q)

constexpr u32 HEADER_MAGIC_FLT1     = 0x31544C46; // "FLT1"
constexpr u16 HEADER_FORMAT_VERSION = 3;
constexpr u32 MAGIC_NUMBER_SIRIUS   = 0xAA55AA55; // Indica voo em andamento (recuperação pós-watchdog)

// Flags compactadas no nibble superior de LogData_t::stateAndFlags
constexpr u8 LOG_FLAG_PRETRIGGER    = 0x10; // 0b10000 bit 5 acionado quando timestamp < 0 (pré-trigger)
constexpr u8 LOG_FLAG_LAUNCH        = 0x20; // 0b100000 bit 6 acionado no registro do lançamento (timestamp == 0)
constexpr u8 LOG_FLAG_RESUMED       = 0x40; // 0b1000000 bit 7 acionado quando o log é retomado após reinicio em voo (watchdog)
constexpr u8 LOG_FLAG_IMU_VALID     = 0x80; // 0b10000000 bit 8 acionado quando o registro contém dados válidos do MPU6050 (acel + gyro)

// ==============================================================================
// ESTRUTURAS DE DADOS (ALINHADAS EM 32 BYTES)
// ==============================================================================

typedef struct __attribute__((packed)) {
    u32   magic;            // 4 B: 0x31544C46 ("FLT1")
    u16   version;          // 2 B: Versão 3
    u16   recordSize;       // 2 B: 32 bytes
    u16   sampleIntervalMs; // 2 B: 50 ms (20 Hz)
    u8    mpuCalibrated;    // 1 B: 1 = giroscópio calibrado
    u8    reserved0;        // 1 B: alinhamento
    float pressaoRefPa;     // 4 B: Pressão base calibrada no solo
    float temperaturaRefC;  // 4 B: Temperatura base calibrada no solo
    i16   gyroBiasRaw[3];   // 6 B: Offset estático do giroscópio
    u8    reserved1[5];     // 5 B: padding para fechar 32 bytes
    u8    crc8;             // 1 B: Checksum dos 31 bytes anteriores
} FlightHeader_t;

typedef struct __attribute__((packed)) {
    i32   timestampMs;      // 4 B: Tempo relativo ao lançamento (negativo = pré-trigger)
    float pressaoPa;        // 4 B: Pressão barométrica (Pa)
    float altitudeM;        // 4 B: Altitude filtrada pelo Kalman (m)
    float velocidadeMs;     // 4 B: Velocidade vertical filtrada pelo Kalman (m/s)
    i16   temperatura_cC;   // 2 B: Temperatura em centésimos de °C (ex: 2540 = 25.40 °C)
    i16   accelRaw[3];      // 6 B: Acelerômetro X, Y, Z (2048 LSB/g)
    i16   gyroRaw[3];       // 6 B: Giroscópio X, Y, Z (16.4 LSB/dps)
    u8    stateAndFlags;    // 1 B: Bits 0..3 = EstadoSistema | Bits 4..7 = LOG_FLAG_*
    u8    crc8;             // 1 B: Checksum dos 31 bytes anteriores
} LogData_t;

static_assert(sizeof(FlightHeader_t) == 32, "FlightHeader_t deve ter exatos 32 bytes");
static_assert(sizeof(LogData_t) == 32, "LogData_t deve ter exatos 32 bytes");

constexpr u32 LOGS_PER_PAGE = W25Q_PAGE_SIZE / sizeof(LogData_t); // 8 registros por página

typedef struct {
    FlightHeader_t header;
    u32            statusVoo;
    bool           headerValido;
    bool           dadosDesconhecidos;
} CaixaPretaInfo_t;

// ==============================================================================
// API PÚBLICA DO DRIVER W25Q
// ==============================================================================

bool w25qInit(void);
u32  w25qGetCapacity(void);
u32  w25qGetRecordCount(void);
u32  w25qGetMaxRecords(void);
bool w25qIsFull(void);

// Cabeçalho e Caixa-Preta (Setor 0)
bool w25qGravarHeader(float pressaoRefPa, float tempRefC, const i16 gyroBias[3], bool gyroCal);
void w25qSetStatusVoo(u32 status);
CaixaPretaInfo_t w25qLerCaixaPreta(void);

// Gravação Paginada de Logs (Setor 1+)
bool w25qAdicionarLog(const LogData_t *registro);
bool w25qDescarregarBuffer(void);
bool w25qLerRegistro(u32 index, LogData_t *dest);

// Limpeza e Visualização
bool w25qApagarLogs(void);
bool w25qApagarTudo(void);
void w25qVisualizarLogsTabela(void);
void w25qVisualizarLogsCSV(void);
void w25qVisualizarUltimoLog(void);
void w25qGerarVooSimulado(void);

#endif /* W25Q_H */