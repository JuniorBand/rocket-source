#ifndef TELEMETRIA_COM_H_
#define TELEMETRIA_COM_H_

#include <macros_config.h>
#include <utils/utils.h>

#define TELEM_SYNC_1            0xAA
#define TELEM_SYNC_2            0x55

// Comandos ASCII recebidos pela UART2 (Placa de Telemetria -> OBC)
#define TELEM_CMD_DADOS         'D'  // Solicita pacote completo (37 bytes)
#define TELEM_CMD_RESUMO        'R'  // Solicita pacote compacto de apogeu/eventos (22 bytes)
#define TELEM_CMD_PING          'P'  // Teste de enlace (responde ACK de 6 bytes)

// Identificadores dos pacotes enviados pelo OBC
typedef enum {
    MSG_TELEM_DADOS  = 0x01,
    MSG_TELEM_RESUMO = 0x02,
    MSG_TELEM_ACK    = 0x11
} TelemMsgId_t;

// Pacote completo de 37 bytes
typedef struct __attribute__((packed)) {
    u8    sync1;
    u8    sync2;
    u8    msgId;
    u8    payloadLen;
    u32   uptimeMs;
    u8    estado;
    float pressaoPa;
    float altitudeM;
    float velocidadeMs;
    i16   temperatura_cC;
    i16   accelRaw[3];
    i16   gyroRaw[3];
    u8    flags;
    u8    crc8;
} TelemetriaPacket_t;

// Pacote compacto de 22 bytes
typedef struct __attribute__((packed)) {
    u8    sync1;
    u8    sync2;
    u8    msgId;
    u8    payloadLen;
    u32   tempoVooMs;
    float altitudeAtualM;
    float altitudeMaximaM;
    float velocidadeMs;
    u8    estadoEFlags;
    u8    crc8;
} TelemResumoPacket_t;

#ifdef __cplusplus
static_assert(sizeof(TelemetriaPacket_t) == 37, "TelemetriaPacket_t deve ter exatos 37 bytes");
static_assert(sizeof(TelemResumoPacket_t) == 22, "TelemResumoPacket_t deve ter exatos 22 bytes");
#endif

void telemetriaInit(void);
void telemetriaEnviar(void);
void telemetriaEnviarPacoteDados(void);
void telemetriaEnviarPacoteResumo(void);
void telemetriaReceber(void);

#endif /* TELEMETRIA_COM_H_ */