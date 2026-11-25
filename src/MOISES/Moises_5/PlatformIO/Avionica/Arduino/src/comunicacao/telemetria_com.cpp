#include <comunicacao/telemetria_com.h>
#include <config_voo.h>
#include <flash/w25q.h>
#include <math.h>
#include <string.h>

#ifdef MANDAR_DADOS_TELEMETRIA

HardwareSerial Serial2(PIN_UART_TTL_RX, PIN_UART_TTL_TX);
static u32 timerTelemMs = 0;

void telemetriaInit(void) {
    Serial2.begin(UART_TTL_BAUD);
    printflnLGreen("[UART2] Telemetria inicializada (%lu bps).", (unsigned long)UART_TTL_BAUD);
}

void telemetriaEnviarPacoteDados(void) {
    if (Serial2.availableForWrite() < (int)sizeof(TelemetriaPacket_t)) {
        return; // Não bloqueia se o buffer TX da UART2 estiver cheio
    }

    TelemetriaPacket_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.sync1          = TELEM_SYNC_1;
    pkt.sync2          = TELEM_SYNC_2;
    pkt.msgId          = MSG_TELEM_DADOS;
    pkt.payloadLen     = sizeof(TelemetriaPacket_t) - 5; // 32 bytes
    pkt.uptimeMs       = dadosVoo.uptimeMs;
    pkt.estado         = (u8)dadosVoo.estadoAtual;
    pkt.pressaoPa      = dadosVoo.pressaoPa;
    pkt.altitudeM      = dadosVoo.altitudeM;
    pkt.velocidadeMs   = dadosVoo.velocidadeMs;
    pkt.temperatura_cC = (i16)lroundf(dadosVoo.temperaturaC * 100.0f);

    for (u8 a = 0; a < 3; ++a) {
        pkt.accelRaw[a] = dadosVoo.accelRaw[a];
        pkt.gyroRaw[a]  = dadosVoo.gyroRaw[a];
    }

    u8 flags = 0;
    if (dadosVoo.imuStatus & MPU_STATUS_VALID) flags |= (1U << 0);
    if (seguroVoo.deployAcionado)              flags |= (1U << 1);
    if (seguroVoo.mainAcionado)                flags |= (1U << 2);
    if (w25qIsFull())                          flags |= (1U << 3);
    pkt.flags = flags;

    pkt.crc8 = crc8((const u8 *)&pkt, sizeof(TelemetriaPacket_t) - 1);
    Serial2.write((const u8 *)&pkt, sizeof(TelemetriaPacket_t));
}

void telemetriaEnviarPacoteResumo(void) {
    if (Serial2.availableForWrite() < (int)sizeof(TelemResumoPacket_t)) {
        return;
    }

    TelemResumoPacket_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.sync1           = TELEM_SYNC_1;
    pkt.sync2           = TELEM_SYNC_2;
    pkt.msgId           = MSG_TELEM_RESUMO;
    pkt.payloadLen      = sizeof(TelemResumoPacket_t) - 5; // 17 bytes
    pkt.tempoVooMs      = (dadosVoo.estadoAtual >= ESTADO_EM_VOO)
                            ? (millis() - seguroVoo.tempoInicioMs)
                            : 0UL;
    pkt.altitudeAtualM  = dadosVoo.altitudeM;
    pkt.altitudeMaximaM = seguroVoo.altitudeMaxima;
    pkt.velocidadeMs    = dadosVoo.velocidadeMs;

    u8 stFlags = ((u8)dadosVoo.estadoAtual & 0x0F);
    if (seguroVoo.deployAcionado) stFlags |= (1U << 4);
    if (seguroVoo.mainAcionado)   stFlags |= (1U << 5);
    if (w25qIsFull())             stFlags |= (1U << 6);
    pkt.estadoEFlags = stFlags;

    pkt.crc8 = crc8((const u8 *)&pkt, sizeof(TelemResumoPacket_t) - 1);
    Serial2.write((const u8 *)&pkt, sizeof(TelemResumoPacket_t));
}

static void telemetriaEnviarPingAck(void) {
    u8 ackPkt[6];
    if (Serial2.availableForWrite() < (int)sizeof(ackPkt)) return;

    ackPkt[0] = TELEM_SYNC_1;
    ackPkt[1] = TELEM_SYNC_2;
    ackPkt[2] = MSG_TELEM_ACK;
    ackPkt[3] = 1;
    ackPkt[4] = (u8)dadosVoo.estadoAtual;
    ackPkt[5] = crc8(ackPkt, 5);

    Serial2.write(ackPkt, sizeof(ackPkt));
}

void telemetriaEnviar(void) {
    const u32 now = millis();
    if ((now - timerTelemMs) >= USB_TELEM_INTERVAL_MS) {
        timerTelemMs = now;
        telemetriaEnviarPacoteDados();
    }
}

    #ifdef RECEBER_DADOS_TELEMETRIA
    void telemetriaReceber(void) {
        while (Serial2.available() > 0) {
            const char cmd = (char)Serial2.read();

            if (cmd == '\r' || cmd == '\n' || cmd == ' ' || cmd == '\t') {
                continue;
            }

            switch (cmd) {
                case TELEM_CMD_DADOS:
                case 'd':
                    telemetriaEnviarPacoteDados();
                    break;

                case TELEM_CMD_RESUMO:
                case 'r':
                    telemetriaEnviarPacoteResumo();
                    break;

                case TELEM_CMD_PING:
                case 'p':
                    telemetriaEnviarPingAck();
                    break;

                default:
                    break;
            }
        }
    }
    #endif /* ENVIAR_DADOS_TELEMETRIA */

#else

void telemetriaInit(void) {}
void telemetriaEnviar(void) {}
void telemetriaEnviarPacoteDados(void) {}
void telemetriaEnviarPacoteResumo(void) {}
void telemetriaReceber(void) {}

#endif /* MANDAR_DADOS_TELEMETRIA */