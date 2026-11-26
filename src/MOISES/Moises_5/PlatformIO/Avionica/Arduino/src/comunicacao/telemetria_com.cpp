/*
  ******************************************************************************
  * @file    telemetria_com.cpp
  * @brief   Implementacao de Envio e Recebimento de Comandos para a Telemetria
  ******************************************************************************
*/

#include <Arduino.h>
#include <comunicacao/telemetria_com.h>
#include <config_voo.h>
#include <flash/w25q.h>
#include <math.h>
#include <string.h>

static u32 timerTelemMs = 0;
// HardwareSerial Serial2(PIN_UART_TTL_RX, PIN_UART_TTL_TX);

void telemetriaInit(void) {
    // Serial2 instancia automaticamente com a flag -D ENABLE_HWSERIAL2 no .ini
    // Caso contrario: instancie HardwareSerial Serial2(PIN_UART_TTL_RX, PIN_UART_TTL_TX); e comente a flag
    Serial2.setRx(PIN_UART_TTL_RX);
    Serial2.setTx(PIN_UART_TTL_TX);
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
    pkt.tempoVooMs      = (seguroVoo.tempoInicioMs > 0UL)
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

void telemetriaEnviarSeletiva(u8 mask) {
    // 4 bytes de cabeçalho + máx. 27 bytes de payload + 1 byte CRC8 = 32 bytes
    u8 txBuf[32];
    u8 pos = 4; // Reserva índices 0..3 para o cabeçalho

    if (mask & TELEM_CAMPO_ALTITUDE) {
        memcpy(&txBuf[pos], &dadosVoo.altitudeM, sizeof(float));
        pos += sizeof(float);
    }
    if (mask & TELEM_CAMPO_VELOCIDADE) {
        memcpy(&txBuf[pos], &dadosVoo.velocidadeMs, sizeof(float));
        pos += sizeof(float);
    }
    if (mask & TELEM_CAMPO_PRESSAO) {
        memcpy(&txBuf[pos], &dadosVoo.pressaoPa, sizeof(float));
        pos += sizeof(float);
    }
    if (mask & TELEM_CAMPO_TEMP) {
        const i16 temp_cC = (i16)(bmpSensor.temperaturaC * 100.0f);
        memcpy(&txBuf[pos], &temp_cC, sizeof(i16));
        pos += sizeof(i16);
    }
    if (mask & TELEM_CAMPO_ESTADO) {
        u8 stFlags = ((u8)dadosVoo.estadoAtual & 0x0F);
        if (dadosVoo.imuStatus & MPU_STATUS_VALID) { stFlags |= 0x10; }
        if (seguroVoo.deployAcionado)              { stFlags |= 0x20; }
        if (seguroVoo.mainAcionado)                { stFlags |= 0x40; }
        if (w25qIsFull())                          { stFlags |= 0x80; }
        txBuf[pos++] = stFlags;
    }
    if (mask & TELEM_CAMPO_ACCEL) {
        memcpy(&txBuf[pos], mpuSensor.accelRaw, sizeof(i16) * 3);
        pos += sizeof(i16) * 3;
    }
    if (mask & TELEM_CAMPO_GYRO) {
        memcpy(&txBuf[pos], mpuSensor.gyroRaw, sizeof(i16) * 3);
        pos += sizeof(i16) * 3;
    }

    const u8 payloadLen = (u8)(pos - 4);
    txBuf[0] = 0xAA;
    txBuf[1] = MSG_TELEM_SELETIVA;
    txBuf[2] = mask;
    txBuf[3] = payloadLen;
    txBuf[pos] = crc8(txBuf, pos);

    Serial2.write(txBuf, pos + 1);
}

#ifdef RECEBER_COMANDOS_RADIO_TTL
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