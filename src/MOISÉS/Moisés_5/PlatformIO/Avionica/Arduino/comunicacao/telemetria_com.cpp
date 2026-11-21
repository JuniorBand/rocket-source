/*
  ******************************************************************************
  * @file    telemetria_com.cpp
  * @date 	 20 de nov. de 2026
  * @author  Junior Bandeira
  * @brief   Gerenciador de Comunicacao de Telemetria para PlatformIO 
  * com Arduino (Estacao Solo)
  ******************************************************************************
*/

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
// UART2 - PROTOCOLO BINARIO v1 (teste USB-TTL, pronto para RP2040)
// ============================================================
// A5 5A VER TIPO SEQ LEN [LEN bytes] CRC16_LO CRC16_HI
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), sobre VER..PAYLOAD.
// TELEMETRIA 0x01, TELECOMANDO 0x02, ACK 0x03, NACK 0x04,
// STATUS 0x05, LOG_RECORD 0x06, LOG_INFO 0x07, LOG_END 0x08.
// Inteiros little-endian. Nao misturar prints ASCII nesta UART.
// Testes de comando: PING=0x10, V=0x01, S=0x02, A=0x03.
// REMOTO: S encerra gravacao em andamento ou desarma, igual ao comando USB.
// V somente quando PARADO com registros gravados; enviar S antes de V.
// A apaga a Flash e arma, exceto enquanto gravando.
// ATENCAO: S remoto e destinado aos ensaios em bancada, nao a voo.

// STM32duino 3.0.0: a classe concreta da UART chama-se Uart (RX, TX).
Uart SerialTTL(PA3, PA2);

static const u8 TTL_VER = 1;
static const u8 TTL_TELEM = 0x01;
static const u8 TTL_CMD = 0x02;
static const u8 TTL_ACK = 0x03;
static const u8 TTL_NACK = 0x04;
static const u8 TTL_STATUS = 0x05;
static const u8 TTL_LOGREC = 0x06;
static const u8 TTL_LOGINFO = 0x07;
static const u8 TTL_LOGEND = 0x08;
static const u8 TTL_PAYLOAD_MAX = 48;

u8 ttlSeq = 0;
u8 ttlRx[TTL_PAYLOAD_MAX + 8];
u8 ttlRxLen = 0;
bool ttlDumpInfoSent = false;
u32 ttlDumpIndex = 0;
u32 ttlDumpNextMs = 0;
u8 ttlDumpResult = 0;

void ttlPush16(u8 *p, u8 &at, i16 v) {
    const u16 bits = static_cast<u16>(v);
    p[at++] = static_cast<u8>(bits);
    p[at++] = static_cast<u8>(bits >> 8);
}
void ttlPush32(u8 *p, u8 &at, u32 v) {
    for (u8 n = 0; n < 4; ++n) { p[at++] = static_cast<u8>(v); v >>= 8; }
}
i16 ttlClampI16(float v) {
    if (!isfinite(v)) return 0;
    if (v > 32767.0f) return INT16_MAX;
    if (v < -32768.0f) return INT16_MIN;
    return static_cast<i16>(lroundf(v));
}
u16 ttlCRC16(const u8 *data, size_t len) {
    u16 crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<u16>(data[i]) << 8;
        for (u8 j = 0; j < 8; ++j) {
            crc = (crc & 0x8000u) ? static_cast<u16>((crc << 1) ^ 0x1021u)
                                  : static_cast<u16>(crc << 1);
        }
    }
    return crc;
}
bool ttlPacket(u8 type, u8 seq, const u8 *payload, u8 length) {
    if (length > TTL_PAYLOAD_MAX) return false;
    u8 out[TTL_PAYLOAD_MAX + 8];
    out[0] = 0xA5; out[1] = 0x5A;
    out[2] = TTL_VER; out[3] = type; out[4] = seq; out[5] = length;
    if (length > 0) memcpy(out + 6, payload, length);
    const u16 crc = ttlCRC16(out + 2, static_cast<size_t>(4 + length));
    out[6 + length] = static_cast<u8>(crc);
    out[7 + length] = static_cast<u8>(crc >> 8);
    const size_t frameLen = static_cast<size_t>(length + 8);
    // Nao bloquear a tarefa de aquisicao quando UART estiver sem espaco.
    if (SerialTTL.availableForWrite() < static_cast<int>(frameLen)) return false;
    return SerialTTL.write(out, frameLen) == frameLen;
}
void ttlReply(u8 type, u8 seq, u8 command, u8 status) {
    const u8 data[2] = {command, status};
    ttlPacket(type, seq, data, sizeof(data));
}
void ttlSendStatus(u8 seq) {
    // state, IMU status, numero de registros, uptime em ms.
    u8 p[10]; u8 i = 0;
    p[i++] = static_cast<u8>(currentState);
    p[i++] = hasLatest ? latest.imuStatus : 0;
    ttlPush32(p, i, recordCount);
    ttlPush32(p, i, millis());
    ttlPacket(TTL_STATUS, seq, p, i);
}
void ttlSendTelemetry() {
    if (!hasLatest) return;
    u8 p[29]; u8 i = 0;
    // TEMPO = millis() da amostra desde boot (nao tempo relativo ao lancamento).
    ttlPush32(p, i, latest.uptimeMs);
    ttlPush32(p, i, static_cast<u32>(lroundf(latest.pressurePa)));  // Pa
    ttlPush16(p, i, ttlClampI16(latest.temperatureC * 100.0f));        // 0.01 C
    ttlPush32(p, i, static_cast<u32>(static_cast<int32>(lroundf(latest.altitudeM * 100.0f)))); // cm, signed
    ttlPush16(p, i, ttlClampI16(verticalSpeed * 10.0f));              // 0.1 m/s
    for (u8 k = 0; k < 3; ++k) {
        // 1 mg por unidade; acelerometro +/-16 g.
        const float mg = (latest.imuStatus & MPU_STATUS_VALID)
            ? (latest.accelRaw[k] * 1000.0f / 2048.0f) : 0.0f;
        ttlPush16(p, i, ttlClampI16(mg));
    }
    for (u8 k = 0; k < 3; ++k) {
        // 0.1 grau/s por unidade (offset calibrado quando disponivel).
        const float dg = (latest.imuStatus & MPU_STATUS_VALID)
            ? (int32(latest.gyroRaw[k]) - ((latest.imuStatus & MPU_STATUS_GYRO_CALIBRATED) ? gyroBiasRaw[k] : 0))
               * MPU_GYR_TO_DPS * 10.0f : 0.0f;
        ttlPush16(p, i, ttlClampI16(dg));
    }
    // bits 0-1 estado (0 armado / 1 gravando / 2 parado), bit2 IMU valida,
    // bit3 giro calibrado, bit4 BMP valido. Os valores IMU zerados sem bit2 nao sao medidas reais.
    u8 state = static_cast<u8>(currentState) | 0x10;
    if (latest.imuStatus & MPU_STATUS_VALID) state |= 0x04;
    if (latest.imuStatus & MPU_STATUS_GYRO_CALIBRATED) state |= 0x08;
    p[i++] = state;
    ttlPacket(TTL_TELEM, ttlSeq++, p, i);  // 29 B payload / 37 B pacote
}
void ttlHandleCommand(u8 seq, const u8 *p, u8 len) {
    if (len != 1) {
        ttlReply(TTL_NACK, seq, len ? p[0] : 0, 1);  // 1=tamanho invalido
        return;
    }
    switch (p[0]) {
        case 0x10:  // PING / status instantaneo; nao altera nada.
            ttlSendStatus(seq);
            break;
        case 0x01:  // V: solicitar transferencia binaria dos registros do voo
            if (currentState != STATE_STOPPED || !caixaPretaValid || !recordCount || ttlDumpActive) {
                ttlReply(TTL_NACK, seq, p[0], 2);  // 2=indisponivel
            } else {
                ttlReply(TTL_ACK, seq, p[0], 0);
                ttlDumpIndex = 0;
                ttlDumpResult = 0;
                ttlDumpInfoSent = false;
                ttlDumpNextMs = millis();
                ttlDumpActive = true;
            }
            break;
        case 0x02:  // S: igual ao comando local - para a gravacao ou desarma
            if (ttlDumpActive) {
                ttlReply(TTL_NACK, seq, p[0], 2);
            } else {
                executeS();  // ARMADO -> PARADO; GRAVANDO -> finaliza e salva
                if (currentState == STATE_STOPPED) {
                    ttlReply(TTL_ACK, seq, p[0], 0);
                } else {
                    ttlReply(TTL_NACK, seq, p[0], 2);
                }
            }
            break;
        case 0x03:  // A: apagar Flash e armar, sem confirmacao extra (teste)
            Serial.println(F("[UART] Telecomando A recebido (0x03)."));
            if (currentState == STATE_RECORDING) {
                // Conserva o comportamento do comando A local durante gravacao.
                ttlReply(TTL_NACK, seq, p[0], 2);
            } else {
                executeA();
                // ACK somente depois de confirmar que o apagamento e a calibracao deram certo.
                if (currentState == STATE_IDLE && recordCount == 0 &&
                    !caixaPretaValid && !unknownFlashData) {
                    ttlReply(TTL_ACK, seq, p[0], 0);
                } else {
                    ttlReply(TTL_NACK, seq, p[0], 2);
                }
            }
            break;
        default:
            ttlReply(TTL_NACK, seq, p[0], 4);  // 4=comando desconhecido
    }
}
void ttlRxByte(u8 c) {
    if (ttlRxLen == 0) {
        if (c == 0xA5) ttlRx[ttlRxLen++] = c;
        return;
    }
    if (ttlRxLen == 1) {
        if (c == 0x5A) ttlRx[ttlRxLen++] = c;
        else if (c != 0xA5) ttlRxLen = 0;
        return;
    }
    if (ttlRxLen >= sizeof(ttlRx)) { ttlRxLen = 0; return; }
    ttlRx[ttlRxLen++] = c;
    if (ttlRxLen < 6) return;
    if (ttlRx[5] > TTL_PAYLOAD_MAX) { ttlRxLen = 0; return; }
    const u8 full = static_cast<u8>(ttlRx[5] + 8);
    if (ttlRxLen != full) return;
    const u16 expected = ttlCRC16(ttlRx + 2, ttlRx[5] + 4);
    const u16 found = static_cast<u16>(ttlRx[full - 2]) |
        (static_cast<u16>(ttlRx[full - 1]) << 8);
    if (expected == found && ttlRx[2] == TTL_VER && ttlRx[3] == TTL_CMD)
        ttlHandleCommand(ttlRx[4], ttlRx + 6, ttlRx[5]);
    ttlRxLen = 0;
}
void ttlReceiveCommands() {
    // Limite de bytes por iteracao, mantendo a prioridade do sensoriamento.
    u8 n = 0;
    while (SerialTTL.available() > 0 && n++ < 64) {
        const int value = SerialTTL.read();
        if (value >= 0) ttlRxByte(static_cast<u8>(value));
    }
}
void ttlStreamLog() {
    if (!ttlDumpActive) return;
    const u32 now = millis();
    if (static_cast<int32>(now - ttlDumpNextMs) < 0) return;
    ttlDumpNextMs = now + 15; // envia ate ~66 registros/segundo

    if (!ttlDumpInfoSent) {
        // Count (4), versao da Flash (1), tamanho registro (1), cabecalho original (32)
        u8 body[38]; u8 k = 0;
        ttlPush32(body, k, recordCount);
        body[k++] = static_cast<u8>(caixaPreta.version);
        body[k++] = static_cast<u8>(activeRecordSize);
        memcpy(body + k, &caixaPreta, sizeof(caixaPreta));
        if (ttlPacket(TTL_LOGINFO, ttlSeq, body, sizeof(body))) {
            ++ttlSeq;
            ttlDumpInfoSent = true;
        }
        return;
    }
    if (ttlDumpIndex < recordCount && ttlDumpResult == 0) {
        u8 body[4 + sizeof(LogRecord)]; u8 k = 0;
        ttlPush32(body, k, ttlDumpIndex);
        bool ok = false;
        if (caixaPreta.version == LEGACY_VERSION) {
            LogRecordV2 r;
            ok = readLegacyRecord(ttlDumpIndex, r);
            if (ok) memcpy(body + k, &r, sizeof(r));
        } else {
            LogRecord r;
            ok = readRecord(ttlDumpIndex, r);
            if (ok) memcpy(body + k, &r, sizeof(r));
        }
        if (!ok) { ttlDumpResult = 1; return; }
        if (ttlPacket(TTL_LOGREC, ttlSeq, body,
                      static_cast<u8>(4 + activeRecordSize))) {
            ++ttlSeq;
            ++ttlDumpIndex;
        }
        return;
    }
    u8 body[5]; u8 k = 0;
    ttlPush32(body, k, ttlDumpIndex); body[k++] = ttlDumpResult;
    if (ttlPacket(TTL_LOGEND, ttlSeq, body, sizeof(body))) {
        ++ttlSeq;
        ttlDumpActive = false;
    }
}