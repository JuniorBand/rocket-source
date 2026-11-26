/*
  ******************************************************************************
  * @file    w25q.cpp
  * @brief   Implementação do driver SPI Flash W25Q com Buffer Paginado e Smart Erase
  ******************************************************************************
*/

#include <flash/w25q.h>
#include <config_voo.h>
#include <SPI.h>
#include <SPIFlash.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static SPIFlash flash(PIN_FLASH_CS, &SPI);

static u32 flashCapacity = 0;
static u32 maxRecords = 0;
static u32 currentAddr = W25Q_DATA_START_ADDR;
static u32 recordCount = 0;

// Buffer de página na RAM (256 bytes = 8 registros de 32 bytes)
static LogData_t bufferRAM[LOGS_PER_PAGE];
static u8 bufferIndex = 0;

static inline bool printsBloqueadosPorVoo(void) {
#ifdef EM_VOO
    return (dadosVoo.estadoAtual >= ESTADO_EM_VOO) &&
           (dadosVoo.estadoAtual <= ESTADO_RECUPERACAO);
#else
    return false;
#endif
}

// ==============================================================================
// FUNÇÕES INTERNAS DE BUSCA E PAGINAÇÃO
// ==============================================================================

static inline bool isErased(const u8 *bytes, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (bytes[i] != 0xFF) return false;
    }
    return true;
}

static bool slotEstaApagado(u32 addr, bool &apagado) {
    u8 raw[sizeof(LogData_t)];

    if (!flash.readByteArray(addr, raw, sizeof(raw))) {
        return false;
    }

    apagado = isErased(raw, sizeof(raw));
    return true;
}

// Busca binária O(log N) para localizar onde os registros terminam após boot/reset
static bool buscarUltimoSlotGravado(u32 totalSlots, u32 &slotsUsados) {
    u32 low = 0;
    u32 high = totalSlots;

    while (low < high) {
        const u32 mid = (u32)(low + (high - low) / 2);
        bool apagado = false;
        if (!slotEstaApagado(W25Q_DATA_START_ADDR + mid * sizeof(LogData_t), apagado)) {
            return false;
        }
        if (apagado) { high = mid; }
        else         { low  = (u32)(mid + 1); }

        feedWatchdog();
    }
    slotsUsados = low;
    return true;
}

static bool w25qFlushPage(void) {
    if (bufferIndex == 0) { return true; }

    const u32 bytesToWrite = bufferIndex * sizeof(LogData_t);
    if ((currentAddr + bytesToWrite) > flashCapacity) {
        bufferIndex = 0;
        return false;
    }

    if (!flash.writeByteArray(currentAddr, (uint8_t *)bufferRAM, bytesToWrite)) {
        return false;
    }

    currentAddr += bytesToWrite;
    recordCount += bufferIndex;
    bufferIndex = 0;
    return true;
}

// ==============================================================================
// INICIALIZAÇÃO E STATUS
// ==============================================================================

bool w25qInit(void) {
    pinMode(PIN_FLASH_CS, OUTPUT);
    digitalWrite(PIN_FLASH_CS, HIGH);
    pinMode(PA6, INPUT_PULLUP); // Evita MISO flutuando sem chip na bancada

    SPI.begin();

    // Sondagem rápida JEDEC ID (0x9F) antes do flash.begin() para evitar loop infinito
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_FLASH_CS, LOW);
    SPI.transfer(0x9F);
    const u32 jedecProbe = ((u32)SPI.transfer(0x00) << 16) |
                           ((u32)SPI.transfer(0x00) << 8)  |
                           (u32)SPI.transfer(0x00);
    digitalWrite(PIN_FLASH_CS, HIGH);
    SPI.endTransaction();

    if (jedecProbe == 0x000000UL || jedecProbe == 0xFFFFFFUL) {
        printflnRed("[ERRO CRITICO] Memoria SPI Flash W25Q nao encontrada (JEDEC: 0x%06lX)!",
                    (unsigned long)jedecProbe);
        return false;
    }

    if (!flash.begin()) {
        printlnRed("[ERRO CRITICO] Falha ao inicializar driver SPIMemory!");
        return false;
    }

    flashCapacity = flash.getCapacity();
    if (flashCapacity <= W25Q_DATA_START_ADDR + sizeof(LogData_t)) {
        printlnRed("[ERRO CRITICO] Capacidade da Flash invalida.");
        return false;
    }

    maxRecords = (flashCapacity - W25Q_DATA_START_ADDR) / sizeof(LogData_t);

    u32 slots = 0;
    if (buscarUltimoSlotGravado(maxRecords, slots)) {
        recordCount = slots;
        currentAddr = W25Q_DATA_START_ADDR + recordCount * sizeof(LogData_t);
    } else {
        recordCount = 0;
        currentAddr = W25Q_DATA_START_ADDR;
    }
    bufferIndex = 0;

    printflnLGreen("[OK] W25Q Online | JEDEC: 0x%06lX | Cap: %lu bytes | Registros: %lu",
                   (ulong)flash.getJEDECID(),
                   (ulong)flashCapacity,
                   (ulong)recordCount);
    return true;
}

u32  w25qGetCapacity(void)    { return flashCapacity; }
u32  w25qGetRecordCount(void) { return recordCount + bufferIndex; }
u32  w25qGetMaxRecords(void)  { return maxRecords; }
bool w25qIsFull(void)         { return (recordCount + bufferIndex) >= maxRecords; }

// ==============================================================================
// CABEÇALHO E CAIXA-PRETA (SETOR 0)
// ==============================================================================

bool w25qGravarHeader(float pressaoRefPa, float tempRefC, const i16 gyroBias[3], bool gyroCal) {
    FlightHeader_t hdr;
    memset(&hdr, 0, sizeof(hdr));

    hdr.magic = HEADER_MAGIC_FLT1;
    hdr.version = HEADER_FORMAT_VERSION;
    hdr.recordSize = sizeof(LogData_t);
    hdr.sampleIntervalMs = SAMPLE_INTERVAL_MS;
    hdr.mpuCalibrated = gyroCal ? 1 : 0;
    hdr.pressaoRefPa = pressaoRefPa;
    hdr.temperaturaRefC = tempRefC;

    if (gyroBias) {
        for (u8 i = 0; i < 3; ++i) { hdr.gyroBiasRaw[i] = gyroBias[i]; }
    }

    hdr.crc8 = crc8((const u8 *)&hdr, sizeof(hdr) - 1);

    return flash.writeByteArray(W25Q_HEADER_ADDR, (uint8_t *)&hdr, sizeof(hdr));
}

void w25qSetStatusVoo(u32 status) {
    // Grava diretamente em 0x000020 (de 0xFFFFFFFF -> 0xAA55AA55 -> 0x00000000 sem apagar setor)
    flash.writeByteArray(W25Q_STATUS_VOA_ADDR, (uint8_t *)&status, sizeof(status));
}

CaixaPretaInfo_t w25qLerCaixaPreta(void) {
    CaixaPretaInfo_t info;
    memset(&info, 0, sizeof(info));
    info.header.pressaoRefPa    = 101325.0f;
    info.header.temperaturaRefC = 25.0f;

    u8 rawHeader[sizeof(FlightHeader_t)];
    if (!flash.readByteArray(W25Q_HEADER_ADDR, rawHeader, sizeof(rawHeader))) {
        info.dadosDesconhecidos = true;
        return info;
    }

    FlightHeader_t candidate;
    memcpy(&candidate, rawHeader, sizeof(candidate));

    const bool crcOk = (candidate.crc8 == crc8(rawHeader, sizeof(candidate) - 1));
    if (candidate.magic == HEADER_MAGIC_FLT1 &&
        candidate.version == HEADER_FORMAT_VERSION &&
        candidate.recordSize == sizeof(LogData_t) &&
        crcOk) {
        info.header = candidate;
        info.headerValido = true;
    } else if (!isErased(rawHeader, sizeof(rawHeader)) || recordCount > 0) {
        info.dadosDesconhecidos = true;
    }

    u32 statusRaw = 0;
    if (flash.readByteArray(W25Q_STATUS_VOA_ADDR, (uint8_t *)&statusRaw, sizeof(statusRaw))) {
        info.statusVoo = statusRaw;
    }

    return info;
}

// ==============================================================================
// GRAVAÇÃO E LEITURA DE REGISTROS
// ==============================================================================

bool w25qAdicionarLog(const LogData_t *registro) {
    if (!registro || w25qIsFull()) { return false; }

    LogData_t item = *registro;
    item.crc8 = crc8((const u8 *)&item, sizeof(item) - 1);

    bufferRAM[bufferIndex++] = item;

    if (bufferIndex >= LOGS_PER_PAGE) {
        return w25qFlushPage();
    }
    return true;
}

bool w25qDescarregarBuffer(void) {
    return w25qFlushPage();
}

bool w25qLerRegistro(u32 index, LogData_t *dest) {
    if (!dest) { return false; }

    // Se o índice solicitado ainda estiver no buffer RAM não descarregado
    if (index >= recordCount && index < (recordCount + bufferIndex)) {
        *dest = bufferRAM[index - recordCount];
        return true;
    }

    if (index >= recordCount) { return false; }

    const u32 addr = W25Q_DATA_START_ADDR + index * sizeof(LogData_t);
    return flash.readByteArray(addr, (uint8_t *)dest, sizeof(LogData_t));
}

// ==============================================================================
// LIMPEZA INTELIGENTE (SMART ERASE) E FORMATAÇÃO
// ==============================================================================

bool w25qApagarLogs(void) {
    w25qDescarregarBuffer();

    // Apaga o Setor 0 (cabeçalho) + todos os blocos de 64 KB que contêm dados
    const u32 usadoBytes = (currentAddr > W25Q_DATA_START_ADDR) ? currentAddr : W25Q_DATA_START_ADDR;
    const u32 numBlocos  = (usadoBytes + W25Q_BLOCK_SIZE - 1) / W25Q_BLOCK_SIZE;

    printflnLYellow("\r\n[SISTEMA] Smart Erase: apagando %lu bloco(s) de 64 KB...", (unsigned long)numBlocos);

    for (u32 b = 0; b < numBlocos; ++b) {
        const u32 blockAddr = b * W25Q_BLOCK_SIZE;
        if (!flash.eraseBlock64K(blockAddr)) {
            printflnRed("[ERRO] Falha ao apagar bloco 0x%06lX!", (unsigned long)blockAddr);
            return false;
        }
        feedWatchdog();
    }

    currentAddr = W25Q_DATA_START_ADDR;
    recordCount = 0;
    bufferIndex = 0;

    printlnLGreen("[OK] Memoria limpa e ponteiros reiniciados.");
    return true;
}

bool w25qApagarTudo(void) {
    const u32 totalBlocos = flashCapacity / W25Q_BLOCK_SIZE;
    printflnLYellow("\r\n[SISTEMA] Apagando chip inteiro (%lu blocos de 64 KB)...", (unsigned long)totalBlocos);

    for (u32 b = 0; b < totalBlocos; ++b) {
        if (!flash.eraseBlock64K(b * W25Q_BLOCK_SIZE)) {
            printflnRed("[ERRO] Falha no apagamento completo no bloco %lu.", (unsigned long)b);
            return false;
        }
        feedWatchdog();
    }

    currentAddr = W25Q_DATA_START_ADDR;
    recordCount = 0;
    bufferIndex = 0;

    printlnLGreen("[OK] Chip W25Q completamente formatado.");
    return true;
}

// ==============================================================================
// VISUALIZAÇÃO (DASHBOARD COLORIDA E EXPORTAÇÃO CSV)
// ==============================================================================

void w25qVisualizarLogsTabela(void) {
    if (printsBloqueadosPorVoo()) { return; }
    w25qDescarregarBuffer();

    if (recordCount == 0) {
        printlnRed("\r\n[AVISO] Nenhum registro gravado na Flash.");
        return;
    }

    char linha[256];
    char barras[55];

    printlnLCyan("\r\n======================================================================================================================");
    printlnLYellow("                                     RELATORIO DE VOO - VISUALIZADOR (RAD v3)                                    ");
    printlnLCyan("======================================================================================================================");
    printlnLMagenta("  TEMPO(s) |        TELEMETRIA (A/V/P/T)           |     STATUS     | GRAFICO (# = 10m)                                ");
    printlnLCyan("-----------|---------------------------------------|----------------|--------------------------------------------------");

    for (u32 i = 0; i < recordCount; ++i) {
        LogData_t reg;
        if (!w25qLerRegistro(i, &reg)) { break; }

        int numBarras = (int)(reg.altitudeM / 10.0f);
        if (numBarras > 50) numBarras = 50;
        if (numBarras < 0)  numBarras = 0;

        memset(barras, 0, sizeof(barras));
        for (int k = 0; k < numBarras; ++k) barras[k] = '#';

        u8 st = reg.stateAndFlags & 0x0F;
        if (st > 6) { st = 6; }

        const float tempoS  = reg.timestampMs / 1000.0f;
        const float tempC   = reg.temperatura_cC / 100.0f;
        const float presHpa = reg.pressaoPa / 100.0f;

        snprintf(linha, sizeof(linha),
            "%s %9.2f %s|"
            "%s A:%7.1f V:%7.1f P:%6.1f T:%6.1f %s|"
            "%s %-14s %s|"
            "%s %s",
            RESET, tempoS, LMAGENTA,
            LCYAN, reg.altitudeM, reg.velocidadeMs, presHpa, tempC, LMAGENTA,
            LYELLOW, PRINT_ESTADO[st], LMAGENTA,
            LGREEN, barras);

        printDebug(linha);
        printlnReset("");

        feedWatchdog();
    }

    printlnMagenta("======================================================================================================================");
    printflnLGreen("Total de registros lidos: %lu", (unsigned long)recordCount);
}

void w25qVisualizarLogsCSV(void) {
    if (printsBloqueadosPorVoo()) { return; }
    w25qDescarregarBuffer();

    if (recordCount == 0) {
        printlnRed("\r\n[AVISO] Nenhum registro gravado na Flash.");
        return;
    }

    CaixaPretaInfo_t info = w25qLerCaixaPreta();

    printlnDebug("");
    printlnDebug("================ DADOS DO VOO (CSV) ================");
    printDebug("# PressaoBase_Pa: "); printlnDebug(info.header.pressaoRefPa, 1);
    printDebug("# TempBase_C: ");     printlnDebug(info.header.temperaturaRefC, 2);
    printlnDebug("Registro,Tempo_ms,Estado,Altitude_m,Velocidade_ms,Pressao_Pa,Temp_C,AccX_ms2,AccY_ms2,AccZ_ms2,GyroX_dps,GyroY_dps,GyroZ_dps,Flags,CRC");

    for (u32 i = 0; i < recordCount; ++i) {
        LogData_t r;
        if (!w25qLerRegistro(i, &r)) { break; }

        const bool crcOk    = (r.crc8 == crc8((const u8 *)&r, sizeof(r) - 1));
        const u8   estado   = r.stateAndFlags & 0x0F;
        const u8   flags    = r.stateAndFlags & 0xF0;
        const bool imuValid = (flags & LOG_FLAG_IMU_VALID) != 0;

        printDebug(i + 1); printDebug(',');
        printDebug(r.timestampMs); printDebug(',');
        printDebug(estado); printDebug(',');
        printDebug(r.altitudeM, 2); printDebug(',');
        printDebug(r.velocidadeMs, 2); printDebug(',');
        printDebug(r.pressaoPa, 1); printDebug(',');
        printDebug(r.temperatura_cC / 100.0f, 2); printDebug(',');

        for (u8 a = 0; a < 3; ++a) {
            if (imuValid) { printDebug(r.accelRaw[a] * MPU_ACC_TO_MS2, 3); }
            printDebug(',');
        }
        for (u8 a = 0; a < 3; ++a) {
            if (imuValid) {
                const i32 corr = (i32)r.gyroRaw[a] - (info.header.mpuCalibrated ? info.header.gyroBiasRaw[a] : 0);
                printDebug(corr * MPU_GYR_TO_DPS, 3);
            }
            printDebug(',');
        }

        printDebug(flags, HEX); printDebug(',');
        printlnDebug(crcOk ? "OK" : "CRC_ERR");

        feedWatchdog();
    }
    printlnDebug("====================================================");
}

void w25qVisualizarUltimoLog(void) {
    if (printsBloqueadosPorVoo()) { return; }

    const u32 total = w25qGetRecordCount();
    if (!total) {
        printlnRed("\r\n[AVISO] Nenhum registro na RAM ou Flash.");
        return;
    }

    LogData_t reg;
    if (!w25qLerRegistro(total - 1, &reg)) { return; }

    u8 st = reg.stateAndFlags & 0x0F;
    if (st > 6) { st = 6; }
    const u8 flags = reg.stateAndFlags & 0xF0;

    printflnLCyan("\r\n[ULTIMO LOG #%lu] T: %.2fs | Alt: %.2fm | Vel: %.2fm/s | P: %.1fPa | Temp: %.2fC | Estado: %s",
                  (unsigned long)total,
                  reg.timestampMs / 1000.0f,
                  reg.altitudeM,
                  reg.velocidadeMs,
                  reg.pressaoPa,
                  reg.temperatura_cC / 100.0f,
                  PRINT_ESTADO[st]);

    printflnWhite("  Flags: 0x%02X [%s%s%s%s] | Accel: [%d, %d, %d] | Gyro: [%d, %d, %d]",
                  flags,
                  (flags & LOG_FLAG_PRETRIGGER) ? "PRE " : "",
                  (flags & LOG_FLAG_LAUNCH)     ? "LAUNCH " : "",
                  (flags & LOG_FLAG_RESUMED)    ? "RESUMED " : "",
                  (flags & LOG_FLAG_IMU_VALID)  ? "IMU_OK" : "SEM_IMU",
                  reg.accelRaw[0], reg.accelRaw[1], reg.accelRaw[2],
                  reg.gyroRaw[0],  reg.gyroRaw[1],  reg.gyroRaw[2]);
}

void w25qGerarVooSimulado(void) {
    if (printsBloqueadosPorVoo()) { return; }

    printlnLYellow("\r\n--- GERANDO VOO SIMULADO NA FLASH (SITL 20Hz) ---");
    if (!w25qApagarLogs()) { return; }

    const i16 biasZero[3] = {0, 0, 0};
    w25qGravarHeader(101325.0f, 30.0f, biasZero, false);

    float alt = 0.0f;
    float vel = 0.0f;
    u8 estado = ESTADO_PRONTO;
    i32 tempoMs = -1000; // Começa com 1s de pré-trigger

    for (u16 step = 0; step < 1200; ++step) { // 60 segundos a 20 Hz
        u8 flags = LOG_FLAG_IMU_VALID;

        if (estado == ESTADO_PRONTO) {
            if (tempoMs < 0) flags |= LOG_FLAG_PRETRIGGER;
            else {
                estado = ESTADO_EM_VOO;
                flags |= LOG_FLAG_LAUNCH;
            }
        } else if (estado == ESTADO_EM_VOO) {
            if (tempoMs < 3000) vel += 45.0f * DT;
            else                vel -= 9.81f * DT;

            if (vel <= 0.0f && alt > 50.0f) estado = ESTADO_APOGEU;
        } else if (estado == ESTADO_APOGEU) {
            estado = ESTADO_RECUPERACAO;
            vel = -15.0f;
        } else if (estado == ESTADO_RECUPERACAO) {
            vel += ((-8.0f - vel) * 0.25f);
            if (alt <= 0.0f) {
                estado = ESTADO_POUSADO;
                vel = 0.0f;
                alt = 0.0f;
            }
        }

        alt += vel * DT;
        if (alt < 0.0f) { alt = 0.0f; }

        LogData_t mock;
        memset(&mock, 0, sizeof(mock));
        mock.timestampMs    = tempoMs;
        mock.pressaoPa      = 101325.0f * powf(1.0f - (alt / 44330.0f), 5.255f);
        mock.altitudeM      = alt;
        mock.velocidadeMs   = vel;
        mock.temperatura_cC = (i16)((30.0f - alt * 0.0065f) * 100.0f);
        mock.accelRaw[2]    = 2048; // +1g no eixo Z
        mock.stateAndFlags  = (estado & 0x0F) | flags;

        w25qAdicionarLog(&mock);
        tempoMs += SAMPLE_INTERVAL_MS;
        feedWatchdog();

        if (estado == ESTADO_POUSADO && tempoMs > 30000) break;
    }

    w25qDescarregarBuffer();
    w25qSetStatusVoo(0x00000000);
    printlnLGreen("[OK] Voo simulado gravado! Envie 'V' (tabela) ou 'C' (CSV) para visualizar.");
}