
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
// FLASH
// ============================================================
SPIFlash flash(W25Q_CS, &SPI);

u32 lastChirpTime    = 0;
bool     ttlDumpActive   = false; // transferencia remota de log em andamento

bool     watchdogStarted  = false;

u8  pageCache[FLASH_PAGE_SIZE];
u32 pageCacheAddr = 0xFFFFFFFF;

void invalidatePageCache() {
    pageCacheAddr = 0xFFFFFFFF;
}

u32 recordAddress(u32 index) {
    return DATA_START_ADDR + index * activeRecordSize;
}

bool readRecordBytes(u32 index, void *dest, u32 size) {
    if (size != activeRecordSize || index >= recordCount) return false;
    const u32 address = recordAddress(index);
    const u32 page = address & ~(FLASH_PAGE_SIZE - 1);
    if (page != pageCacheAddr) {
        if (!flash.readByteArray(page, pageCache, FLASH_PAGE_SIZE)) {
            invalidatePageCache();
            return false;
        }
        pageCacheAddr = page;
    }
    // Ambos os formatos sao divisores exatos de 256 B: nao cruzam pagina.
    memcpy(dest, &pageCache[address - page], size);
    return true;
}

bool readRecord(u32 index, LogRecord &record) {
    return readRecordBytes(index, &record, sizeof(record));
}

bool readLegacyRecord(u32 index, LogRecordV2 &record) {
    return readRecordBytes(index, &record, sizeof(record));
}

bool recordCrcOk(const LogRecord &record) {
    return record.crc8 == crc8(reinterpret_cast<const u8 *>(&record), sizeof(record) - 1);
}

bool recordCrcOk(const LogRecordV2 &record) {
    return record.crc8 == crc8(reinterpret_cast<const u8 *>(&record), sizeof(record) - 1);
}

bool slotIsErased(u32 address, u32 slotSize, bool &erased) {
    u8 bytes[RECORD_SIZE];
    if (slotSize > sizeof(bytes)) return false;
    if (!flash.readByteArray(address, bytes, slotSize)) return false;
    erased = isErased(bytes, slotSize);
    return true;
}

// Busca binaria com tamanho de registro variavel (16 B antigo, 32 B novo).
bool findFirstErasedSlot(u32 startAddress, u32 slotCount,
                         u32 slotSize, u32 &result) {
    u32 low = 0, high = slotCount;
    while (low < high) {
        const u32 mid = low + (high - low) / 2;
        bool erased = false;
        if (!slotIsErased(startAddress + mid * slotSize, slotSize, erased)) return false;
        if (erased) high = mid; else low = mid + 1;
        feedWatchdog();
    }
    result = low;
    return true;
}

bool salvarCaixaPreta() {
    CaixaPreta_t candidate;
    if (!flash.readByteArray(HEADER_ADDR, reinterpret_cast<u8 *>(&candidate),
                             sizeof(candidate))) return false;
    const bool formatKnown =
        (candidate.version == HEADER_VERSION && candidate.recordSize == RECORD_SIZE) ||
        (candidate.version == LEGACY_VERSION && candidate.recordSize == LEGACY_RECORD_SIZE);
    const bool valid =
        candidate.magic == HEADER_MAGIC && formatKnown &&
        candidate.crc8 == crc8(reinterpret_cast<const u8 *>(&candidate),
                               sizeof(candidate) - 1);
    if (valid) {
        caixaPreta = candidate;
        activeRecordSize = candidate.recordSize;
    }
    return valid;
}

bool caixaPretaAreaErased() {
    u8 bytes[sizeof(FlightHeader)];
    if (!flash.readByteArray(HEADER_ADDR, bytes, sizeof(bytes))) return false;
    return isErased(bytes, sizeof(bytes));
}

bool writeHeader() {
    if (activeRecordSize != RECORD_SIZE || recordCount != 0) return false;
    memset(&caixaPreta, 0, sizeof(caixaPreta));
    caixaPreta.magic = HEADER_MAGIC;
    caixaPreta.version = HEADER_VERSION;
    caixaPreta.recordSize = RECORD_SIZE;
    caixaPreta.sampleIntervalMs = SAMPLE_INTERVAL_MS;
    caixaPreta.pretriggerSamples = PRETRIGGER_SAMPLES;
    caixaPreta.basePressurePa = basePressurePa;
    caixaPreta.baseTemperatureC = baseTemperatureC;
    caixaPreta.launchThresholdM = LAUNCH_THRESHOLD_M;
    for (u8 a = 0; a < 3; ++a) caixaPreta.gyroBiasRaw[a] = gyroBiasRaw[a];
    caixaPreta.mpuCalibrated = gyroCalibrated ? 1 : 0;
    caixaPreta.crc8 = crc8(reinterpret_cast<const u8 *>(&caixaPreta), sizeof(caixaPreta) - 1);
    invalidatePageCache();
    if (!flash.writeByteArray(HEADER_ADDR, reinterpret_cast<u8 *>(&caixaPreta),
                              sizeof(caixaPreta))) return false;
    caixaPretaValid = true;
    return true;
}

bool registrarDados(const DadosVoo_t &dadosVoo, int32 timestampMs, u8 flags) {
    if (activeRecordSize != RECORD_SIZE || recordCount >= maxRecords) return false;
    LogRecord record = {};
    record.timestampMs = timestampMs;
    record.pressurePa = dadosVoo.pressurePa;
    record.altitudeM = dadosVoo.altitudeM;
    record.temperature_cC = static_cast<i16>(lroundf(dadosVoo.temperatureC * 100.0f));
    for (u8 a = 0; a < 3; ++a) {
        record.accelRaw[a] = dadosVoo.accelRaw[a];
        record.gyroRaw[a] = dadosVoo.gyroRaw[a];
    }
    record.flags = static_cast<u8>((flags | pendingRecordFlags) & 0x7F);
    record.imuStatus = dadosVoo.imuStatus;
    record.crc8 = crc8(reinterpret_cast<const u8 *>(&record), sizeof(record) - 1);
    invalidatePageCache();
    if (!flash.writeByteArray(recordAddress(recordCount),
            reinterpret_cast<u8 *>(&record), sizeof(record))) { return false; }
    pendingRecordFlags = 0;
    ++recordCount;
    return true;
}

bool flashHasData() {
    return caixaPretaValid || unknownFlashData || recordCount > 0;
}

// Extensao usada da Flash, em bytes, a partir do endereco 0.
bool usedFlashExtent(u32 &extent) {
    if (unknownFlashData) {
        // Formato nao reconhecido: por seguranca, o comando A apagara TODOS os blocos.
        extent = flashCapacity;
    } else {
        extent = recordAddress(recordCount);
    }
    if (extent < DATA_START_ADDR) extent = DATA_START_ADDR;
    if (extent > flashCapacity) extent = flashCapacity;
    return true;
}

bool isErased(const u8 *bytes, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (bytes[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

void adicionarLogW25Q(const DadosVoo_t &dadosVoo) {
    const int32 timestamp = static_cast<int32>(dadosVoo.uptimeMs - flightStartTime);

    u8 flags = 0;
    const bool memoriaCheia = (recordCount + 1 >= maxRecords);
    const bool timeoutDetectado = (timestamp >= static_cast<int32>(TEMPO_MAX_VOO_MS));

    if (memoriaCheia) {
        flags |= FLAG_END_FULL;
    } else if (timeoutDetectado) {
        flags |= FLAG_END_TIMEOUT;
    } else if (pousoDetectado) {
        flags |= FLAG_END_LANDED;
    }

    if (!registrarDados(dadosVoo, timestamp, flags)) {
        stopWithError(F("Falha ao gravar na Flash; gravacao parada."));
        return;
    }

    latestStored = true;

    if (memoriaCheia) {
        stopRecording(FLAG_END_FULL, false);
    } else if (timeoutDetectado) {
        stopRecording(FLAG_END_TIMEOUT, false);
    } else if (pousoDetectado) {
        stopRecording(FLAG_END_LANDED, false);
    }
}