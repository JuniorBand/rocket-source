// ============================================================
// ALTÍMETRO / DATALOGGER DE VOO - Black Pill STM32F411CE
// BMP280 (SPI3) + W25Q64 (SPI1) + buzzer + LED
//
// Funções principais:
//  - Telemetria ao vivo na serial em todos os estados (inclusive gravando)
//  - Detecção de lançamento com confirmação por amostras consecutivas
//  - Buffer pré-trigger: grava os ~2 s anteriores ao lançamento
//  - Grava pressão, temperatura e altitude relativa com CRC por registro
//  - Retoma a gravação só se o watchdog reiniciar a placa em voo
//    (ligar a placa normalmente nunca retoma: o log fica protegido)
//  - Watchdog (IWDG)
//  - Grava até receber S ou a Flash encher
//
// Comandos pela serial (só 3):
//   V  mostra os dados do voo (CSV)
//   A  apaga o voo e arma para o próximo
//   S  encerra a gravação (envie depois do pouso)
// ============================================================

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
// PINOS - devem corresponder à fiação atual da placa
// ============================================================

#define BUZZER_PIN PB1
#define LED_PIN    PC13       // LED onboard da Black Pill: ativo em LOW

// W25Q64: SPI1 padrão da Black Pill (PA5 SCK / PA6 MISO / PA7 MOSI)
#define FLASH_CS   PA4

// BMP280: mesmos pinos da versão anterior
#define BMP_CS     PA15
#define BMP_SCK    PB3
#define BMP_MISO   PB4
#define BMP_MOSI   PB5

// 1 = SPI3 em hardware nos pinos PB3/PB4/PB5 (função alternativa ALT1)
// 0 = SPI por software (igual à versão anterior). Use 0 se o BMP280
//     não for encontrado com o SPI em hardware.
#define BMP_USE_HW_SPI 1

// ============================================================
// CONFIGURAÇÕES
// ============================================================

static const uint32_t SAMPLE_INTERVAL_MS      = 50;     // 20 Hz de amostragem e gravação
static const uint32_t TELEMETRY_INTERVAL_MS   = 200;    // 5 Hz de telemetria na serial
static const uint32_t PRETRIGGER_MS           = 2000;   // histórico gravado antes do lançamento
static const uint16_t PRETRIGGER_SAMPLES      = PRETRIGGER_MS / SAMPLE_INTERVAL_MS;

static const float    LAUNCH_THRESHOLD_M      = 3.0f;
static const uint8_t  LAUNCH_CONFIRM_SAMPLES  = 3;      // amostras seguidas acima do limiar

static const uint8_t  CALIB_SAMPLES           = 32;
static const uint8_t  CALIB_MIN_VALID         = 24;
static const uint32_t CALIB_SETTLE_MS         = 300;


static const uint32_t WATCHDOG_TIMEOUT_US     = 4000000; // 4 s
static const uint32_t USB_WAIT_MS             = 1500;

static const uint32_t READY_CHIRP_INTERVAL_MS = 5000;   // 0 desativa o bipe de "armado"
static const float    VZ_FILTER_ALPHA         = 0.2f;

static const float    PRESSURE_MIN_PA         = 30000.0f;
static const float    PRESSURE_MAX_PA         = 120000.0f;

// ============================================================
// LAYOUT DA FLASH
// ============================================================
//
//  0x000000 .. 0x0000FF : cabeçalho do voo (FlightHeader)
//  0x000100 .. fim      : registros LogRecord de 16 bytes, contíguos
//
// Os registros têm 16 bytes e começam alinhados à página (256 bytes),
// então nenhum registro atravessa fronteira de página.

static const uint32_t HEADER_ADDR      = 0x000000;
static const uint32_t DATA_START_ADDR  = 0x000100;
static const uint32_t FLASH_PAGE_SIZE  = 256;
static const uint32_t FLASH_BLOCK_SIZE = 65536;

static const uint32_t HEADER_MAGIC     = 0x31544C46; // "FLT1"
static const uint16_t HEADER_VERSION   = 2;

struct FlightHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t recordSize;
    uint16_t sampleIntervalMs;
    uint16_t pretriggerSamples;
    float    basePressurePa;
    float    baseTemperatureC;
    float    launchThresholdM;
    uint8_t  reserved[7];
    uint8_t  crc8;
};

struct LogRecord {
    int32_t  timestampMs;     // tempo relativo ao lançamento (negativo = pré-trigger)
    float    pressurePa;
    float    altitudeM;       // altitude relativa ao solo
    int16_t  temperature_cC;  // centésimos de °C
    uint8_t  flags;           // RecordFlag; bit 7 sempre 0
    uint8_t  crc8;
};

static_assert(sizeof(FlightHeader) == 32, "FlightHeader deve ter 32 bytes");
static_assert(sizeof(LogRecord) == 16, "LogRecord deve ter 16 bytes");
static_assert(FLASH_PAGE_SIZE % sizeof(LogRecord) == 0, "Registro deve caber inteiro na pagina");

static const uint32_t RECORD_SIZE = sizeof(LogRecord);

enum RecordFlag : uint8_t {
    FLAG_PRETRIGGER = 0x01,
    FLAG_LAUNCH     = 0x02,
    FLAG_RESUMED    = 0x04,
    FLAG_END_LANDED = 0x08,   // gravado por versões anteriores (pouso automático)
    FLAG_END_MANUAL = 0x10,
    FLAG_END_FULL   = 0x20,
};
static const uint8_t FLAG_END_MASK = FLAG_END_LANDED | FLAG_END_MANUAL | FLAG_END_FULL;

// ============================================================
// ESTADOS
// ============================================================

enum FlightState : uint8_t {
    STATE_IDLE,        // armado, aguardando lançamento
    STATE_RECORDING,   // gravando
    STATE_STOPPED      // parado; log preservado ou sistema desarmado
};

struct Sample {
    uint32_t uptimeMs;
    float    pressurePa;
    float    temperatureC;
    float    altitudeM;
};

// ============================================================
// OBJETOS
// ============================================================

#if BMP_USE_HW_SPI
// Os pinos do SPI3 são definidos em setup() com PinName (PB_x_ALT1).
// Os macros PBx_ALT1 não servem no construtor: são truncados para 8 bits
// e cairiam no SPI1, o mesmo da Flash.
SPIClass SPI_BMP;
Adafruit_BMP280 bmp(BMP_CS, &SPI_BMP);
#else
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);
#endif

SPIFlash flash(FLASH_CS, &SPI);

// ============================================================
// ESTADO GLOBAL
// ============================================================

FlightState currentState = STATE_STOPPED;

FlightHeader header;
bool     headerValid      = false;
bool     unknownFlashData = false;   // dados em formato antigo/desconhecido

float    basePressurePa   = 0.0f;
float    baseTemperatureC = 0.0f;

uint32_t flashCapacity    = 0;
uint32_t maxRecords       = 0;
uint32_t recordCount      = 0;
uint32_t flightStartTime  = 0;
uint8_t  pendingRecordFlags = 0;

Sample   latest;
bool     hasLatest        = false;
bool     latestStored     = false;
float    verticalSpeed    = 0.0f;
float    prevAltitude     = 0.0f;
uint32_t prevAltitudeTime = 0;
bool     prevAltitudeValid = false;

uint32_t nextSampleTime        = 0;
uint32_t lastTelemetryTime     = 0;
uint32_t lastSensorErrorPrint  = 0;

uint8_t  launchConfirmCount    = 0;
uint32_t launchCandidateTime   = 0;

Sample   preRing[PRETRIGGER_SAMPLES];
uint16_t preRingHead  = 0;
uint16_t preRingCount = 0;



uint32_t lastChirpTime    = 0;

bool     watchdogStarted  = false;

uint8_t  pageCache[FLASH_PAGE_SIZE];
uint32_t pageCacheAddr = 0xFFFFFFFF;

// ============================================================
// FUNÇÕES AUXILIARES
// ============================================================

void feedWatchdog() {
    if (watchdogStarted) {
        IWatchdog.reload();
    }
}

uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

bool isErased(const uint8_t *bytes, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        if (bytes[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

bool timeReached(uint32_t now, uint32_t target) {
    return static_cast<int32_t>(now - target) >= 0;
}

const __FlashStringHelper *stateName(FlightState state) {
    switch (state) {
        case STATE_IDLE:      return F("ARMADO  ");
        case STATE_RECORDING: return F("GRAVANDO");
        default:              return F("PARADO  ");
    }
}

float relativeAltitude(float pressurePa) {
    if (basePressurePa <= 0.0f) {
        return NAN;
    }
    return 44330.0f * (1.0f - powf(pressurePa / basePressurePa, 0.1902949f));
}

// ============================================================
// BUZZER NÃO BLOQUEANTE (sequência de passos liga/desliga)
// ============================================================

struct BeepStep {
    uint16_t onMs;
    uint16_t offMs;
};

static const uint8_t BEEPER_MAX_STEPS = 64;
BeepStep beeperSteps[BEEPER_MAX_STEPS];
uint8_t  beeperCount = 0;
uint8_t  beeperIndex = 0;
bool     beeperActive = false;
bool     beeperOn = false;
uint32_t beeperNextChange = 0;

void beeperClear() {
    beeperActive = false;
    beeperOn = false;
    beeperCount = 0;
    beeperIndex = 0;
    digitalWrite(BUZZER_PIN, LOW);
}

bool beeperAdd(uint16_t onMs, uint16_t offMs) {
    if (beeperCount >= BEEPER_MAX_STEPS) {
        return false;
    }
    beeperSteps[beeperCount].onMs = onMs;
    beeperSteps[beeperCount].offMs = offMs;
    ++beeperCount;
    return true;
}

void beeperStart() {
    if (beeperCount == 0) {
        return;
    }
    beeperIndex = 0;
    beeperActive = true;
    beeperOn = true;
    digitalWrite(BUZZER_PIN, HIGH);
    beeperNextChange = millis() + beeperSteps[0].onMs;
}

void beeperUpdate() {
    if (!beeperActive) {
        return;
    }

    const uint32_t now = millis();
    if (!timeReached(now, beeperNextChange)) {
        return;
    }

    if (beeperOn) {
        digitalWrite(BUZZER_PIN, LOW);
        beeperOn = false;
        beeperNextChange = now + beeperSteps[beeperIndex].offMs;
    } else {
        ++beeperIndex;
        if (beeperIndex >= beeperCount) {
            beeperActive = false;
            return;
        }
        digitalWrite(BUZZER_PIN, HIGH);
        beeperOn = true;
        beeperNextChange = now + beeperSteps[beeperIndex].onMs;
    }
}

void playBeeps(uint8_t count, uint16_t onMs, uint16_t offMs) {
    beeperClear();
    for (uint8_t i = 0; i < count; ++i) {
        beeperAdd(onMs, (i + 1 < count) ? offMs : 0);
    }
    beeperStart();
}

// Dois bipes curtos e um longo.
void playLaunchPattern() {
    beeperClear();
    beeperAdd(100, 120);
    beeperAdd(100, 120);
    beeperAdd(500, 0);
    beeperStart();
}

// Bipe bloqueante: só para inicialização e erros críticos.
void beepBlocking(uint16_t durationMs, uint8_t repetitions = 1) {
    beeperClear();
    for (uint8_t i = 0; i < repetitions; ++i) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(durationMs);
        digitalWrite(BUZZER_PIN, LOW);
        if (i + 1 < repetitions) {
            delay(80);
        }
        feedWatchdog();
    }
}

void haltWithError(const __FlashStringHelper *message, uint16_t durationMs,
                   uint8_t repetitions, uint16_t pauseMs) {
    while (true) {
        Serial.print(F("[ERRO CRITICO] "));
        Serial.println(message);
        beepBlocking(durationMs, repetitions);
        delay(pauseMs);
    }
}

// ============================================================
// LED DE ESTADO
// ============================================================

void updateLed(uint32_t now) {
    bool on = false;
    switch (currentState) {
        case STATE_IDLE:      on = (now % 1000) < 50;  break;  // piscada curta: armado
        case STATE_RECORDING: on = true;               break;  // aceso: gravando
        default:              on = false;              break;  // apagado: parado
    }
    digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// ============================================================
// SENSOR
// ============================================================

bool readRawSensor(float &pressurePa, float &temperatureC) {
    temperatureC = bmp.readTemperature();
    pressurePa = bmp.readPressure();

    return isfinite(pressurePa) && isfinite(temperatureC) &&
           pressurePa > PRESSURE_MIN_PA && pressurePa < PRESSURE_MAX_PA &&
           temperatureC > -40.0f && temperatureC < 85.0f;
}

void resetFlightTracking() {
    launchConfirmCount = 0;
    preRingHead = 0;
    preRingCount = 0;
    prevAltitudeValid = false;
    verticalSpeed = 0.0f;
    pendingRecordFlags = 0;
}

bool calibrateBase() {
    Serial.println(F("[SISTEMA] Calibrando pressao de referencia no solo..."));

    const uint32_t settleStart = millis();
    while (millis() - settleStart < CALIB_SETTLE_MS) {
        feedWatchdog();
        delay(10);
    }

    double pressureSum = 0.0;
    double temperatureSum = 0.0;
    uint8_t validSamples = 0;

    for (uint8_t i = 0; i < CALIB_SAMPLES; ++i) {
        float pressure;
        float temperature;
        if (readRawSensor(pressure, temperature)) {
            pressureSum += pressure;
            temperatureSum += temperature;
            ++validSamples;
        }
        delay(20);
        feedWatchdog();
    }

    if (validSamples < CALIB_MIN_VALID) {
        Serial.println(F("[ERRO] Leituras validas insuficientes para calibrar o BMP280."));
        return false;
    }

    basePressurePa = static_cast<float>(pressureSum / validSamples);
    baseTemperatureC = static_cast<float>(temperatureSum / validSamples);
    resetFlightTracking();

    Serial.print(F("[OK] Pressao base: "));
    Serial.print(basePressurePa, 1);
    Serial.print(F(" Pa | Temperatura: "));
    Serial.print(baseTemperatureC, 2);
    Serial.println(F(" C"));
    return true;
}

// ============================================================
// FLASH
// ============================================================

void invalidatePageCache() {
    pageCacheAddr = 0xFFFFFFFF;
}

uint32_t recordAddress(uint32_t index) {
    return DATA_START_ADDR + index * RECORD_SIZE;
}

bool readRecord(uint32_t index, LogRecord &record) {
    const uint32_t address = recordAddress(index);
    const uint32_t page = address & ~(FLASH_PAGE_SIZE - 1);

    if (page != pageCacheAddr) {
        if (!flash.readByteArray(page, pageCache, FLASH_PAGE_SIZE)) {
            invalidatePageCache();
            return false;
        }
        pageCacheAddr = page;
    }

    memcpy(&record, &pageCache[address - page], RECORD_SIZE);
    return true;
}

bool recordCrcOk(const LogRecord &record) {
    return record.crc8 == crc8(reinterpret_cast<const uint8_t *>(&record), RECORD_SIZE - 1);
}

bool slotIsErased(uint32_t address, bool &erased) {
    uint8_t bytes[RECORD_SIZE];
    if (!flash.readByteArray(address, bytes, RECORD_SIZE)) {
        return false;
    }
    erased = isErased(bytes, RECORD_SIZE);
    return true;
}

// Busca binária do primeiro slot de 16 bytes apagado, assumindo dados
// contíguos a partir de startAddress. Cerca de 20 leituras para 8 MB.
bool findFirstErasedSlot(uint32_t startAddress, uint32_t slotCount, uint32_t &result) {
    uint32_t low = 0;
    uint32_t high = slotCount;

    while (low < high) {
        const uint32_t mid = low + (high - low) / 2;
        bool erased = false;
        if (!slotIsErased(startAddress + mid * RECORD_SIZE, erased)) {
            return false;
        }
        if (erased) {
            high = mid;
        } else {
            low = mid + 1;
        }
        feedWatchdog();
    }

    result = low;
    return true;
}

bool loadHeader() {
    FlightHeader candidate;
    if (!flash.readByteArray(HEADER_ADDR, reinterpret_cast<uint8_t *>(&candidate),
                             sizeof(candidate))) {
        return false;
    }

    const bool valid =
        candidate.magic == HEADER_MAGIC &&
        candidate.version == HEADER_VERSION &&
        candidate.recordSize == RECORD_SIZE &&
        candidate.crc8 == crc8(reinterpret_cast<const uint8_t *>(&candidate),
                               sizeof(candidate) - 1);

    if (valid) {
        header = candidate;
    }
    return valid;
}

bool headerAreaErased() {
    uint8_t bytes[sizeof(FlightHeader)];
    if (!flash.readByteArray(HEADER_ADDR, bytes, sizeof(bytes))) {
        return false;
    }
    return isErased(bytes, sizeof(bytes));
}

bool writeHeader() {
    memset(&header, 0, sizeof(header));
    header.magic = HEADER_MAGIC;
    header.version = HEADER_VERSION;
    header.recordSize = RECORD_SIZE;
    header.sampleIntervalMs = SAMPLE_INTERVAL_MS;
    header.pretriggerSamples = PRETRIGGER_SAMPLES;
    header.basePressurePa = basePressurePa;
    header.baseTemperatureC = baseTemperatureC;
    header.launchThresholdM = LAUNCH_THRESHOLD_M;
    header.crc8 = crc8(reinterpret_cast<const uint8_t *>(&header), sizeof(header) - 1);

    invalidatePageCache();
    if (!flash.writeByteArray(HEADER_ADDR, reinterpret_cast<uint8_t *>(&header),
                              sizeof(header))) {
        return false;
    }
    headerValid = true;
    return true;
}

bool appendRecord(const Sample &sample, int32_t timestampMs, uint8_t flags) {
    if (recordCount >= maxRecords) {
        return false;
    }

    LogRecord record;
    record.timestampMs = timestampMs;
    record.pressurePa = sample.pressurePa;
    record.altitudeM = sample.altitudeM;
    record.temperature_cC = static_cast<int16_t>(lroundf(sample.temperatureC * 100.0f));
    record.flags = static_cast<uint8_t>((flags | pendingRecordFlags) & 0x7F);
    record.crc8 = crc8(reinterpret_cast<const uint8_t *>(&record), RECORD_SIZE - 1);

    invalidatePageCache();
    if (!flash.writeByteArray(recordAddress(recordCount),
                              reinterpret_cast<uint8_t *>(&record), RECORD_SIZE)) {
        return false;
    }

    pendingRecordFlags = 0;
    ++recordCount;
    return true;
}

bool flashHasData() {
    return headerValid || unknownFlashData || recordCount > 0;
}

// Extensão usada da Flash, em bytes, a partir do endereço 0.
bool usedFlashExtent(uint32_t &extent) {
    if (unknownFlashData) {
        uint32_t slots = 0;
        if (!findFirstErasedSlot(0, flashCapacity / RECORD_SIZE, slots)) {
            return false;
        }
        extent = slots * RECORD_SIZE;
    } else {
        extent = recordAddress(recordCount);
    }

    if (extent < DATA_START_ADDR) {
        extent = DATA_START_ADDR;
    }
    if (extent > flashCapacity) {
        extent = flashCapacity;
    }
    return true;
}

// ============================================================
// CICLO DE VOO
// ============================================================

void preRingPush(const Sample &sample) {
    preRing[preRingHead] = sample;
    preRingHead = (preRingHead + 1) % PRETRIGGER_SAMPLES;
    if (preRingCount < PRETRIGGER_SAMPLES) {
        ++preRingCount;
    }
}

void stopWithError(const __FlashStringHelper *message) {
    Serial.println();
    Serial.print(F("[ERRO] "));
    Serial.println(message);
    currentState = STATE_STOPPED;
    playBeeps(2, 500, 200);
}

void startFlight(uint32_t launchTime) {
    if (!writeHeader()) {
        stopWithError(F("Falha ao gravar o cabecalho do voo; gravacao nao iniciada."));
        return;
    }

    flightStartTime = launchTime;

    // Descarrega o buffer pré-trigger, do mais antigo para o mais recente.
    for (uint16_t i = 0; i < preRingCount; ++i) {
        const uint16_t index =
            (preRingHead + PRETRIGGER_SAMPLES - preRingCount + i) % PRETRIGGER_SAMPLES;
        Sample sample = preRing[index];
        sample.altitudeM = relativeAltitude(sample.pressurePa);

        const int32_t timestamp = static_cast<int32_t>(sample.uptimeMs - launchTime);
        uint8_t flags = (timestamp < 0) ? FLAG_PRETRIGGER : 0;
        if (sample.uptimeMs == launchTime) {
            flags |= FLAG_LAUNCH;
        }

        if (!appendRecord(sample, timestamp, flags)) {
            stopWithError(F("Falha ao gravar o buffer pre-trigger."));
            return;
        }
    }
    preRingCount = 0;
    latestStored = true;

    currentState = STATE_RECORDING;
    playLaunchPattern();

    Serial.println();
    Serial.println(F(">>> LANCAMENTO DETECTADO - GRAVACAO INICIADA <<<"));
}

void handleIdleSample(const Sample &sample) {
    preRingPush(sample);

    if (sample.altitudeM >= LAUNCH_THRESHOLD_M) {
        if (launchConfirmCount == 0) {
            launchCandidateTime = sample.uptimeMs;
        }
        ++launchConfirmCount;

        if (launchConfirmCount >= LAUNCH_CONFIRM_SAMPLES) {
            startFlight(launchCandidateTime);
        }
        return;
    }

    // O zero fica fixo desde a calibração (como no código original).
    launchConfirmCount = 0;
}

void handleRecordingSample(const Sample &sample) {
    const int32_t timestamp = static_cast<int32_t>(sample.uptimeMs - flightStartTime);

    uint8_t flags = 0;
    const bool lastSlot = (recordCount + 1 >= maxRecords);
    if (lastSlot) {
        flags |= FLAG_END_FULL;
    }

    if (!appendRecord(sample, timestamp, flags)) {
        stopWithError(F("Falha ao gravar na Flash; gravacao parada."));
        return;
    }
    latestStored = true;

    if (lastSlot) {
        stopWithError(F("Memoria Flash cheia; gravacao parada."));
    }
}

void processSample(uint32_t now) {
    float pressure;
    float temperature;

    latestStored = false;

    if (!readRawSensor(pressure, temperature)) {
        if (now - lastSensorErrorPrint >= 1000) {
            Serial.println(F("[ERRO] Leitura invalida do BMP280; amostra ignorada."));
            lastSensorErrorPrint = now;
        }
        return;
    }

    Sample sample;
    sample.uptimeMs = now;
    sample.pressurePa = pressure;
    sample.temperatureC = temperature;
    sample.altitudeM = relativeAltitude(pressure);

    // Velocidade vertical estimada (filtrada) para a telemetria.
    if (prevAltitudeValid && now != prevAltitudeTime) {
        const float dt = (now - prevAltitudeTime) / 1000.0f;
        const float rawSpeed = (sample.altitudeM - prevAltitude) / dt;
        verticalSpeed += VZ_FILTER_ALPHA * (rawSpeed - verticalSpeed);
    }
    prevAltitude = sample.altitudeM;
    prevAltitudeTime = now;
    prevAltitudeValid = true;

    latest = sample;
    hasLatest = true;

    if (currentState == STATE_IDLE) {
        handleIdleSample(sample);
    } else if (currentState == STATE_RECORDING) {
        handleRecordingSample(sample);
    }
}

// ============================================================
// TELEMETRIA AO VIVO
// ============================================================

// Só imprime se houver monitor conectado e espaço no buffer de envio,
// para nunca bloquear o loop durante o voo.
void printTelemetry(uint32_t now) {
    if (!hasLatest) {
        return;
    }
    if (now - lastTelemetryTime < TELEMETRY_INTERVAL_MS) {
        return;
    }
    lastTelemetryTime = now;

    if (!Serial || Serial.availableForWrite() < 100) {
        return;
    }

    Serial.print(F("TLM "));
    Serial.print(stateName(currentState));
    Serial.print(F(" t="));
    if (currentState == STATE_RECORDING) {
        Serial.print(static_cast<int32_t>(now - flightStartTime) / 1000.0f, 2);
        Serial.print('s');
    } else {
        Serial.print('-');
    }
    Serial.print(F(" alt="));
    Serial.print(latest.altitudeM, 2);
    Serial.print(F("m vz="));
    Serial.print(verticalSpeed, 1);
    Serial.print(F("m/s P="));
    Serial.print(latest.pressurePa, 0);
    Serial.print(F("Pa n="));
    Serial.println(recordCount);
}

// ============================================================
// COMANDO V - TABELA COMPLETA DO VOO
// ============================================================

void printEvents(const LogRecord &record) {
    bool first = true;
    auto add = [&first](const __FlashStringHelper *text) {
        if (!first) Serial.print('|');
        Serial.print(text);
        first = false;
    };

    if (!recordCrcOk(record))               add(F("CRC_ERRO"));
    if (record.flags & FLAG_PRETRIGGER)     add(F("PRE"));
    if (record.flags & FLAG_LAUNCH)         add(F("LANCAMENTO"));
    if (record.flags & FLAG_RESUMED)        add(F("RETOMADA"));
    if (record.flags & FLAG_END_MANUAL)     add(F("FIM_MANUAL"));
    if (record.flags & FLAG_END_FULL)       add(F("FIM_FLASH_CHEIA"));
}

void executeV() {
    if (currentState == STATE_RECORDING) {
        Serial.println();
        Serial.println(F("[AVISO] Envie S antes de V para encerrar a gravacao."));
        return;
    }
    if (unknownFlashData) {
        Serial.println();
        Serial.println(F("[AVISO] Dados em formato desconhecido; nao e possivel listar."));
        return;
    }
    if (recordCount == 0) {
        Serial.println();
        Serial.println(F("[AVISO] Nenhum registro de voo salvo na Flash."));
        return;
    }

    beeperClear();  // evita buzzer preso durante a listagem

    LogRecord record;

    Serial.println();
    Serial.println(F("================ DADOS DO VOO ================"));
    if (headerValid) {
        Serial.print(F("# Pressao base (Pa): "));
        Serial.println(header.basePressurePa, 1);
        Serial.print(F("# Temperatura base (C): "));
        Serial.println(header.baseTemperatureC, 2);
        Serial.print(F("# Intervalo de amostragem (ms): "));
        Serial.println(header.sampleIntervalMs);
    }
    Serial.println(F("Registro,Tempo_voo_ms,Altitude_m,Pressao_Pa,Temperatura_C,Evento"));

    uint32_t row = 0;
    for (uint32_t i = 0; i < recordCount; ++i) {
        if (!readRecord(i, record)) {
            Serial.println(F("[ERRO] Falha ao ler registro durante exibicao da tabela."));
            break;
        }

        Serial.print(i + 1);
        Serial.print(',');
        Serial.print(record.timestampMs);
        Serial.print(',');
        Serial.print(record.altitudeM, 2);
        Serial.print(',');
        Serial.print(record.pressurePa, 1);
        Serial.print(',');
        Serial.print(record.temperature_cC / 100.0f, 2);
        Serial.print(',');
        printEvents(record);
        Serial.println();

        ++row;
        feedWatchdog();
        beeperUpdate();
    }

    Serial.println(F("================================================"));
    Serial.print(F("Registros listados: "));
    Serial.println(row);
    Serial.println();
}

// ============================================================
// COMANDO S - PARAR GRAVAÇÃO / DESARMAR
// ============================================================

void stopRecordingManual() {
    if (currentState != STATE_RECORDING) {
        Serial.println(F("[STATUS] A gravacao ja nao estava ativa."));
        return;
    }

    // Grava um registro marcador de fim para não retomar após reset.
    Sample endSample = latest;
    if (!hasLatest) {
        endSample.uptimeMs = millis();
        endSample.pressurePa = basePressurePa;
        endSample.temperatureC = baseTemperatureC;
        endSample.altitudeM = 0.0f;
    }
    const int32_t timestamp = static_cast<int32_t>(millis() - flightStartTime);
    const bool markerOk = appendRecord(endSample, timestamp, FLAG_END_MANUAL);

    currentState = STATE_STOPPED;

    Serial.println();
    Serial.println(F("[STATUS] Gravacao parada."));
    if (!markerOk) {
        Serial.println(F("[AVISO] Nao foi possivel gravar o marcador de fim."));
    }
    Serial.print(F("Total de registros salvos: "));
    Serial.println(recordCount);
    playBeeps(3, 100, 80);
}

void executeS() {
    switch (currentState) {
        case STATE_RECORDING:
            stopRecordingManual();
            break;
        case STATE_IDLE:
            currentState = STATE_STOPPED;
            Serial.println();
            Serial.println(F("[STATUS] Sistema desarmado. Envie A para armar de novo."));
            playBeeps(1, 100, 0);
            break;
        default:
            Serial.println();
            Serial.println(F("[STATUS] Sistema ja estava parado."));
            break;
    }
}

// ============================================================
// COMANDO A - APAGAR E PREPARAR NOVO VOO
// ============================================================

void executeA() {
    if (currentState == STATE_RECORDING) {
        Serial.println();
        Serial.println(F("[AVISO] Gravando: envie S antes de apagar."));
        return;
    }

    beeperClear();
    currentState = STATE_STOPPED;
    updateLed(millis());

    uint32_t extent = flashCapacity;
    if (!usedFlashExtent(extent)) {
        Serial.println(F("[ERRO] Falha ao medir a area usada da Flash."));
        return;
    }

    const uint32_t blocks = (extent + FLASH_BLOCK_SIZE - 1) / FLASH_BLOCK_SIZE;
    Serial.println();
    Serial.print(F("[SISTEMA] Apagando "));
    Serial.print(blocks);
    Serial.print(F(" bloco(s) de 64 KB"));

    // Apaga bloco a bloco (em vez de eraseChip) para alimentar o watchdog.
    for (uint32_t block = 0; block < blocks; ++block) {
        if (!flash.eraseBlock64K(block * FLASH_BLOCK_SIZE)) {
            Serial.println();
            Serial.println(F("[ERRO] Falha ao apagar a Flash. Sistema mantido parado."));
            playBeeps(2, 500, 200);
            return;
        }
        feedWatchdog();
        if ((block & 0x07) == 0) {
            Serial.print('.');
        }
    }
    Serial.println();
    invalidatePageCache();

    bool slotErased = false;
    if (!headerAreaErased() || !slotIsErased(DATA_START_ADDR, slotErased) || !slotErased) {
        Serial.println(F("[ERRO] Verificacao do apagamento falhou."));
        playBeeps(2, 500, 200);
        return;
    }

    headerValid = false;
    unknownFlashData = false;
    recordCount = 0;
    flightStartTime = 0;
    resetFlightTracking();

    Serial.println(F("[OK] Flash apagada; dados do voo anterior removidos."));

    if (!calibrateBase()) {
        Serial.println(F("[ERRO] Calibracao falhou. Sistema continuara parado (envie A de novo)."));
        playBeeps(2, 500, 200);
        return;
    }

    currentState = STATE_IDLE;
    lastChirpTime = millis();
    Serial.println(F("[STATUS] Pronto para novo voo; aguardando subida de +3 m."));
    playBeeps(3, 100, 80);
}

// ============================================================
// PROCESSAMENTO DA SERIAL
// ============================================================

void processSerialCommands() {
    while (Serial.available() > 0) {
        const int received = Serial.read();
        if (received < 0) {
            return;
        }

        switch (toupper(received)) {
            case 'V': executeV(); break;
            case 'A': executeA(); break;
            case 'S': executeS(); break;
            default:  break;   // ignora qualquer outro caractere
        }
    }
}

// ============================================================
// INICIALIZAÇÃO
// ============================================================

void waitForSerial(uint32_t timeoutMs) {
    const uint32_t start = millis();
    while (!Serial && millis() - start < timeoutMs) {
        delay(10);
    }
}

void resumeFlight() {
    basePressurePa = header.basePressurePa;
    baseTemperatureC = header.baseTemperatureC;
    resetFlightTracking();

    // Continua a contagem de tempo a partir do último registro salvo.
    int32_t lastTimestamp = 0;
    LogRecord record;
    if (recordCount > 0 && readRecord(recordCount - 1, record) && recordCrcOk(record)) {
        lastTimestamp = record.timestampMs;
    }
    flightStartTime = millis() - static_cast<uint32_t>(lastTimestamp + SAMPLE_INTERVAL_MS);

    pendingRecordFlags = FLAG_RESUMED;
    currentState = STATE_RECORDING;

    Serial.println(F("[AVISO] Reinicio pelo watchdog em voo; GRAVACAO RETOMADA."));
    Serial.println(F("Usando a pressao base salva no cabecalho (sem recalibrar)."));
}

void setup() {
    const bool watchdogReset = IWatchdog.isReset(true);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(FLASH_CS, OUTPUT);
    digitalWrite(FLASH_CS, HIGH);
    pinMode(BMP_CS, OUTPUT);
    digitalWrite(BMP_CS, HIGH);

    Serial.begin(115200);

    // --- Flash primeiro: decide rápido se é preciso retomar um voo ---
    SPI.begin();
    const bool flashOk = flash.begin();
    bool resumeCandidate = false;
    bool flightEnded = false;

    if (flashOk) {
        flashCapacity = flash.getCapacity();
        if (flashCapacity > DATA_START_ADDR + RECORD_SIZE) {
            maxRecords = (flashCapacity - DATA_START_ADDR) / RECORD_SIZE;
            headerValid = loadHeader();

            uint32_t slots = 0;
            if (findFirstErasedSlot(DATA_START_ADDR, maxRecords, slots)) {
                recordCount = slots;
            }

            if (!headerValid) {
                unknownFlashData = (recordCount > 0) || !headerAreaErased();
                if (unknownFlashData) {
                    recordCount = 0;
                }
            } else if (recordCount > 0) {
                LogRecord last;
                flightEnded = readRecord(recordCount - 1, last) && recordCrcOk(last) &&
                              (last.flags & FLAG_END_MASK);
            }

            // Só retoma se o reset veio do watchdog (travamento em voo).
            // Ligar a placa normalmente nunca retoma a gravação.
            resumeCandidate = watchdogReset && headerValid && !flightEnded &&
                              recordCount < maxRecords;
        }
    }

    // Em retomada de voo não há tempo a perder esperando o USB.
    if (!resumeCandidate) {
        waitForSerial(USB_WAIT_MS);
        beepBlocking(100, 3);
    }

    Serial.println();
    Serial.println(F("Inicializando avionica..."));
    if (watchdogReset) {
        Serial.println(F("[AVISO] Reinicio causado pelo watchdog."));
    }

    // --- Flash ---
    if (!flashOk) {
        haltWithError(F("Memoria SPI Flash nao encontrada."), 100, 2, 1000);
    }
    if (maxRecords == 0) {
        haltWithError(F("Capacidade da Flash insuficiente."), 500, 2, 1000);
    }

    Serial.print(F("[OK] Flash: JEDEC 0x"));
    Serial.print(flash.getJEDECID(), HEX);
    Serial.print(F(", "));
    Serial.print(flashCapacity);
    Serial.println(F(" bytes"));

    // --- BMP280 ---
#if BMP_USE_HW_SPI
    SPI_BMP.setMOSI(PB_5_ALT1);   // SPI3_MOSI (AF6)
    SPI_BMP.setMISO(PB_4_ALT1);   // SPI3_MISO (AF6)
    SPI_BMP.setSCLK(PB_3_ALT1);   // SPI3_SCK  (AF6)
#endif
    if (!bmp.begin()) {
        haltWithError(F("BMP280 nao encontrado. Verifique pinos, alimentacao e BMP_USE_HW_SPI."),
                      500, 1, 500);
    }

    // Oversampling moderado e IIR baixo: resposta rápida (sem atraso na leitura).
    // A filtragem fina deve ser feita no pós-processamento.
    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X1,     // temperatura
        Adafruit_BMP280::SAMPLING_X4,     // pressão
        Adafruit_BMP280::FILTER_X2,
        Adafruit_BMP280::STANDBY_MS_1
    );
    Serial.println(F("[OK] BMP280 inicializado."));

    Serial.print(F("Registros encontrados: "));
    Serial.println(recordCount);

    // --- Decide o estado inicial ---
    if (resumeCandidate) {
        resumeFlight();
    } else if (unknownFlashData) {
        currentState = STATE_STOPPED;
        Serial.println(F("[AVISO] Flash contem dados em formato antigo/desconhecido."));
        Serial.println(F("Envie A para apagar e armar."));
    } else if (headerValid) {
        basePressurePa = header.basePressurePa;
        baseTemperatureC = header.baseTemperatureC;
        currentState = STATE_STOPPED;
        Serial.println(F("[AVISO] Existe um log salvo; gravacao bloqueada para nao misturar voos."));
        Serial.println(F("Envie V para ver os dados ou A para apagar e armar."));
        // A telemetria usa a pressão atual do solo como referência.
        if (!calibrateBase()) {
            Serial.println(F("[AVISO] Calibracao falhou; telemetria usara a base do log."));
            basePressurePa = header.basePressurePa;
        }
    } else {
        if (!calibrateBase()) {
            haltWithError(F("Nao foi possivel calibrar o BMP280."), 500, 2, 1000);
        }
        currentState = STATE_IDLE;
    }

    IWatchdog.begin(WATCHDOG_TIMEOUT_US);
    watchdogStarted = true;

    nextSampleTime = millis();
    lastChirpTime = millis();

    if (!resumeCandidate) {
        beepBlocking(500);
    }

    Serial.println();
    Serial.println(F(">>> SISTEMA INICIALIZADO <<<"));
    switch (currentState) {
        case STATE_IDLE:
            Serial.println(F(">>> Armado: aguardando subida de +3 m <<<"));
            break;
        case STATE_RECORDING:
            Serial.println(F(">>> Gravando (retomado) <<<"));
            break;
        default:
            Serial.println(F(">>> Log preservado: V para ver, A para apagar e armar <<<"));
            break;
    }
    Serial.println(F("Comandos: V = ver dados | A = apagar e armar | S = parar"));
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    feedWatchdog();
    beeperUpdate();
    processSerialCommands();

    const uint32_t now = millis();

    // Amostragem em taxa fixa, sem acumular atraso.
    if (timeReached(now, nextSampleTime)) {
        nextSampleTime += SAMPLE_INTERVAL_MS;
        if (timeReached(now, nextSampleTime)) {
            nextSampleTime = now + SAMPLE_INTERVAL_MS;  // atrasou demais: ressincroniza
        }
        processSample(now);
    }

    printTelemetry(now);
    updateLed(now);

    // Sinais sonoros de estado (não bloqueantes).
    if (!beeperActive) {
        if (currentState == STATE_IDLE && READY_CHIRP_INTERVAL_MS > 0 &&
            now - lastChirpTime >= READY_CHIRP_INTERVAL_MS) {
            lastChirpTime = now;
            playBeeps(1, 30, 0);
        }
    }
}
