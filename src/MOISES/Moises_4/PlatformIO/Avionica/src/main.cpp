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

// MPU6050 soldado nas trilhas do BMI088 (PB14 = antigo MISO; PB15 = antigo MOSI)
// Usa I2C por software; NAO habilitar SPI2 nesses pinos.
#define MPU_SCL    PB14
#define MPU_SDA    PB15
#define MPU_AD0    PB2
#define MPU_ADDR   0x68

// UART2: PA3 RX, PA2 TX. Somente protocolo binario v1, 115200 baud.
// Telemetria 20 Hz + telecomandos/ACK/NACK + transferencia de log.
// Nao existe mais modo texto do Tera Term nesta interface.

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
static const uint32_t MPU_RETRY_MS            = 3000;
static const uint32_t UART_TTL_BAUD           = 115200;  // UART binaria 8N1
static const uint16_t MPU_CAL_SAMPLES         = 96;
static const uint8_t  MPU_FAIL_LIMIT          = 5;

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

static const uint32_t HEADER_ADDR      = 0x000000;
static const uint32_t DATA_START_ADDR  = 0x000100;
static const uint32_t FLASH_PAGE_SIZE  = 256;
static const uint32_t FLASH_BLOCK_SIZE = 65536;

static const uint32_t HEADER_MAGIC     = 0x31544C46; // "FLT1"
static const uint16_t HEADER_VERSION   = 3;
static const uint16_t LEGACY_VERSION   = 2;

struct FlightHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t recordSize;
    uint16_t sampleIntervalMs;
    uint16_t pretriggerSamples;
    float    basePressurePa;
    float    baseTemperatureC;
    float    launchThresholdM;
    int16_t  gyroBiasRaw[3];  // novo v3; ocupava reserved[0..5] no v2
    uint8_t  mpuCalibrated;   // novo v3; ocupava reserved[6] no v2
    uint8_t  crc8;
};

struct LogRecordV2 {
    int32_t  timestampMs;
    float    pressurePa;
    float    altitudeM;
    int16_t  temperature_cC;
    uint8_t  flags;
    uint8_t  crc8;
};

struct LogRecord {
    int32_t  timestampMs;
    float    pressurePa;
    float    altitudeM;
    int16_t  temperature_cC;
    int16_t  accelRaw[3];     // X,Y,Z: 2048 LSB/g
    int16_t  gyroRaw[3];      // X,Y,Z: 16.4 LSB/(grau/s)
    uint8_t  flags;
    uint8_t  imuStatus;       // bit0: leitura valida; bit1: gyro calibrado
    uint8_t  reserved[3];    // zero para manter 32 bytes
    uint8_t  crc8;
};

static_assert(sizeof(FlightHeader) == 32, "Cabecalho deve ter 32 bytes");
static_assert(sizeof(LogRecordV2) == 16, "Registro v2 deve ter 16 bytes");
static_assert(sizeof(LogRecord) == 32, "Registro v3 deve ter 32 bytes");
static_assert(FLASH_PAGE_SIZE % sizeof(LogRecord) == 0, "Registro deve caber na pagina");

static const uint32_t RECORD_SIZE = sizeof(LogRecord);
static const uint32_t LEGACY_RECORD_SIZE = sizeof(LogRecordV2);
static const uint8_t MPU_STATUS_VALID = 0x01;
static const uint8_t MPU_STATUS_GYRO_CALIBRATED = 0x02;

enum RecordFlag : uint8_t {
    FLAG_PRETRIGGER = 0x01,
    FLAG_LAUNCH     = 0x02,
    FLAG_RESUMED    = 0x04,
    FLAG_END_LANDED = 0x08,
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
    int16_t  accelRaw[3];
    int16_t  gyroRaw[3];
    uint8_t  imuStatus;
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

// STM32duino 3.0.0: a classe concreta da UART chama-se Uart (RX, TX).
Uart SerialTTL(PA3, PA2);

// ============================================================
// ESTADO GLOBAL
// ============================================================

FlightState currentState = STATE_STOPPED;

FlightHeader header;
bool     headerValid      = false;
bool     unknownFlashData = false;   // dados em formato antigo/desconhecido

float    basePressurePa   = 0.0f;
float    baseTemperatureC = 0.0f;

// Registro ativo pode ser 16 bytes (voo antigo v2) ou 32 bytes (v3).
uint32_t activeRecordSize = RECORD_SIZE;
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



// Estado do MPU6050: uma falha nao deve interromper o altimetro BMP280.
bool     mpuAvailable = false;
bool     gyroCalibrated = false;
uint8_t  mpuConsecutiveFailures = 0;
uint32_t lastMpuRetryTime = 0;
uint32_t lastMpuErrorPrint = 0;
int16_t  gyroBiasRaw[3] = {0, 0, 0};

uint32_t lastChirpTime    = 0;
bool     ttlDumpActive   = false; // transferencia remota de log em andamento

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
// MPU6050 / I2C POR SOFTWARE EM PB14(SCL) E PB15(SDA)
// ============================================================
// Importante: precisam existir pull-ups em SCL e SDA para 3V3.
// INPUT_PULLUP interno apenas auxilia o teste de bancada.
// Nunca usamos OUTPUT HIGH, pois I2C requer saida tipo open-drain.

void mpuDelayI2C() { delayMicroseconds(7); }

void mpuSclLow() { digitalWrite(MPU_SCL, LOW); pinMode(MPU_SCL, OUTPUT); }
void mpuSclRelease() { pinMode(MPU_SCL, INPUT_PULLUP); }
void mpuSdaLow() { digitalWrite(MPU_SDA, LOW); pinMode(MPU_SDA, OUTPUT); }
void mpuSdaRelease() { pinMode(MPU_SDA, INPUT_PULLUP); }

bool mpuWaitSclHigh() {
    const uint32_t t0 = micros();
    while (digitalRead(MPU_SCL) == LOW) {
        if (static_cast<uint32_t>(micros() - t0) > 2000) return false;
    }
    return true;
}

bool mpuStart() {
    mpuSdaRelease(); mpuDelayI2C();
    mpuSclRelease();
    if (!mpuWaitSclHigh()) return false;
    mpuDelayI2C();
    if (digitalRead(MPU_SDA) == LOW) return false;
    mpuSdaLow(); mpuDelayI2C();
    mpuSclLow(); mpuDelayI2C();
    return true;
}

void mpuStop() {
    mpuSdaLow(); mpuDelayI2C();
    mpuSclRelease();
    (void)mpuWaitSclHigh();
    mpuDelayI2C();
    mpuSdaRelease(); mpuDelayI2C();
}

bool mpuWriteByte(uint8_t value) {
    for (int8_t i = 7; i >= 0; --i) {
        mpuSclLow();
        if (value & (1u << i)) mpuSdaRelease(); else mpuSdaLow();
        mpuDelayI2C();
        mpuSclRelease();
        if (!mpuWaitSclHigh()) return false;
        mpuDelayI2C();
        mpuSclLow();
    }
    mpuSdaRelease(); mpuDelayI2C();
    mpuSclRelease();
    if (!mpuWaitSclHigh()) return false;
    mpuDelayI2C();
    bool ack = (digitalRead(MPU_SDA) == LOW);
    mpuSclLow(); mpuDelayI2C();
    return ack;
}

bool mpuReadByte(uint8_t &value, bool ack) {
    value = 0;
    mpuSdaRelease();
    for (uint8_t i = 0; i < 8; ++i) {
        mpuSclLow(); mpuDelayI2C();
        mpuSclRelease();
        if (!mpuWaitSclHigh()) return false;
        mpuDelayI2C();
        value = static_cast<uint8_t>((value << 1) | (digitalRead(MPU_SDA) ? 1 : 0));
        mpuSclLow();
    }
    if (ack) mpuSdaLow(); else mpuSdaRelease();
    mpuDelayI2C();
    mpuSclRelease();
    if (!mpuWaitSclHigh()) return false;
    mpuDelayI2C();
    mpuSclLow(); mpuSdaRelease();
    return true;
}

bool mpuWriteReg(uint8_t reg, uint8_t value) {
    if (!mpuStart()) { mpuStop(); return false; }
    bool ok = mpuWriteByte(MPU_ADDR << 1);
    if (ok) ok = mpuWriteByte(reg);
    if (ok) ok = mpuWriteByte(value);
    mpuStop();
    return ok;
}

bool mpuReadRegs(uint8_t reg, uint8_t *data, uint8_t count) {
    if (!mpuStart()) { mpuStop(); return false; }
    bool ok = mpuWriteByte(MPU_ADDR << 1);
    if (ok) ok = mpuWriteByte(reg);
    if (ok) ok = mpuStart();  // repeated START
    if (ok) ok = mpuWriteByte((MPU_ADDR << 1) | 1);
    if (ok) {
        for (uint8_t i = 0; i < count && ok; ++i) {
            ok = mpuReadByte(data[i], i + 1 < count);
        }
    }
    mpuStop();
    return ok;
}

int16_t mpuDecode16(uint8_t msb, uint8_t lsb) {
    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
}

bool mpuReadRaw(int16_t (&acc)[3], int16_t (&gyro)[3]) {
    uint8_t data[14];
    if (!mpuReadRegs(0x3B, data, sizeof(data))) return false;
    for (uint8_t i = 0; i < 3; ++i) {
        acc[i]  = mpuDecode16(data[2*i], data[2*i + 1]);
        gyro[i] = mpuDecode16(data[8 + 2*i], data[9 + 2*i]);
    }
    return true;
}

void mpuSetupPins() {
    pinMode(MPU_AD0, OUTPUT);
    digitalWrite(MPU_AD0, LOW);
    mpuSclRelease();
    mpuSdaRelease();
}

bool mpuInit() {
    mpuSclRelease(); mpuSdaRelease();
    delay(5);
    if (digitalRead(MPU_SCL) == LOW || digitalRead(MPU_SDA) == LOW) return false;
    uint8_t whoami = 0;
    if (!mpuReadRegs(0x75, &whoami, 1) || whoami != 0x68) return false;
    // Acorda; filtro DLPF para reduzir ruido; amostragem interna de 100 Hz.
    if (!mpuWriteReg(0x6B, 0x00)) return false;
    delay(10);
    return mpuWriteReg(0x1A, 0x03) &&
           mpuWriteReg(0x19, 0x09) &&
           mpuWriteReg(0x1C, 0x18) &&  // +/-16 g
           mpuWriteReg(0x1B, 0x18);    // +/-2000 graus/s
}

// Apenas com IMU imovel. Nao remover a gravidade do acelerometro.
// Em movimento, nao aplicar offset, para nao contaminar o log.
bool mpuCalibrateGyro() {
    gyroCalibrated = false;
    gyroBiasRaw[0] = gyroBiasRaw[1] = gyroBiasRaw[2] = 0;
    if (!mpuAvailable) return false;
    int32_t sums[3] = {0, 0, 0};
    int32_t minGyro[3] = {INT32_MAX, INT32_MAX, INT32_MAX};
    int32_t maxGyro[3] = {INT32_MIN, INT32_MIN, INT32_MIN};
    const uint16_t count = MPU_CAL_SAMPLES;
    for (uint16_t n = 0; n < count; ++n) {
        int16_t acc[3], gyro[3];
        if (!mpuReadRaw(acc, gyro)) return false;
        for (uint8_t a = 0; a < 3; ++a) {
            sums[a] += gyro[a];
            if (gyro[a] < minGyro[a]) minGyro[a] = gyro[a];
            if (gyro[a] > maxGyro[a]) maxGyro[a] = gyro[a];
        }
        delay(10);
        feedWatchdog();
    }
    // Mais de 4 graus/s de variacao durante a calibração => sensor se moveu.
    for (uint8_t a = 0; a < 3; ++a) {
        if (maxGyro[a] - minGyro[a] > 66) return false; // 4 * 16.4 LSB
    }
    for (uint8_t a = 0; a < 3; ++a) {
        gyroBiasRaw[a] = static_cast<int16_t>(lroundf(sums[a] / float(count)));
    }
    gyroCalibrated = true;
    return true;
}

void mpuAcquire(Sample &sample, uint32_t now) {
    memset(sample.accelRaw, 0, sizeof(sample.accelRaw));
    memset(sample.gyroRaw, 0, sizeof(sample.gyroRaw));
    sample.imuStatus = 0;

    if (!mpuAvailable && now - lastMpuRetryTime >= MPU_RETRY_MS) {
        lastMpuRetryTime = now;
        mpuAvailable = mpuInit();
        mpuConsecutiveFailures = 0;
        if (mpuAvailable) Serial.println(F("[OK] MPU6050 reconectado."));
    }
    if (!mpuAvailable) return;
    if (mpuReadRaw(sample.accelRaw, sample.gyroRaw)) {
        mpuConsecutiveFailures = 0;
        sample.imuStatus = MPU_STATUS_VALID | (gyroCalibrated ? MPU_STATUS_GYRO_CALIBRATED : 0);
    } else {
        if (now - lastMpuErrorPrint >= 1000) {
            Serial.println(F("[AVISO] Falha de leitura MPU6050; registro sem IMU."));
            lastMpuErrorPrint = now;
        }
        if (++mpuConsecutiveFailures >= MPU_FAIL_LIMIT) {
            mpuAvailable = false;
            lastMpuRetryTime = now;
        }
    }
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
    return DATA_START_ADDR + index * activeRecordSize;
}

bool readRecordBytes(uint32_t index, void *dest, uint32_t size) {
    if (size != activeRecordSize || index >= recordCount) return false;
    const uint32_t address = recordAddress(index);
    const uint32_t page = address & ~(FLASH_PAGE_SIZE - 1);
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

bool readRecord(uint32_t index, LogRecord &record) {
    return readRecordBytes(index, &record, sizeof(record));
}

bool readLegacyRecord(uint32_t index, LogRecordV2 &record) {
    return readRecordBytes(index, &record, sizeof(record));
}

bool recordCrcOk(const LogRecord &record) {
    return record.crc8 == crc8(reinterpret_cast<const uint8_t *>(&record), sizeof(record) - 1);
}

bool recordCrcOk(const LogRecordV2 &record) {
    return record.crc8 == crc8(reinterpret_cast<const uint8_t *>(&record), sizeof(record) - 1);
}

bool slotIsErased(uint32_t address, uint32_t slotSize, bool &erased) {
    uint8_t bytes[RECORD_SIZE];
    if (slotSize > sizeof(bytes)) return false;
    if (!flash.readByteArray(address, bytes, slotSize)) return false;
    erased = isErased(bytes, slotSize);
    return true;
}

// Busca binaria com tamanho de registro variavel (16 B antigo, 32 B novo).
bool findFirstErasedSlot(uint32_t startAddress, uint32_t slotCount,
                         uint32_t slotSize, uint32_t &result) {
    uint32_t low = 0, high = slotCount;
    while (low < high) {
        const uint32_t mid = low + (high - low) / 2;
        bool erased = false;
        if (!slotIsErased(startAddress + mid * slotSize, slotSize, erased)) return false;
        if (erased) high = mid; else low = mid + 1;
        feedWatchdog();
    }
    result = low;
    return true;
}

bool loadHeader() {
    FlightHeader candidate;
    if (!flash.readByteArray(HEADER_ADDR, reinterpret_cast<uint8_t *>(&candidate),
                             sizeof(candidate))) return false;
    const bool formatKnown =
        (candidate.version == HEADER_VERSION && candidate.recordSize == RECORD_SIZE) ||
        (candidate.version == LEGACY_VERSION && candidate.recordSize == LEGACY_RECORD_SIZE);
    const bool valid =
        candidate.magic == HEADER_MAGIC && formatKnown &&
        candidate.crc8 == crc8(reinterpret_cast<const uint8_t *>(&candidate),
                               sizeof(candidate) - 1);
    if (valid) {
        header = candidate;
        activeRecordSize = candidate.recordSize;
    }
    return valid;
}

bool headerAreaErased() {
    uint8_t bytes[sizeof(FlightHeader)];
    if (!flash.readByteArray(HEADER_ADDR, bytes, sizeof(bytes))) return false;
    return isErased(bytes, sizeof(bytes));
}

bool writeHeader() {
    if (activeRecordSize != RECORD_SIZE || recordCount != 0) return false;
    memset(&header, 0, sizeof(header));
    header.magic = HEADER_MAGIC;
    header.version = HEADER_VERSION;
    header.recordSize = RECORD_SIZE;
    header.sampleIntervalMs = SAMPLE_INTERVAL_MS;
    header.pretriggerSamples = PRETRIGGER_SAMPLES;
    header.basePressurePa = basePressurePa;
    header.baseTemperatureC = baseTemperatureC;
    header.launchThresholdM = LAUNCH_THRESHOLD_M;
    for (uint8_t a = 0; a < 3; ++a) header.gyroBiasRaw[a] = gyroBiasRaw[a];
    header.mpuCalibrated = gyroCalibrated ? 1 : 0;
    header.crc8 = crc8(reinterpret_cast<const uint8_t *>(&header), sizeof(header) - 1);
    invalidatePageCache();
    if (!flash.writeByteArray(HEADER_ADDR, reinterpret_cast<uint8_t *>(&header),
                              sizeof(header))) return false;
    headerValid = true;
    return true;
}

bool appendRecord(const Sample &sample, int32_t timestampMs, uint8_t flags) {
    if (activeRecordSize != RECORD_SIZE || recordCount >= maxRecords) return false;
    LogRecord record = {};
    record.timestampMs = timestampMs;
    record.pressurePa = sample.pressurePa;
    record.altitudeM = sample.altitudeM;
    record.temperature_cC = static_cast<int16_t>(lroundf(sample.temperatureC * 100.0f));
    for (uint8_t a = 0; a < 3; ++a) {
        record.accelRaw[a] = sample.accelRaw[a];
        record.gyroRaw[a] = sample.gyroRaw[a];
    }
    record.flags = static_cast<uint8_t>((flags | pendingRecordFlags) & 0x7F);
    record.imuStatus = sample.imuStatus;
    record.crc8 = crc8(reinterpret_cast<const uint8_t *>(&record), sizeof(record) - 1);
    invalidatePageCache();
    if (!flash.writeByteArray(recordAddress(recordCount),
                              reinterpret_cast<uint8_t *>(&record), sizeof(record))) return false;
    pendingRecordFlags = 0;
    ++recordCount;
    return true;
}

bool flashHasData() {
    return headerValid || unknownFlashData || recordCount > 0;
}

// Extensao usada da Flash, em bytes, a partir do endereco 0.
bool usedFlashExtent(uint32_t &extent) {
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
    mpuAcquire(sample, now);

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

    // Evita bloquear a aquisicao quando nao houver monitor USB conectado.
    if (!Serial || Serial.availableForWrite() < 32) {
        return;
    }

    Serial.print(F("["));
    Serial.print(stateName(currentState));
    Serial.print(F("] Tempo: "));
    if (currentState == STATE_RECORDING) {
        Serial.print(static_cast<int32_t>(now - flightStartTime) / 1000.0f, 2);
        Serial.print(F(" s"));
    } else {
        Serial.print(F("--"));
    }
    Serial.print(F(" | Registros: "));
    Serial.println(recordCount);

    Serial.print(F("  BMP280  | Alt: "));
    Serial.print(latest.altitudeM, 2);
    Serial.print(F(" m | Vz: "));
    Serial.print(verticalSpeed, 1);
    Serial.print(F(" m/s | P: "));
    Serial.print(latest.pressurePa, 0);
    Serial.print(F(" Pa | T: "));
    Serial.print(latest.temperatureC, 1);
    Serial.println(F(" C"));

    if (latest.imuStatus & MPU_STATUS_VALID) {
        Serial.print(F("  MPU6050 | ACC (m/s2)  X: "));
        Serial.print(latest.accelRaw[0] * MPU_ACC_TO_M_S2, 2);
        Serial.print(F("  Y: "));
        Serial.print(latest.accelRaw[1] * MPU_ACC_TO_M_S2, 2);
        Serial.print(F("  Z: "));
        Serial.println(latest.accelRaw[2] * MPU_ACC_TO_M_S2, 2);

        Serial.print(F("          | GYR (graus/s) X: "));
        Serial.print((latest.gyroRaw[0] - (gyroCalibrated ? gyroBiasRaw[0] : 0)) * MPU_GYR_TO_DPS, 1);
        Serial.print(F("  Y: "));
        Serial.print((latest.gyroRaw[1] - (gyroCalibrated ? gyroBiasRaw[1] : 0)) * MPU_GYR_TO_DPS, 1);
        Serial.print(F("  Z: "));
        Serial.println((latest.gyroRaw[2] - (gyroCalibrated ? gyroBiasRaw[2] : 0)) * MPU_GYR_TO_DPS, 1);
    } else {
        Serial.println(F("  MPU6050 | ERRO: sem leitura do sensor"));
    }
    Serial.println(F("------------------------------------------------------------"));
}

// ============================================================
// COMANDO V - TABELA COMPLETA DO VOO
// ============================================================

void printEvents(uint8_t flags, bool crcValid, uint8_t imuStatus = MPU_STATUS_VALID) {
    bool first = true;
    auto add = [&first](const __FlashStringHelper *text) {
        if (!first) Serial.print('|');
        Serial.print(text);
        first = false;
    };
    if (!crcValid)                  add(F("CRC_ERRO"));
    if (flags & FLAG_PRETRIGGER)    add(F("PRE"));
    if (flags & FLAG_LAUNCH)        add(F("LANCAMENTO"));
    if (flags & FLAG_RESUMED)       add(F("RETOMADA"));
    if (flags & FLAG_END_LANDED)    add(F("FIM_POUSO"));
    if (flags & FLAG_END_MANUAL)    add(F("FIM_MANUAL"));
    if (flags & FLAG_END_FULL)      add(F("FIM_FLASH_CHEIA"));
    if (!(imuStatus & MPU_STATUS_VALID)) add(F("IMU_INVALIDO"));
}

void executeV() {
    if (currentState == STATE_RECORDING) {
        Serial.println(F("[AVISO] Envie S antes de V para encerrar a gravacao."));
        return;
    }
    if (unknownFlashData) {
        Serial.println(F("[AVISO] Dados em formato desconhecido; nao e possivel listar."));
        return;
    }
    if (recordCount == 0) {
        Serial.println(F("[AVISO] Nenhum registro de voo salvo na Flash."));
        return;
    }
    beeperClear();
    Serial.println();
    Serial.println(F("================ DADOS DO VOO ================"));
    Serial.print(F("# Formato: v")); Serial.println(header.version);
    Serial.print(F("# Pressao base (Pa): ")); Serial.println(header.basePressurePa, 1);
    Serial.print(F("# Temperatura base (C): ")); Serial.println(header.baseTemperatureC, 2);
    Serial.print(F("# Intervalo (ms): ")); Serial.println(header.sampleIntervalMs);
    if (header.version == HEADER_VERSION) {
        Serial.print(F("# MPU6050: +/-16g, +/-2000dps | giro calibrado: "));
        Serial.println(header.mpuCalibrated ? F("sim") : F("nao"));
        Serial.println(F("Registro,Tempo_voo_ms,Altitude_m,Pressao_Pa,Temperatura_C,AccX_m_s2,AccY_m_s2,AccZ_m_s2,GyroX_dps,GyroY_dps,GyroZ_dps,Evento"));
    } else {
        Serial.println(F("Registro,Tempo_voo_ms,Altitude_m,Pressao_Pa,Temperatura_C,Evento"));
    }

    uint32_t row = 0;
    for (uint32_t i = 0; i < recordCount; ++i) {
        if (header.version == LEGACY_VERSION) {
            LogRecordV2 r;
            if (!readLegacyRecord(i, r)) {
                Serial.println(F("[ERRO] Falha de leitura do registro v2."));
                break;
            }
            Serial.print(i + 1); Serial.print(',');
            Serial.print(r.timestampMs); Serial.print(',');
            Serial.print(r.altitudeM, 2); Serial.print(',');
            Serial.print(r.pressurePa, 1); Serial.print(',');
            Serial.print(r.temperature_cC / 100.0f, 2); Serial.print(',');
            printEvents(r.flags, recordCrcOk(r));
        } else {
            LogRecord r;
            if (!readRecord(i, r)) {
                Serial.println(F("[ERRO] Falha de leitura do registro v3."));
                break;
            }
            Serial.print(i + 1); Serial.print(',');
            Serial.print(r.timestampMs); Serial.print(',');
            Serial.print(r.altitudeM, 2); Serial.print(',');
            Serial.print(r.pressurePa, 1); Serial.print(',');
            Serial.print(r.temperature_cC / 100.0f, 2); Serial.print(',');
            for (uint8_t a = 0; a < 3; ++a) {
                if (r.imuStatus & MPU_STATUS_VALID)
                    Serial.print(r.accelRaw[a] * MPU_ACC_TO_M_S2, 3);
                Serial.print(',');
            }
            for (uint8_t a = 0; a < 3; ++a) {
                if (r.imuStatus & MPU_STATUS_VALID) {
                    const int32_t corrected = int32_t(r.gyroRaw[a]) -
                        ((header.mpuCalibrated && (r.imuStatus & MPU_STATUS_GYRO_CALIBRATED))
                         ? header.gyroBiasRaw[a] : 0);
                    Serial.print(corrected * MPU_GYR_TO_DPS, 3);
                }
                Serial.print(',');
            }
            printEvents(r.flags, recordCrcOk(r), r.imuStatus);
        }
        Serial.println();
        ++row;
        feedWatchdog();
        beeperUpdate();
    }
    Serial.println(F("================================================"));
    Serial.print(F("Registros listados: ")); Serial.println(row);
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
        memset(endSample.accelRaw, 0, sizeof(endSample.accelRaw));
        memset(endSample.gyroRaw, 0, sizeof(endSample.gyroRaw));
        endSample.imuStatus = 0;
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
    ttlDumpActive = false;  // cancela eventual transferencia de log pelo TTL
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
    if (!headerAreaErased() || !slotIsErased(DATA_START_ADDR, RECORD_SIZE, slotErased) || !slotErased) {
        Serial.println(F("[ERRO] Verificacao do apagamento falhou."));
        playBeeps(2, 500, 200);
        return;
    }

    headerValid = false;
    unknownFlashData = false;
    recordCount = 0;
    activeRecordSize = RECORD_SIZE;
    maxRecords = (flashCapacity - DATA_START_ADDR) / activeRecordSize;
    flightStartTime = 0;
    resetFlightTracking();

    Serial.println(F("[OK] Flash apagada; dados do voo anterior removidos."));

    if (!calibrateBase()) {
        Serial.println(F("[ERRO] Calibracao falhou. Sistema continuara parado (envie A de novo)."));
        playBeeps(2, 500, 200);
        return;
    }
    if (mpuAvailable) {
        Serial.println(F("[MPU] Mantenha a placa parada: calibrando giroscopio..."));
        if (mpuCalibrateGyro()) Serial.println(F("[OK] Offset do giroscopio calibrado."));
        else Serial.println(F("[AVISO] Calibracao de giro rejeitada: dados brutos preservados."));
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

static const uint8_t TTL_VER = 1;
static const uint8_t TTL_TELEM = 0x01;
static const uint8_t TTL_CMD = 0x02;
static const uint8_t TTL_ACK = 0x03;
static const uint8_t TTL_NACK = 0x04;
static const uint8_t TTL_STATUS = 0x05;
static const uint8_t TTL_LOGREC = 0x06;
static const uint8_t TTL_LOGINFO = 0x07;
static const uint8_t TTL_LOGEND = 0x08;
static const uint8_t TTL_PAYLOAD_MAX = 48;

uint8_t ttlSeq = 0;
uint8_t ttlRx[TTL_PAYLOAD_MAX + 8];
uint8_t ttlRxLen = 0;
bool ttlDumpInfoSent = false;
uint32_t ttlDumpIndex = 0;
uint32_t ttlDumpNextMs = 0;
uint8_t ttlDumpResult = 0;

void ttlPush16(uint8_t *p, uint8_t &at, int16_t v) {
    const uint16_t bits = static_cast<uint16_t>(v);
    p[at++] = static_cast<uint8_t>(bits);
    p[at++] = static_cast<uint8_t>(bits >> 8);
}
void ttlPush32(uint8_t *p, uint8_t &at, uint32_t v) {
    for (uint8_t n = 0; n < 4; ++n) { p[at++] = static_cast<uint8_t>(v); v >>= 8; }
}
int16_t ttlClampI16(float v) {
    if (!isfinite(v)) return 0;
    if (v > 32767.0f) return INT16_MAX;
    if (v < -32768.0f) return INT16_MIN;
    return static_cast<int16_t>(lroundf(v));
}
uint16_t ttlCRC16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}
bool ttlPacket(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t length) {
    if (length > TTL_PAYLOAD_MAX) return false;
    uint8_t out[TTL_PAYLOAD_MAX + 8];
    out[0] = 0xA5; out[1] = 0x5A;
    out[2] = TTL_VER; out[3] = type; out[4] = seq; out[5] = length;
    if (length > 0) memcpy(out + 6, payload, length);
    const uint16_t crc = ttlCRC16(out + 2, static_cast<size_t>(4 + length));
    out[6 + length] = static_cast<uint8_t>(crc);
    out[7 + length] = static_cast<uint8_t>(crc >> 8);
    const size_t frameLen = static_cast<size_t>(length + 8);
    // Nao bloquear a tarefa de aquisicao quando UART estiver sem espaco.
    if (SerialTTL.availableForWrite() < static_cast<int>(frameLen)) return false;
    return SerialTTL.write(out, frameLen) == frameLen;
}
void ttlReply(uint8_t type, uint8_t seq, uint8_t command, uint8_t status) {
    const uint8_t data[2] = {command, status};
    ttlPacket(type, seq, data, sizeof(data));
}
void ttlSendStatus(uint8_t seq) {
    // state, IMU status, numero de registros, uptime em ms.
    uint8_t p[10]; uint8_t i = 0;
    p[i++] = static_cast<uint8_t>(currentState);
    p[i++] = hasLatest ? latest.imuStatus : 0;
    ttlPush32(p, i, recordCount);
    ttlPush32(p, i, millis());
    ttlPacket(TTL_STATUS, seq, p, i);
}
void ttlSendTelemetry() {
    if (!hasLatest) return;
    uint8_t p[29]; uint8_t i = 0;
    // TEMPO = millis() da amostra desde boot (nao tempo relativo ao lancamento).
    ttlPush32(p, i, latest.uptimeMs);
    ttlPush32(p, i, static_cast<uint32_t>(lroundf(latest.pressurePa)));  // Pa
    ttlPush16(p, i, ttlClampI16(latest.temperatureC * 100.0f));        // 0.01 C
    ttlPush32(p, i, static_cast<uint32_t>(static_cast<int32_t>(lroundf(latest.altitudeM * 100.0f)))); // cm, signed
    ttlPush16(p, i, ttlClampI16(verticalSpeed * 10.0f));              // 0.1 m/s
    for (uint8_t k = 0; k < 3; ++k) {
        // 1 mg por unidade; acelerometro +/-16 g.
        const float mg = (latest.imuStatus & MPU_STATUS_VALID)
            ? (latest.accelRaw[k] * 1000.0f / 2048.0f) : 0.0f;
        ttlPush16(p, i, ttlClampI16(mg));
    }
    for (uint8_t k = 0; k < 3; ++k) {
        // 0.1 grau/s por unidade (offset calibrado quando disponivel).
        const float dg = (latest.imuStatus & MPU_STATUS_VALID)
            ? (int32_t(latest.gyroRaw[k]) - ((latest.imuStatus & MPU_STATUS_GYRO_CALIBRATED) ? gyroBiasRaw[k] : 0))
               * MPU_GYR_TO_DPS * 10.0f : 0.0f;
        ttlPush16(p, i, ttlClampI16(dg));
    }
    // bits 0-1 estado (0 armado / 1 gravando / 2 parado), bit2 IMU valida,
    // bit3 giro calibrado, bit4 BMP valido. Os valores IMU zerados sem bit2 nao sao medidas reais.
    uint8_t state = static_cast<uint8_t>(currentState) | 0x10;
    if (latest.imuStatus & MPU_STATUS_VALID) state |= 0x04;
    if (latest.imuStatus & MPU_STATUS_GYRO_CALIBRATED) state |= 0x08;
    p[i++] = state;
    ttlPacket(TTL_TELEM, ttlSeq++, p, i);  // 29 B payload / 37 B pacote
}
void ttlHandleCommand(uint8_t seq, const uint8_t *p, uint8_t len) {
    if (len != 1) {
        ttlReply(TTL_NACK, seq, len ? p[0] : 0, 1);  // 1=tamanho invalido
        return;
    }
    switch (p[0]) {
        case 0x10:  // PING / status instantaneo; nao altera nada.
            ttlSendStatus(seq);
            break;
        case 0x01:  // V: solicitar transferencia binaria dos registros do voo
            if (currentState != STATE_STOPPED || !headerValid || !recordCount || ttlDumpActive) {
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
                    !headerValid && !unknownFlashData) {
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
void ttlRxByte(uint8_t c) {
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
    const uint8_t full = static_cast<uint8_t>(ttlRx[5] + 8);
    if (ttlRxLen != full) return;
    const uint16_t expected = ttlCRC16(ttlRx + 2, ttlRx[5] + 4);
    const uint16_t found = static_cast<uint16_t>(ttlRx[full - 2]) |
        (static_cast<uint16_t>(ttlRx[full - 1]) << 8);
    if (expected == found && ttlRx[2] == TTL_VER && ttlRx[3] == TTL_CMD)
        ttlHandleCommand(ttlRx[4], ttlRx + 6, ttlRx[5]);
    ttlRxLen = 0;
}
void ttlReceiveCommands() {
    // Limite de bytes por iteracao, mantendo a prioridade do sensoriamento.
    uint8_t n = 0;
    while (SerialTTL.available() > 0 && n++ < 64) {
        const int value = SerialTTL.read();
        if (value >= 0) ttlRxByte(static_cast<uint8_t>(value));
    }
}
void ttlStreamLog() {
    if (!ttlDumpActive) return;
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - ttlDumpNextMs) < 0) return;
    ttlDumpNextMs = now + 15; // envia ate ~66 registros/segundo

    if (!ttlDumpInfoSent) {
        // Count (4), versao da Flash (1), tamanho registro (1), cabecalho original (32)
        uint8_t body[38]; uint8_t k = 0;
        ttlPush32(body, k, recordCount);
        body[k++] = static_cast<uint8_t>(header.version);
        body[k++] = static_cast<uint8_t>(activeRecordSize);
        memcpy(body + k, &header, sizeof(header));
        if (ttlPacket(TTL_LOGINFO, ttlSeq, body, sizeof(body))) {
            ++ttlSeq;
            ttlDumpInfoSent = true;
        }
        return;
    }
    if (ttlDumpIndex < recordCount && ttlDumpResult == 0) {
        uint8_t body[4 + sizeof(LogRecord)]; uint8_t k = 0;
        ttlPush32(body, k, ttlDumpIndex);
        bool ok = false;
        if (header.version == LEGACY_VERSION) {
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
                      static_cast<uint8_t>(4 + activeRecordSize))) {
            ++ttlSeq;
            ++ttlDumpIndex;
        }
        return;
    }
    uint8_t body[5]; uint8_t k = 0;
    ttlPush32(body, k, ttlDumpIndex); body[k++] = ttlDumpResult;
    if (ttlPacket(TTL_LOGEND, ttlSeq, body, sizeof(body))) {
        ++ttlSeq;
        ttlDumpActive = false;
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
    for (uint8_t a = 0; a < 3; ++a) gyroBiasRaw[a] = header.gyroBiasRaw[a];
    gyroCalibrated = (header.mpuCalibrated != 0);

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
    SerialTTL.begin(UART_TTL_BAUD);  // USART2 PA3=RX e PA2=TX, 8N1

    // --- Flash primeiro: decide rápido se é preciso retomar um voo ---
    SPI.begin();
    const bool flashOk = flash.begin();
    bool resumeCandidate = false;
    bool flightEnded = false;

    if (flashOk) {
        flashCapacity = flash.getCapacity();
        if (flashCapacity > DATA_START_ADDR + RECORD_SIZE) {
            headerValid = loadHeader();
            maxRecords = (flashCapacity - DATA_START_ADDR) / activeRecordSize;
            uint32_t slots = 0;
            const bool scanned = findFirstErasedSlot(DATA_START_ADDR, maxRecords,
                                                     activeRecordSize, slots);
            if (scanned) recordCount = slots;
            if (!headerValid) {
                unknownFlashData = !scanned || (recordCount > 0) || !headerAreaErased();
                if (unknownFlashData) recordCount = 0;
            } else if (recordCount > 0) {
                if (header.version == HEADER_VERSION) {
                    LogRecord last;
                    if (readRecord(recordCount - 1, last) && recordCrcOk(last))
                        flightEnded = (last.flags & FLAG_END_MASK) != 0;
                } else {
                    LogRecordV2 last;
                    if (readLegacyRecord(recordCount - 1, last) && recordCrcOk(last))
                        flightEnded = (last.flags & FLAG_END_MASK) != 0;
                }
            }
            // Apenas um voo v3 inacabado pode ser retomado apos watchdog.
            resumeCandidate = watchdogReset && headerValid &&
                              header.version == HEADER_VERSION && !flightEnded &&
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

    // MPU6050: I2C bit-bang no lugar do antigo barramento SPI2 MISO/MOSI.
    mpuSetupPins();
    mpuAvailable = mpuInit();
    lastMpuRetryTime = millis();
    if (mpuAvailable) {
        Serial.println(F("[OK] MPU6050 em PB14/PB15 - 0x68, +/-16g, +/-2000dps."));
    } else {
        Serial.println(F("[AVISO] MPU6050 nao encontrado; BMP280 e Flash continuam ativos."));
    }

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
        if (header.version == LEGACY_VERSION) Serial.println(F("[INFO] Voo antigo v2 legivel pelo comando V."));
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
        if (mpuAvailable) {
            Serial.println(F("[MPU] Mantenha a placa parada: calibrando giroscopio..."));
            if (mpuCalibrateGyro()) Serial.println(F("[OK] Offset do giroscopio calibrado."));
            else Serial.println(F("[AVISO] Calibracao de giro rejeitada; giro sem correcao."));
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
    Serial.println(F("Comandos USB: V = ver dados | A = apagar e armar | S = parar"));
    Serial.println(F("[FIRMWARE] UART BINARIA v2 - S REMOTO + V + A HABILITADOS"));
    Serial.println(F("UART2 PA2 TX / PA3 RX: binario 115200, 20 Hz, sem modo Tera Term"));
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    feedWatchdog();
    beeperUpdate();
    processSerialCommands();
    ttlReceiveCommands();        // Pacotes binarios; respostas ACK/NACK

    const uint32_t now = millis();

    // Amostragem em taxa fixa, sem acumular atraso.
    if (timeReached(now, nextSampleTime)) {
        nextSampleTime += SAMPLE_INTERVAL_MS;
        if (timeReached(now, nextSampleTime)) {
            nextSampleTime = now + SAMPLE_INTERVAL_MS;  // atrasou demais: ressincroniza
        }
        processSample(now);
        if (hasLatest && latest.uptimeMs == now) ttlSendTelemetry();
    }

    ttlStreamLog();               // transferencia binaria de logs
    printTelemetry(now);          // USB CDC continua igual nos dois modos
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