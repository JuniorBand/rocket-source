#include <sensores/mpu6050.h>
#include <math.h>
#include <string.h>
#include <config_voo.h>

static u8 consecutiveFailures = 0;
static u32 lastRetryMs = 0;
static u32 lastWarnMs = 0;

// --- Barramento I2C por software (Open-Drain em PB14/PB15) ---
static inline void i2cDelay(void)      { delayMicroseconds(7); }
static inline void sclLow(void)        { digitalWrite(PIN_MPU_SCL, LOW); pinMode(PIN_MPU_SCL, OUTPUT); }
static inline void sclRelease(void)    { pinMode(PIN_MPU_SCL, INPUT_PULLUP); }
static inline void sdaLow(void)        { digitalWrite(PIN_MPU_SDA, LOW); pinMode(PIN_MPU_SDA, OUTPUT); }
static inline void sdaRelease(void)    { pinMode(PIN_MPU_SDA, INPUT_PULLUP); }


static inline bool printsBloqueadosPorVoo(void) {
#ifdef EM_VOO
    return (dadosVoo.estadoAtual >= ESTADO_EM_VOO) &&
           (dadosVoo.estadoAtual <= ESTADO_RECUPERACAO);
#else // EM_SOLO sempre printa
    return false;
#endif
}

static bool waitSclHigh(void) {
    const u32 t0 = micros();
    while (digitalRead(PIN_MPU_SCL) == LOW) {
        if ((u32)(micros() - t0) > 2000) return false;
    }
    return true;
}

static bool i2cStart(void) {
    sdaRelease(); i2cDelay();
    sclRelease();
    if (!waitSclHigh()) return false;
    i2cDelay();
    if (digitalRead(PIN_MPU_SDA) == LOW) return false;
    sdaLow(); i2cDelay();
    sclLow(); i2cDelay();
    return true;
}

static void i2cStop(void) {
    sdaLow(); i2cDelay();
    sclRelease();
    (void)waitSclHigh();
    i2cDelay();
    sdaRelease(); i2cDelay();
}

static bool i2cWriteByte(u8 value) {
    for (i8 i = 7; i >= 0; --i) {
        sclLow();
        if (value & (1u << i)) sdaRelease(); else sdaLow();
        i2cDelay();
        sclRelease();
        if (!waitSclHigh()) return false;
        i2cDelay();
        sclLow();
    }
    sdaRelease(); i2cDelay();
    sclRelease();
    if (!waitSclHigh()) return false;
    i2cDelay();
    const bool ack = (digitalRead(PIN_MPU_SDA) == LOW);
    sclLow(); i2cDelay();
    return ack;
}

static bool i2cReadByte(u8 &value, bool ack) {
    value = 0;
    sdaRelease();
    for (u8 i = 0; i < 8; ++i) {
        sclLow(); i2cDelay();
        sclRelease();
        if (!waitSclHigh()) return false;
        i2cDelay();
        value = (u8)((value << 1) | (digitalRead(PIN_MPU_SDA) ? 1 : 0));
        sclLow();
    }
    if (ack) sdaLow(); else sdaRelease();
    i2cDelay();
    sclRelease();
    if (!waitSclHigh()) return false;
    i2cDelay();
    sclLow(); sdaRelease();
    return true;
}

static bool mpuWriteReg(u8 reg, u8 value) {
    if (!i2cStart()) { i2cStop(); return false; }
    bool ok = i2cWriteByte(MPU_I2C_ADDR << 1);
    if (ok) ok = i2cWriteByte(reg);
    if (ok) ok = i2cWriteByte(value);
    i2cStop();
    return ok;
}

static bool mpuReadRegs(u8 reg, u8 *data, u8 count) {
    if (!i2cStart()) { i2cStop(); return false; }
    bool ok = i2cWriteByte(MPU_I2C_ADDR << 1);
    if (ok) ok = i2cWriteByte(reg);
    if (ok) ok = i2cStart();
    if (ok) ok = i2cWriteByte((MPU_I2C_ADDR << 1) | 1);
    if (ok) {
        for (u8 i = 0; i < count && ok; ++i) {
            ok = i2cReadByte(data[i], i + 1 < count);
        }
    }
    i2cStop();
    return ok;
}

static bool mpuReadRaw(i16 (&acc)[3], i16 (&gyro)[3]) {
    u8 raw[14];
    if (!mpuReadRegs(0x3B, raw, sizeof(raw))) return false;
    for (u8 i = 0; i < 3; ++i) {
        acc[i]  = transf_16_bigend(raw, 2 * i);
        gyro[i] = transf_16_bigend(raw, 8 + 2 * i);
    }
    return true;
}

// --- API Pública ---

bool mpu6050Init(MPU6050_t *imu) {
    pinMode(PIN_MPU_AUX_XDA, INPUT); // Trilha PB13 ligada ao XDA na PCB
    pinMode(PIN_MPU_AD0, OUTPUT);
    digitalWrite(PIN_MPU_AD0, LOW);

    sclRelease();
    sdaRelease();
    delay(5);

    if (digitalRead(PIN_MPU_SCL) == LOW || digitalRead(PIN_MPU_SDA) == LOW) {
        if (imu) imu->online = false;
        return false;
    }

    u8 whoami = 0;
    if (!mpuReadRegs(0x75, &whoami, 1) || whoami != 0x68) {
        if (imu) imu->online = false;
        return false;
    }

    // Acorda sensor, DLPF ~44Hz, taxa interna 100Hz, +/-16g, +/-2000dps
    const bool ok = mpuWriteReg(0x6B, 0x00) &&
                    (delay(10), true) &&
                    mpuWriteReg(0x1A, 0x03) &&
                    mpuWriteReg(0x19, 0x09) &&
                    mpuWriteReg(0x1C, 0x18) &&
                    mpuWriteReg(0x1B, 0x18);

    if (imu) {
        imu->online = ok;
        imu->status = 0;
    }
    lastRetryMs = millis();
    consecutiveFailures = 0;

    if (ok) {
        if (!printsBloqueadosPorVoo()) {
            printlnLGreen("[OK] MPU6050 inicializado (0x68, +/-16g, +/-2000dps).");
        }
    } else {
        if (!printsBloqueadosPorVoo()) {
            printlnLYellow("[AVISO] MPU6050 nao respondeu; operando apenas com BMP280.");
        }
    }

    return ok;
}

bool mpu6050CalibrateGyro(MPU6050_t *imu) {
    if (!imu || !imu->online) return false;

    imu->gyroCalibrated = false;
    memset(imu->gyroBiasRaw, 0, sizeof(imu->gyroBiasRaw));

    i32 sums[3]    = {0, 0, 0};
    i32 minGyro[3] = {INT32_MAX, INT32_MAX, INT32_MAX};
    i32 maxGyro[3] = {INT32_MIN, INT32_MIN, INT32_MIN};

    for (u16 n = 0; n < MPU_CAL_SAMPLES; ++n) {
        i16 acc[3], gyro[3];
        if (!mpuReadRaw(acc, gyro)) return false;

        for (u8 a = 0; a < 3; ++a) {
            sums[a] += gyro[a];
            if (gyro[a] < minGyro[a]) minGyro[a] = gyro[a];
            if (gyro[a] > maxGyro[a]) maxGyro[a] = gyro[a];
        }
        delay(10);
        feedWatchdog();
    }

    // Rejeita se houve movimento > 4 graus/s (66 LSB) durante a calibracao
    for (u8 a = 0; a < 3; ++a) {
        if (maxGyro[a] - minGyro[a] > 66) return false;
    }

    for (u8 a = 0; a < 3; ++a) {
        imu->gyroBiasRaw[a] = (i16)lroundf(sums[a] / (float)MPU_CAL_SAMPLES);
    }

    imu->gyroCalibrated = true;
    return true;
}

void mpu6050Acquire(MPU6050_t *imu, u32 nowMs) {
    if (!imu) return;

    memset(imu->accelRaw, 0, sizeof(imu->accelRaw));
    memset(imu->gyroRaw, 0, sizeof(imu->gyroRaw));
    imu->status = 0;

    if (!imu->online && (nowMs - lastRetryMs >= MPU_RETRY_MS)) {
        lastRetryMs = nowMs;
        if (mpu6050Init(imu)) {
            if (!printsBloqueadosPorVoo()) {
                printlnLGreen("[OK] MPU6050 reconectado.");
            }
        }
    }

    if (!imu->online) return;

    if (mpuReadRaw(imu->accelRaw, imu->gyroRaw)) {
        consecutiveFailures = 0;
        imu->status = MPU_STATUS_VALID | (imu->gyroCalibrated ? MPU_STATUS_GYRO_CALIBRATED : 0);
    } else {
        if (nowMs - lastWarnMs >= 1000) {
            if (!printsBloqueadosPorVoo()) {
                printlnLYellow("[AVISO] Falha de leitura MPU6050; amostra sem IMU.");
            }
            lastWarnMs = nowMs;
        }
        if (++consecutiveFailures >= MPU_FAIL_LIMIT) {
            imu->online = false;
            lastRetryMs = nowMs;
        }
    }
}