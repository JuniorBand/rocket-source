
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
// MPU6050 / I2C POR SOFTWARE EM PB14(SCL) E PB15(SDA)
// ============================================================
// Importante: precisam existir pull-ups em SCL e SDA para 3V3.
// INPUT_PULLUP interno apenas auxilia o teste de bancada.
// Nunca usamos OUTPUT HIGH, pois I2C requer saida tipo open-drain.

// Estado do MPU6050: uma falha nao deve interromper o altimetro BMP280.
bool     mpuAvailable = false;
bool     gyroCalibrated = false;
u8  mpuConsecutiveFailures = 0;
u32 lastMpuRetryTime = 0;
u32 lastMpuErrorPrint = 0;
i16  gyroBiasRaw[3] = {0, 0, 0};

void mpuDelayI2C() { delayMicroseconds(7); }

void mpuSclLow() { digitalWrite(MPU_SCL, LOW); pinMode(MPU_SCL, OUTPUT); }
void mpuSclRelease() { pinMode(MPU_SCL, INPUT_PULLUP); }
void mpuSdaLow() { digitalWrite(MPU_SDA, LOW); pinMode(MPU_SDA, OUTPUT); }
void mpuSdaRelease() { pinMode(MPU_SDA, INPUT_PULLUP); }


void mpuSetup() {
// MPU6050: I2C bit-bang no lugar do antigo barramento SPI2 MISO/MOSI.
    mpuSetupPins();
    mpuAvailable = mpuInit();
    lastMpuRetryTime = millis();
    if (mpuAvailable) {
        Serial.println(F("[OK] MPU6050 em PB14/PB15 - 0x68, +/-16g, +/-2000dps."));
    } else {
        Serial.println(F("[AVISO] MPU6050 nao encontrado; BMP280 e Flash continuam ativos."));
    }
}


bool mpuWaitSclHigh() {
    const u32 t0 = micros();
    while (digitalRead(MPU_SCL) == LOW) {
        if (static_cast<u32>(micros() - t0) > 2000) return false;
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

bool mpuWriteByte(u8 value) {
    for (i8 i = 7; i >= 0; --i) {
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

bool mpuReadByte(u8 &value, bool ack) {
    value = 0;
    mpuSdaRelease();
    for (u8 i = 0; i < 8; ++i) {
        mpuSclLow(); mpuDelayI2C();
        mpuSclRelease();
        if (!mpuWaitSclHigh()) return false;
        mpuDelayI2C();
        value = static_cast<u8>((value << 1) | (digitalRead(MPU_SDA) ? 1 : 0));
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

bool mpuWriteReg(u8 reg, u8 value) {
    if (!mpuStart()) { mpuStop(); return false; }
    bool ok = mpuWriteByte(MPU_ADDR << 1);
    if (ok) ok = mpuWriteByte(reg);
    if (ok) ok = mpuWriteByte(value);
    mpuStop();
    return ok;
}

bool mpuReadRegs(u8 reg, u8 *data, u8 count) {
    if (!mpuStart()) { mpuStop(); return false; }
    bool ok = mpuWriteByte(MPU_ADDR << 1);
    if (ok) ok = mpuWriteByte(reg);
    if (ok) ok = mpuStart();  // repeated START
    if (ok) ok = mpuWriteByte((MPU_ADDR << 1) | 1);
    if (ok) {
        for (u8 i = 0; i < count && ok; ++i) {
            ok = mpuReadByte(data[i], i + 1 < count);
        }
    }
    mpuStop();
    return ok;
}

i16 mpuDecode16(u8 msb, u8 lsb) {
    return static_cast<i16>((static_cast<u16>(msb) << 8) | lsb);
}

bool mpuReadRaw(i16 (&acc)[3], i16 (&gyro)[3]) {
    u8 data[14];
    if (!mpuReadRegs(0x3B, data, sizeof(data))) return false;
    for (u8 i = 0; i < 3; ++i) {
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
    u8 whoami = 0;
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
    int32 sums[3] = {0, 0, 0};
    int32 minGyro[3] = {INT32_MAX, INT32_MAX, INT32_MAX};
    int32 maxGyro[3] = {INT32_MIN, INT32_MIN, INT32_MIN};
    const u16 count = MPU_CAL_SAMPLES;
    for (u16 n = 0; n < count; ++n) {
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
    // Mais de 4 graus/s de variacao durante a calibração => sensor se moveu.
    for (u8 a = 0; a < 3; ++a) {
        if (maxGyro[a] - minGyro[a] > 66) return false; // 4 * 16.4 LSB
    }
    for (u8 a = 0; a < 3; ++a) {
        gyroBiasRaw[a] = static_cast<i16>(lroundf(sums[a] / float(count)));
    }
    gyroCalibrated = true;
    return true;
}

void mpuAcquire(DadosVoo_t &dadosVoo, u32 now) {
    memset(dadosVoo.accelRaw, 0, sizeof(dadosVoo.accelRaw));
    memset(dadosVoo.gyroRaw, 0, sizeof(dadosVoo.gyroRaw));
    dadosVoo.imuStatus = 0;

    if (!mpuAvailable && now - lastMpuRetryTime >= MPU_RETRY_MS) {
        lastMpuRetryTime = now;
        mpuAvailable = mpuInit();
        mpuConsecutiveFailures = 0;
        if (mpuAvailable) Serial.println(F("[OK] MPU6050 reconectado."));
    }
    if (!mpuAvailable) return;
    if (mpuReadRaw(dadosVoo.accelRaw, dadosVoo.gyroRaw)) {
        mpuConsecutiveFailures = 0;
        dadosVoo.imuStatus = MPU_STATUS_VALID | (gyroCalibrated ? MPU_STATUS_GYRO_CALIBRATED : 0);
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
