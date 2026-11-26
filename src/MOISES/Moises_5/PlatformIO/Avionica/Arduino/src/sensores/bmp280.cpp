/*
  ******************************************************************************
  * @file    bmp280.cpp
  * @brief   Implementacao do Driver para o sensor BMP280
  ******************************************************************************
*/

#include <sensores/bmp280.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <config_voo.h>

#if BMP_USE_HW_SPI
static SPIClass SPI_BMP;
static Adafruit_BMP280 bmp(PIN_BMP_CS, &SPI_BMP);
#else
static Adafruit_BMP280 bmp(PIN_BMP_CS, PIN_BMP_MOSI, PIN_BMP_MISO, PIN_BMP_SCK);
#endif

static inline bool printsBloqueadosPorVoo(void) {
#ifdef EM_VOO
    return (dadosVoo.estadoAtual >= ESTADO_EM_VOO) &&
           (dadosVoo.estadoAtual <= ESTADO_RECUPERACAO);
#else // EM_SOLO sempre printa
    return false;
#endif
}

bool bmp280Init(BMP280_t *dev) {
    pinMode(PIN_BMP_MISO, INPUT_PULLUP);
    pinMode(PIN_BMP_CS, OUTPUT);
    digitalWrite(PIN_BMP_CS, HIGH);

#if BMP_USE_HW_SPI
    SPI_BMP.setMOSI(PB_5_ALT1); // SPI3_MOSI
    SPI_BMP.setMISO(PB_4_ALT1); // SPI3_MISO
    SPI_BMP.setSCLK(PB_3_ALT1); // SPI3_SCK
#endif

    if (!bmp.begin()) {
        printlnRed("[ERRO] BMP280 nao encontrado. Verifique SPI3 / BMP_USE_HW_SPI.");
        return false;
    }

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X1,
        Adafruit_BMP280::SAMPLING_X4,
        Adafruit_BMP280::FILTER_X2,
        Adafruit_BMP280::STANDBY_MS_1
    );

    if (dev) {
        dev->pressaoRefPa = 101325.0f;
        dev->temperaturaRefC = 25.0f;
    }

    printlnLGreen("[OK] BMP280 inicializado.");
    return true;
}

float bmp280CalcAltitude(float pressurePa, float basePressurePa) {
    if (basePressurePa <= 0.0f || pressurePa <= 0.0f) return NAN;
    return 44330.0f * (1.0f - powf(pressurePa / basePressurePa, 0.1902949f));
}

bool bmp280Read(BMP280_t *dev) {
    if (!dev) return false;

    const float temp = bmp.readTemperature();
    const float pres = bmp.readPressure();

    if (!isfinite(pres) || !isfinite(temp) ||
        pres <= PRESSURE_MIN_PA || pres >= PRESSURE_MAX_PA ||
        temp <= -40.0f || temp >= 85.0f) {
        return false;
    }

    dev->temperaturaC = temp;
    dev->pressaoPa    = pres;
    dev->altitudeM    = bmp280CalcAltitude(pres, dev->pressaoRefPa);
    return true;
}

bool bmp280CalibrateBase(BMP280_t *dev) {
    if (!dev) return false;

    printlnLCyan("[SISTEMA] Calibrando pressao de referencia do BMP280 no solo...");

    for (u8 i = 0; i < 30; ++i) {
        feedWatchdog();
        delay(10);
    }

    double sumPres = 0.0;
    double sumTemp = 0.0;
    u8 valid = 0;

    for (u8 i = 0; i < BMP_CALIB_SAMPLES; ++i) {
        const float temp = bmp.readTemperature();
        const float pres = bmp.readPressure();

        if (isfinite(pres) && isfinite(temp) &&
            pres > PRESSURE_MIN_PA && pres < PRESSURE_MAX_PA &&
            temp > -40.0f && temp < 85.0f) {
            sumPres += (double)pres;
            sumTemp += (double)temp;
            ++valid;
        }
        delay(20);
        feedWatchdog();
    }

    if (valid < BMP_CALIB_MIN_VALID) {
        printlnRed("[ERRO] Leituras validas insuficientes na calibracao do BMP280.");
        return false;
    }

    dev->pressaoRefPa    = (float)(sumPres / valid);
    dev->temperaturaRefC = (float)(sumTemp / valid);
    dev->pressaoPa       = dev->pressaoRefPa;
    dev->temperaturaC    = dev->temperaturaRefC;
    dev->altitudeM       = 0.0f;

    printflnLGreen("[OK] Pressao base: %.1f Pa | Temperatura: %.2f C",
                   dev->pressaoRefPa, dev->temperaturaRefC);
    return true;
}