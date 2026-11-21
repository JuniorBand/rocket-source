

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

#if BMP_USE_HW_SPI
// Os pinos do SPI3 são definidos em setup() com PinName (PB_x_ALT1).
// Os macros PBx_ALT1 não servem no construtor: são truncados para 8 bits
// e cairiam no SPI1, o mesmo da Flash.
SPIClass SPI_BMP;
Adafruit_BMP280 bmp(BMP_CS, &SPI_BMP);
#else
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);
#endif

void bmpSetup() {
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
}


bool readRawSensor(float &pressurePa, float &temperatureC) {
    temperatureC = bmp.readTemperature();
    pressurePa = bmp.readPressure();

    return isfinite(pressurePa) && isfinite(temperatureC) &&
           pressurePa > PRESSURE_MIN_PA && pressurePa < PRESSURE_MAX_PA &&
           temperatureC > -40.0f && temperatureC < 85.0f;
}

float relativeAltitude(float pressurePa) {
    if (basePressurePa <= 0.0f) {
        return NAN;
    }
    return 44330.0f * (1.0f - powf(pressurePa / basePressurePa, 0.1902949f));
}
