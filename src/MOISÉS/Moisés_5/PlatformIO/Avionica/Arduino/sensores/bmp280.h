



#include <macros_config.h>
#include <utils.h>
#include <prints.h>
#include <config_voo.h>
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

typedef struct BMP280_s {
    u16 c[8];              // Coeficientes C0-C7 lidos da PROM interna do sensor
    float temperatura;          // Temperatura real ja calculada (Celsius)
    float pressao;              // Pressao real ja calculada (hPa)
    float altitude;             // Altitude final relativa ao chao (Metros)
    float pressao_ref;   		// Pressao salva no momento em que a placa ligou no chao (P0)
} BMP280_t;

bool readRawSensor(float &pressurePa, float &temperatureC);