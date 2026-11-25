#ifndef BMP280_DRIVER_H
#define BMP280_DRIVER_H

#include <macros_config.h>
#include <utils/utils.h>

typedef struct {
    float pressaoPa;
    float temperaturaC;
    float altitudeM;
    float pressaoRefPa;
    float temperaturaRefC;
} BMP280_t;

bool  bmp280Init(BMP280_t *dev);
bool  bmp280Read(BMP280_t *dev);
bool  bmp280CalibrateBase(BMP280_t *dev);
float bmp280CalcAltitude(float pressurePa, float basePressurePa);

#endif /* BMP280_DRIVER_H */