


#include <macros_config.h>
#include <utils.h>
#include <prints.h>
#include <config_voo.h>
#include <bmp280.h>
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



void mpuDelayI2C(void);

void mpuSclLow(void);
void mpuSclRelease(void);
void mpuSdaLow(void);
void mpuSdaRelease(void);

bool mpuWaitSclHigh(void);

bool mpuStart(void);

void mpuStop(void);

bool mpuWriteByte(u8 value);

bool mpuReadByte(u8 &value, bool ack);

bool mpuWriteReg(u8 reg, u8 value);

bool mpuReadRegs(u8 reg, u8 *data, u8 count);

i16 mpuDecode16(u8 msb, u8 lsb);

bool mpuReadRaw(i16 (&acc)[3], i16 (&gyro)[3]);
void mpuSetupPins(void);

bool mpuInit(void);

// Apenas com IMU imovel. Nao remover a gravidade do acelerometro.
// Em movimento, nao aplicar offset, para nao contaminar o log.
bool mpuCalibrateGyro(void);

void mpuAcquire(DadosVoo_t &dadosVoo, u32 now);
