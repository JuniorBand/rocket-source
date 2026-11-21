
#include <macros_config.h>
#include <utils.h>
#include <prints.h>
#include <config_voo.h>
#include <bmp280.h>
#include <mpu6050.h>
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


typedef struct LogDataV2_s {
    int32  timestampMs;
    float    pressurePa;
    float    altitudeM;
    i16  temperature_cC;
    u8  flags;
    u8  crc8;
} __attribute__((aligned(4))) LogDataV2_t;

typedef struct LogData_s {
    int32  timestampMs;
    float    pressurePa;
    float    altitudeM;
    i16  temperature_cC;
    i16  accelRaw[3];     // X,Y,Z: 2048 LSB/g
    i16  gyroRaw[3];      // X,Y,Z: 16.4 LSB/(grau/s)
    u8  flags;
    u8  imuStatus;       // bit0: leitura valida; bit1: gyro calibrado
    u8  reserved[3];    // zero para manter 32 bytes
    u8  crc8;
} __attribute__((aligned(4))) LogData_t;


void invalidatePageCache(void);

u32 dataAddress(u32 index);

bool readDataBytes(u32 index, void *dest, u32 size);

bool readData(u32 index, LogData_t &data);

bool readLegacyData(u32 index, LogDataV2_t &data);

bool dataCrcOk(const LogData_t &data);

bool dataCrcOk(const LogDataV2_t &data);

bool slotIsErased(u32 address, u32 slotSize, bool &erased);

// Busca binaria com tamanho de registro variavel (16 B antigo, 32 B novo).
bool findFirstErasedSlot(u32 startAddress, u32 slotCount, u32 slotSize, u32 &result);
bool loadHeader(void);

bool caixaPretaAreaErased(void);
bool writeHeader(void);

bool appendData(const DadosVoo_t &dadosVoo, int32 timestampMs, u8 flags);

bool flashHasData(void);

// Extensao usada da Flash, em bytes, a partir do endereco 0.
bool usedFlashExtent(u32 &extent);