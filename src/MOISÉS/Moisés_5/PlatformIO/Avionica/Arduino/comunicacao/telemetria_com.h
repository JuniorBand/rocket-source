/*
  ******************************************************************************
  * @file    telemetria_com.h
  * @date 	 20 de nov. de 2026
  * @author  Junior Bandeira
  * @brief   Header do Gerenciador de Comunicacao de Telemetria para PlatformIO 
  * com Arduino (Estacao Solo)
  ******************************************************************************
*/

#include <macros_config.h>
#include <utils.h>
#include <prints.h>
#include <config_voo.h>
#include <bmp280.h>
#include <mpu6050.h>
#include <w25q.h>
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

void ttlPush16(u8 *p, u8 &at, i16 v);
void ttlPush32(u8 *p, u8 &at, u32 v);
i16 ttlClampI16(float v);
u16 ttlCRC16(const u8 *data, size_t len);
bool ttlPacket(u8 type, u8 seq, const u8 *payload, u8 length);
void ttlReply(u8 type, u8 seq, u8 command, u8 status);
void ttlSendStatus(u8 seq);
void ttlSendTelemetry(void);
void ttlHandleCommand(u8 seq, const u8 *p, u8 len);
void ttlRxByte(u8 c);
void ttlReceiveCommands(void);
void ttlStreamLog(void);