/*
  ******************************************************************************
  * @file    usb_com.h
  * @date 	 20 de nov. de 2026
  * @author  Junior Bandeira
  * @brief   Header do Gerenciador de Comunicacao USB para PlatformIO 
  * com Arduino (Estacao Solo)
  ******************************************************************************
*/

#ifndef USB_COM_H
#define USB_COM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <macros_config.h>
#include <utils.h>
#include <prints.h>
#include <config_voo.h>
#include <bmp280.h>
#include <mpu6050.h>
#include <w25q.h>
#include <telemetria_com.h>

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SPIFlash.h>
#include <IWatchdog.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>


// Redirecionamento do Printf (Mágica do GCC)
// Sempre que você chamar printf(), o compilador vai redirecionar os dados para ca.
int _write(int file, char *ptr, int len);
void interruptUSB(uint8_t* Buf, uint32_t *Len);
void processarComandosUSB(void);






#ifdef __cplusplus
}
#endif


#endif /* USB_COM_H */
