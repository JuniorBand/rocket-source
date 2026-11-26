/*
  ******************************************************************************
  * @file    usb_com.h
  * @brief   Interface de Comandos e Diagnostico via USB CDC (Estacao Solo)
  ******************************************************************************
*/

#ifndef USB_COM_H
#define USB_COM_H

#include <macros_config.h>
#include <utils/utils.h>

void usbComImprimirBanner(void);
void usbComImprimirAjuda(void);
void usbComImprimirStatus(void);
void usbComProcessar(void);

#endif /* USB_COM_H */