/*
  ******************************************************************************
  * @file    usb_com.h
  * @date 	 4 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Header do Gerenciador de Comunicacao USB CDC (Estacao Solo)
  ******************************************************************************
*/

#ifndef USB_COM_H
#define USB_COM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <utils.h>
#include <main.h>
#include <usbd_cdc_if.h>


// Redirecionamento do Printf (Mágica do GCC)
// Sempre que você chamar printf(), o compilador vai redirecionar os dados para ca.
int _write(int file, char *ptr, int len);
void interruptUSB(uint8_t* Buf, uint32_t *Len);
void processarComandosUSB(void);






#ifdef __cplusplus
}
#endif


#endif /* USB_COM_H */
