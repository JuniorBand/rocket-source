/*
  ******************************************************************************
  * @file    flash_stm.h
  * @date 	 3 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Header do driver para o controlar a flash nativa dos STM32F411/STM32F401
  ******************************************************************************
*/


#ifndef FLASH_STM_H
#define FLASH_STM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f4xx_hal.h>
#include <config_voo.h>

// Verifica qual chip o CubeIDE está compilando e ajusta a Flash
#if defined(STM32F411xE) || defined(STM32F401xE)
    // Se for o F411 ou o F401 grande (512 KB)
	#pragma message("Atenção: o chip é o STM32F411 ou STM32F401 de 512 KB!")
    #define ENDERECO_FLASH_ALVO 0x08060000
    #define SETOR_FLASH_ALVO    FLASH_SECTOR_7

// Doc: Sector 7 0x0806 0000 - 0x0807 FFFF 128 Kbytes

#elif defined(STM32F401xC)
    // Se for o F401 padrão (256 KB)
	#pragma message("Atenção: o chip é o STM32F401 de 256 KB!")
    #define ENDERECO_FLASH_ALVO 0x08020000
    #define SETOR_FLASH_ALVO    FLASH_SECTOR_5

#else
    // Trava a compilação se você tentar rodar num chip desconhecido
    #error Microcontrolador nao configurado para gravacao na Flash!
#endif

// Defina o tamanho do seu setor (Para o Setor 7 do F411 ou Setor 5 do F401, são 128 KB)
#define TAMANHO_SETOR_FLASH 0x20000 // Ou 131072 em decimal

// Assinatura hexadecimal para validar se o bloco de memória realmente contém
// dados legítimos do voo. Evita que leituras de sensores corrompidas
// (que gerem 0xFFFFFFFF por acaso) enganem o sistema de varredura.
#define MAGIC_NUMBER_SIRIUS 0xAA55AA55

void salvarDado(DadosVoo_t *dados);

void printFlash(void);

void apagarCaixaPreta(void);


#ifdef __cplusplus
}
#endif

#endif /* FLASH_STM_H */
