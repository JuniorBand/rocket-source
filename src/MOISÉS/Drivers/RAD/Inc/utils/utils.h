/*
  ******************************************************************************
  * @file    utils.h
  * @date 	 3 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   O utils é um header facilitador/portátil ainda em desenvolvimento.
  ******************************************************************************
*/

//Descrição do utils.h AQUI!


#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef unsigned long ulong;

// 2. DETECÇÃO DE PLATAFORMA (Cross-platform)
#if defined(ARDUINO)
    // Se a flag ARDUINO existir, inclui a base do Arduino/ESP
    #include <Arduino.h>

#elif defined(STM32F411xE) || defined(STM32F401xC) || defined(STM32F401xE) || defined(USE_HAL_DRIVER)
    // Se as flags da ST existirem, inclui a HAL
    #include "stm32f4xx_hal.h"

    // Como estamos no STM32, incluímos o main.h para ter acesso aos #define de pinos.
    // Lembre-se: O main.h não deve dar include em arquivos como lkf.h ou config_voo.h!
    #include "main.h"

	#ifdef STM32F411xE
		#pragma message("Você está utilizando o STM32F411.")
		#include "stm32f411xe.h"
	#endif

	#ifdef STM32F401xC
		#pragma message("Você está utilizando o STM32F401.")
		#include "stm32f401xc.h"
	#endif

#else
    #error "Plataforma não reconhecida! Verifique as configurações do compilador."
#endif



#include "prints.h" // Inclui o prints.h para usar as macros de print, mesmo que sejam "vazias" em MODO_VOO.

#pragma message("Você está usando o utils.h para facilitar a sua vida ;)")


// <<<< Se MODO_VOO está comentado, assume-se: MODO_SOLO. >>>>
//#define MODO_VOO // Descomente para o compilador apagar qualquer print do código se em voo, economizando processamento e memória.


// Transforma de um u8 *ptr em um número 32-bit, 16-bit e 24-bit, singned ou unsigned.
#define TRANSF_32_BIT_LITEND(ptr, n) ((i32)ptr[n + 3] << 24 | (i32)ptr[n + 2] << 16 | (i32)ptr[n + 1] << 8 | ptr[n])
#define TRANSF_16_BIT_LITEND(ptr, n)  ((i16)ptr[n + 1] << 8 | ptr[n])
#define TRANSF_U32_BIT_LITEND(ptr, n) ((u32)ptr[n + 3] << 24 | (u32)ptr[n + 2] << 16 | (u32)ptr[n + 1] << 8 | ptr[n])
#define TRANSF_U16_BIT_LITEND(ptr, n)  ((u16)ptr[n + 1] << 8 | ptr[n])
#define TRANSF_24_BIT_LITEND(ptr, n) ((i32)ptr[n + 2] << 16 | (i32)ptr[n + 1] << 8 | ptr[n])
#define TRANSF_U24_BIT_LITEND(ptr, n) ((u32)ptr[n + 2] << 16 | (u32)ptr[n + 1] << 8 | ptr[n])

#define TRANSF_32_BIT_BIGEND(ptr, n) ((i32)ptr[n] << 24 | (i32)ptr[n + 1] << 16 | (i32)ptr[n + 2] << 8 | ptr[n + 3])
#define TRANSF_16_BIT_BIGEND(ptr, n)  ((i16)ptr[n] << 8 | ptr[n + 1])
#define TRANSF_U32_BIT_BIGEND(ptr, n) ((u32)ptr[n] << 24 | (u32)ptr[n + 1] << 16 | (u32)ptr[n + 2] << 8 | ptr[n + 3])
#define TRANSF_U16_BIT_BIGEND(ptr, n)  ((u16)ptr[n] << 8 | ptr[n + 1])
#define TRANSF_24_BIT_BIGEND(ptr, n) ((i32)ptr[n] << 16 | (i32)ptr[n + 1] << 8 | ptr[n + 2])
#define TRANSF_U24_BIT_BIGEND(ptr, n) ((u32)ptr[n] << 16 | (u32)ptr[n + 1] << 8 | ptr[n + 2])

#if !defined(MIN) || !defined(MAX)
	#define MIN(a, b) (a < b) ? a : b
	#define MAX(a, b) (a < b) ? b : a
#endif

extern const char* ESTADO[];



#if defined(USE_HALDRIVER) || defined(STM32F4xx) || defined(STM32F401xC) || defined(STM32F411xE) || defined(STM32F401xE)
//=============================================================================
// <<<<<< GPIO UTILITIES >>>>>>
//=============================================================================

// OBS: PIN_MASK é 0x1UL << (PIN_NUM) e __builtin_ctz(PIN_MASK) retorna o numero do pino (ex: 5 para 0x20, 6 para 0x40)

// Configura registradores de 2 bits (MODER, OSPEEDR, PUPDR) usando PIN_MASK
#define GPIO_CONFIG_2BIT(PORT, PIN_MASK, REG, VALUE) \
    ((PORT)->REG = ((PORT)->REG & ~(0x3UL << (__builtin_ctz(PIN_MASK) * 2))) | ((VALUE) << (__builtin_ctz(PIN_MASK) * 2)))

// Configura registradores de 1 bit (OTYPER) usando PIN_MASK
#define GPIO_CONFIG_1BIT(PORT, PIN_MASK, REG, VALUE) \
    ((PORT)->REG = ((PORT)->REG & ~(0x1UL << __builtin_ctz(PIN_MASK))) | ((VALUE) << __builtin_ctz(PIN_MASK)))


// Macros de Operacao (Leitura / Escrita)
// Macros otimizados para usar as definições nativas (ex: GPIO_PIN_13)

#include "stm32f4xx_hal.h" // Necessário para reconhecer o GPIO_TypeDef

#ifndef EM_VOO

	static inline void writePinHigh(GPIO_TypeDef *PORT, u16 PIN_MASK) {
		PORT->BSRR = PIN_MASK;
	}

	static inline void writePinLow(GPIO_TypeDef *PORT, u16 PIN_MASK) {
		PORT->BSRR = (uint32_t)PIN_MASK << 16;
	}

	static inline void togglePin(GPIO_TypeDef *PORT, u16 PIN_MASK) {
		PORT->ODR ^= PIN_MASK;
	}

	static inline u8 readPin(GPIO_TypeDef *PORT, u16 PIN_MASK) {
		// Parênteses corrigidos para garantir que o AND ocorra antes da comparação
		return ((PORT->IDR & PIN_MASK) != 0);
	}

	/*
	 *VERSÕES ANTIGAS COMO MACROS
	#define GPIO_WRITE_HIGH(PORT, PIN_MASK) ((PORT)->BSRR = (PIN_MASK))
	#define GPIO_WRITE_LOW(PORT, PIN_MASK)  ((PORT)->BSRR = (uint32_t)(PIN_MASK) << 16)
	#define GPIO_TOGGLE(PORT, PIN_MASK)     ((PORT)->ODR ^= (PIN_MASK))
	#define GPIO_READ(PORT, PIN_MASK)       (((PORT)->IDR & (PIN_MASK)) != 0)
	*/
#else
	#define writePinHigh(...)  do { } while(0)
	#define writePinLow(...)   do { } while(0)
	#define togglePin(...)     do { } while(0)
	#define readPin(...)       do { } while(0)
	#define chamarComandosFlashSTM(comando) do { } while(0)
#endif

#endif /* GPIO UTILITIES */






#ifdef __cplusplus
}
#endif

#endif /* UTILS_H */
