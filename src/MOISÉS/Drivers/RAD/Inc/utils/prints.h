/*
  ******************************************************************************
  * @file    prints.h
  * @date 	 10 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Utilidades para prints.
  ******************************************************************************
*/

#ifndef PRINTS_H
#define PRINTS_H


#ifdef __cplusplus
extern "C" {    
#endif


#include "main.h"
#include "stm32f4xx_hal.h"

#ifdef STM32F411xE
	#pragma message("Você está utilizando o STM32F411.")
	#include "stm32f411xe.h"
#endif

#ifdef STM32F401xC
	#pragma message("Você está utilizando o STM32F401.")
	#include "stm32f401xc.h"
#endif



// A macro F() é desnecessária para ESPs e STMs
#if defined(__AVR__)
  #warning Usando __FlashStringHelper.
  // 1. Para placas de 8-bits como Arduino Uno/Mega/Nano (RAM escassa)
  // Força o uso da memória Flash.
  typedef const __FlashStringHelper* cstr;
  #define T(x) F(x)
#elif defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_STM32)
  // 2. Para placas de 32-bits modernas (ESP32, STM32, etc.)
  // Ignora o F() e usa ponteiro de char normal para manter compatibilidade total com printf.
  typedef const char* cstr;
  #define T(x) x
#else
  // 3. Fallback genérico de segurança
  typedef const char* cstr;
  #define T(x) x
#endif

//=============================================================================
// PRINTS COLORIDOS E SEM COR PARA ARDUINO E STM32/ESP32
//=============================================================================

// Cores Normais
#define BLACK       "\x1b[30m"
#define RED         "\x1b[31m"
#define GREEN       "\x1b[32m"
#define YELLOW      "\x1b[33m"
#define BLUE        "\x1b[34m"
#define MAGENTA     "\x1b[35m" // Roxo/Magenta
#define CYAN        "\x1b[36m" // Azul claro/Ciano
#define WHITE       "\x1b[37m"

// Cores claras (Bright / Light)
#define LBLACK   "\x1b[90m" // Cinza
#define LRED     "\x1b[91m"
#define LGREEN   "\x1b[92m"
#define LYELLOW  "\x1b[93m"
#define LBLUE    "\x1b[94m"
#define LMAGENTA "\x1b[95m"
#define LCYAN    "\x1b[96m"
#define LWHITE     "\x1b[97m"

#define RESET   "\x1b[0m" // Para resetar a cor

//###########################################################################################################

// MACROS PARA ARDUINO (Serial.print) - Compatível com Arduino Uno/Mega/Nano e também com ESP32, ESP8266, STM32 usando Serial.print (Arduino.h)
#ifdef ARDUINO
	#pragma message("Utilizando Arduino.h.")

  #ifndef MODO_VOO // Do utils.h
    #pragma message("Em MODO_SOLO: prints via USB ativados.")
    // ==============================================================================
    // MACROS PARA PRINT (Texto Simples, sem formatadores %d, %f)
    // ==============================================================================

    // MACROS DE DEBUG PADRÃO (Sem cor)
    #define printDebug(...)    do { Serial.print(__VA_ARGS__); } while(0)
    #define printlnDebug(...)  do { Serial.println(__VA_ARGS__); } while(0)
    #define printfDebug(...)   do { Serial.printf(__VA_ARGS__); } while(0)

    #define printReset(...)   do { Serial.print(RESET); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printBlack(...)   do { Serial.print(BLACK); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printRed(...)     do { Serial.print(RED); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printGreen(...)   do { Serial.print(GREEN); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printYellow(...)  do { Serial.print(YELLOW); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printBlue(...)    do { Serial.print(BLUE); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printCyan(...)    do { Serial.print(CYAN); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printMagenta(...) do { Serial.print(MAGENTA); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printWhite(...)   do { Serial.print(WHITE); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)

    #define printLBlack(...)    do { Serial.print(LBLACK); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLRed(...)    do { Serial.print(LRED); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLGreen(...)  do { Serial.print(LGREEN); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLYellow(...) do { Serial.print(LYELLOW); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLBlue(...)   do { Serial.print(LBLUE); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLMagenta(...) do { Serial.print(LMAGENTA); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLCyan(...)   do { Serial.print(LCYAN); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLWhite(...)   do { Serial.print(LWHITE); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)

    // ==============================================================================
    // MACROS PARA PRINTLN (Texto Simples com Quebra de Linha)
    // ==============================================================================

    #define printlnReset(...)   do { Serial.print(RESET); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnBlack(...)   do { Serial.print(BLACK); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnRed(...)     do { Serial.print(RED); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnGreen(...)   do { Serial.print(GREEN); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnYellow(...)  do { Serial.print(YELLOW); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnBlue(...)    do { Serial.print(BLUE); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnCyan(...)    do { Serial.print(CYAN); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnMagenta(...) do { Serial.print(MAGENTA); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnWhite(...)   do { Serial.print(WHITE); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)

    #define printlnLBlack(...)   do { Serial.print(LBLACK); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLRed(...)    do { Serial.print(LRED); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLGreen(...)  do { Serial.print(LGREEN); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLYellow(...) do { Serial.print(LYELLOW); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLBlue(...)   do { Serial.print(LBLUE); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLCyan(...)   do { Serial.print(LCYAN); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLMagenta(...) do { Serial.print(LMAGENTA); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLWhite(...)   do { Serial.print(LWHITE); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)

    #if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_STM32)
      // ==============================================================================
      // MACROS PARA PRINTF (Texto Formatado ex: %d, %.2f)
      // Requer suporte de arquitetura 32 bits (ESP32, ESP8266, STM32)
      // Uso: printfYellow("Velocidade: %.2f km/h", velH);
      // ==============================================================================

      #define printfReset(fmt, ...)   do { Serial.printf(RESET fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfBlack(fmt, ...)   do { Serial.printf(BLACK fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfRed(fmt, ...)     do { Serial.printf(RED fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfGreen(fmt, ...)   do { Serial.printf(GREEN fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfYellow(fmt, ...)  do { Serial.printf(YELLOW fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfBlue(fmt, ...)    do { Serial.printf(BLUE fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfMagenta(fmt, ...) do { Serial.printf(MAGENTA fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfCyan(fmt, ...)    do { Serial.printf(CYAN fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfWhite(fmt, ...)   do { Serial.printf(WHITE fmt RESET, ##__VA_ARGS__); } while(0)

      #define printfLBlack(fmt, ...)   do { Serial.printf(LBLACK fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfLRed(fmt, ...)    do { Serial.printf(LRED fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfLGreen(fmt, ...)  do { Serial.printf(LGREEN fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfLYellow(fmt, ...) do { Serial.printf(LYELLOW fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfLBlue(fmt, ...)   do { Serial.printf(LBLUE fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfLMagenta(fmt, ...) do { Serial.printf(LMAGENTA fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfLCyan(fmt, ...)   do { Serial.printf(LCYAN fmt RESET, ##__VA_ARGS__); } while(0)
      #define printfLWhite(fmt, ...)   do { Serial.printf(LWHITE fmt RESET, ##__VA_ARGS__); } while(0)


      #define printflnReset(fmt, ...)   do { Serial.printf(RESET fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnBlack(fmt, ...)   do { Serial.printf(BLACK fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnRed(fmt, ...)     do { Serial.printf(RED fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnGreen(fmt, ...)   do { Serial.printf(GREEN fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnYellow(fmt, ...)  do { Serial.printf(YELLOW fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnBlue(fmt, ...)    do { Serial.printf(BLUE fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnMagenta(fmt, ...) do { Serial.printf(MAGENTA fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnCyan(fmt, ...)    do { Serial.printf(CYAN fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnWhite(fmt, ...)   do { Serial.printf(WHITE fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)

      #define printflnLBlack(fmt, ...)   do { Serial.printf(LBLACK fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnLRed(fmt, ...)    do { Serial.printf(LRED fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnLGreen(fmt, ...)  do { Serial.printf(LGREEN fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnLYellow(fmt, ...) do { Serial.printf(LYELLOW fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnLBlue(fmt, ...)   do { Serial.printf(LBLUE fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnLMagenta(fmt, ...) do { Serial.printf(LMAGENTA fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnLCyan(fmt, ...)   do { Serial.printf(LCYAN fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)
      #define printflnLWhite(fmt, ...)   do { Serial.printf(LWHITE fmt RESET, ##__VA_ARGS__);  Serial.println(RESET); } while(0)

    #else
      #pragma message("Aviso: Macros de cor printf/printfln desativadas (Arquitetura não suporta Serial.printf nativo). Use print/println.")
    #endif
  
  #else
    #pragma message("Em MODO_VOO: prints via USB desativados.")
    // ==============================================================================
    // [ MODO VOO ] - O compilador substitui as funções por NADA (Zero overhead)
    // ==============================================================================

    // MACROS DE DEBUG PADRÃO (Sem cor)
    #define printDebug(...)    do { } while(0)
    #define printlnDebug(...)  do { } while(0)
    #define printfDebug(...)   do { } while(0)

    #define printReset(...)   do { } while(0)
    #define printBlack(...)   do { } while(0)
    #define printRed(...)     do { } while(0)
    #define printGreen(...)   do { } while(0)
    #define printYellow(...)  do { } while(0)
    #define printBlue(...)    do { } while(0)
    #define printCyan(...)    do { } while(0)
    #define printMagenta(...) do { } while(0)
    #define printWhite(...)   do { } while(0)

    #define printLBlack(...)    do { } while(0)
    #define printLRed(...)    do { } while(0)
    #define printLGreen(...)  do { } while(0)
    #define printLYellow(...) do { } while(0)
    #define printLBlue(...)   do { } while(0)
    #define printLMagenta(...) do { } while(0)
    #define printLCyan(...)   do { } while(0)
    #define printLWhite(...)   do { } while(0)

    // ==============================================================================
    // MACROS PARA PRINTLN (Texto Simples com Quebra de Linha)
    // ==============================================================================

    #define printlnReset(...)   do { } while(0)
    #define printlnBlack(...)   do { } while(0)
    #define printlnRed(...)     do { } while(0)
    #define printlnGreen(...)   do { } while(0)
    #define printlnYellow(...)  do { } while(0)
    #define printlnBlue(...)    do { } while(0)
    #define printlnCyan(...)    do { } while(0)
    #define printlnMagenta(...) do { } while(0)
    #define printlnWhite(...)   do { } while(0)

    #define printlnLBlack(...)   do { } while(0)
    #define printlnLRed(...)    do { } while(0)
    #define printlnLGreen(...)  do { } while(0)
    #define printlnLYellow(...) do { } while(0)
    #define printlnLBlue(...)   do { } while(0)
    #define printlnLCyan(...)   do { } while(0)
    #define printlnLMagenta(...) do { } while(0)
    #define printlnLWhite(...)   do { } while(0)


    #if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_STM32)
    // ==============================================================================
      #define printfRed(fmt, ...)     do {} while(0)
      #define printfGreen(fmt, ...)   do {} while(0)
      #define printfYellow(fmt, ...)  do {} while(0)
      #define printfBlue(fmt, ...)    do {} while(0)

      #define printfLRed(fmt, ...)    do {} while(0)
      #define printfLGreen(fmt, ...)  do {} while(0)
      #define printfLYellow(fmt, ...) do {} while(0)
      #define printfLBlue(fmt, ...)   do {} while(0)
      #define printfLCyan(fmt, ...)   do {} while(0)
      #define printfWhite(fmt, ...)   do {} while(0)

      // ==============================================================================
      #define printflnRed(fmt, ...)     do {} while(0)
      #define printflnGreen(fmt, ...)   do {} while(0)
      #define printflnYellow(fmt, ...)  do {} while(0)
      #define printflnBlue(fmt, ...)    do {} while(0)

      #define printflnLRed(fmt, ...)    do {} while(0)
      #define printflnLGreen(fmt, ...)  do {} while(0)
      #define printflnLYellow(fmt, ...) do {} while(0)
      #define printflnLBlue(fmt, ...)   do {} while(0)
      #define printflnLCyan(fmt, ...)   do {} while(0)
      #define printflnWhite(fmt, ...)   do {} while(0)
    #endif
	#endif
#endif /* ARDUINO */

//###########################################################################################################

// Para STM32, ESP32 etc, usando printf via USB CDC.
#if defined(USE_HALDRIVER) || defined(STM32F4xx) || defined(STM32F401xC) || defined(STM32F411xE) || defined(STM32F401xE)

	#define printDevID() do { printf("%x", HAL_GetDEVID()); } while(0)

	#ifndef EM_VOO
		#pragma message("Utilizando <stdio.h> do C para prints.")

		// MACROS DE DEBUG PADRÃO (Sem cor)
		#define printDebug(var, ...)    do { printf(var, ##__VA_ARGS__); } while(0)
		#define printlnDebug(var, ...)  do { printf(var "\r\n", ##__VA_ARGS__); } while(0)


			#define printReset(var, ...)   do { printf(RESET var RESET, ##__VA_ARGS__); } while(0)
			#define printBlack(var, ...)   do { printf(BLACK var RESET, ##__VA_ARGS__); } while(0)
			#define printRed(var, ...)     do { printf(RED var RESET, ##__VA_ARGS__); } while(0)
			#define printGreen(var, ...)   do { printf(GREEN var RESET, ##__VA_ARGS__); } while(0)
			#define printYellow(var, ...)  do { printf(YELLOW var RESET, ##__VA_ARGS__); } while(0)
			#define printBlue(var, ...)    do { printf(BLUE var RESET, ##__VA_ARGS__); } while(0)
			#define printMagenta(var, ...) do { printf(MAGENTA var RESET, ##__VA_ARGS__); } while(0)
			#define printCyan(var, ...)    do { printf(CYAN var RESET, ##__VA_ARGS__); } while(0)
			#define printWhite(var, ...)   do { printf(WHITE var RESET, ##__VA_ARGS__); } while(0)
			#define printLBlack(var, ...)  do { printf(LBLACK var RESET, ##__VA_ARGS__); } while(0)
			#define printLRed(var, ...)    do { printf(LRED var RESET, ##__VA_ARGS__); } while(0)
			#define printLGreen(var, ...)  do { printf(LGREEN var RESET, ##__VA_ARGS__); } while(0)
			#define printLYellow(var, ...) do { printf(LYELLOW var RESET, ##__VA_ARGS__); } while(0)
			#define printLBlue(var, ...)   do { printf(LBLUE var RESET, ##__VA_ARGS__); } while(0)
			#define printLMagenta(var, ...) do { printf(LMAGENTA var RESET, ##__VA_ARGS__); } while(0)
			#define printLCyan(var, ...)    do { printf(LCYAN var RESET, ##__VA_ARGS__); } while(0)
			#define printLWhite(var, ...)   do { printf(LWHITE var RESET, ##__VA_ARGS__); } while(0)


			#define printlnReset(var, ...)  do { printf(RESET var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnBlack(var, ...)  do { printf(BLACK var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnRed(var, ...)    do { printf(RED var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnGreen(var, ...)  do { printf(GREEN var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnYellow(var, ...) do { printf(YELLOW var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnBlue(var, ...)   do { printf(BLUE var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnMagenta(var, ...) do { printf(MAGENTA var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnCyan(var, ...)    do { printf(CYAN var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnWhite(var, ...)   do { printf(WHITE var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnLBlack(var, ...)  do { printf(LBLACK var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnLRed(var, ...)    do { printf(LRED var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnLGreen(var, ...)   do { printf(LGREEN var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnLYellow(var, ...)  do { printf(LYELLOW var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnLBlue(var, ...)   do { printf(LBLUE var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnLMagenta(var, ...) do { printf(LMAGENTA var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnLCyan(var, ...)    do { printf(LCYAN var RESET "\r\n", ##__VA_ARGS__); } while(0)
			#define printlnLWhite(var, ...)   do { printf(LWHITE var RESET "\r\n", ##__VA_ARGS__); } while(0)
	#else
		#pragma message("Em MODO_VOO: prints via USB desativados.")
		// ==============================================================================
		// [ MODO VOO ] - O compilador substitui as funções por NADA (Zero overhead)
		// ==============================================================================

	#define printDebug(...)    do {} while(0)
		#define printlnDebug(...)  do {} while(0)
		#define printfDebug(...)   do {} while(0)

	//================================================================================

	#define printReset(...)   do {} while(0)
	#define printBlack(...)   do {} while(0)
		#define printRed(...)     do {} while(0)
		#define printGreen(...)   do {} while(0)
		#define printYellow(...)  do {} while(0)
		#define printBlue(...)    do {} while(0)
		#define printCyan(...)    do {} while(0)
	#define printMagenta(...) do {} while(0)
	#define printWhite(...)   do {} while(0)

	#define printLBlack(...)    do {} while(0)
		#define printLRed(...)    do {} while(0)
		#define printLGreen(...)  do {} while(0)
		#define printLYellow(...) do {} while(0)
		#define printLBlue(...)   do {} while(0)
	#define printLMagenta(...)    do {} while(0)
		#define printLCyan(...)    do {} while(0)
		#define printLWhite(...)    do {} while(0)

		// ==============================================================================

	#define printlnReset(...)  do {} while(0)
	#define printlnBlack(...)  do {} while(0)
	#define printlnRed(...)     do {} while(0)
		#define printlnGreen(...)   do {} while(0)
		#define printlnYellow(...)  do {} while(0)
		#define printlnBlue(...)    do {} while(0)
	#define printlnCyan(...)    do {} while(0)
	#define printlnMagenta(...) do {} while(0)
	#define printlnWhite(...)   do {} while(0)

	#define printlnLBlack(...)    do {} while(0)
		#define printlnLRed(...)    do {} while(0)
		#define printlnLGreen(...)  do {} while(0)
		#define printlnLYellow(...) do {} while(0)
		#define printlnLBlue(...)   do {} while(0)
		#define printlnLMagenta(...)    do {} while(0)
		#define printlnLCyan(...)    do {} while(0)
		#define printlnLWhite(...)    do {} while(0)

  #endif
#endif /* STM32/ESP32 */



#ifdef __cplusplus
}
#endif

#endif /* PRINTS_H */
