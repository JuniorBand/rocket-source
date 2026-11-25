/*
  ******************************************************************************
  * @file    utils.h
  * @author  Júnior Bandeira
  * @brief   O utils é um header facilitador/portátil ainda em desenvolvimento.
  ******************************************************************************
*/

#ifndef UTILS_H
#define UTILS_H

#pragma message("Você está usando o utils.h para facilitar a sua vida ;)")

// Se MODO_VOO está comentado, assume-se: MODO_SOLO.
//#define MODO_VOO // Descomente para o compilador apagar qualquer print do código se em voo, economizando processamento e memória.

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

// Transforma de um u8 *ptr em um número 32-bit ou 16-bit, singned ou unsigned.
#define TRANSF_32_BIT(ptr, n) ((i32)ptr[n + 3] << 24 | (i32)ptr[n + 2] << 16 | (i32)ptr[n + 1] << 8 | ptr[n])
#define TRANSF_16_BIT(ptr, n)  ((i16)ptr[n + 1] << 8 | ptr[n])
#define TRANSF_U32_BIT(ptr, n) ((u32)ptr[n + 3] << 24 | (u32)ptr[n + 2] << 16 | (u32)ptr[n + 1] << 8 | ptr[n])
#define TRANSF_U16_BIT(ptr, n)  ((u16)ptr[n + 1] << 8 | ptr[n])

#if !defined(MIN) || !defined(MAX)
	#define MIN(a, b) (a < b) ? a : b
	#define MAX(a, b) (a < b) ? b : a
#endif

// Cores normais
#define RED     "\x1b[31m"
#define GREEN   "\x1b[32m"
#define YELLOW  "\x1b[33m"
#define BLUE    "\x1b[34m"
#define MAGENTA "\x1b[35m"
#define CYAN    "\x1b[36m"

// Cores claras (Bright / Light)
#define L_RED     "\x1b[91m"
#define L_GREEN   "\x1b[92m"
#define L_YELLOW  "\x1b[93m"
#define L_BLUE    "\x1b[94m"
#define L_MAGENTA "\x1b[95m"
#define L_CYAN    "\x1b[96m"
#define WHITE     "\x1b[97m"

#define RESET   "\x1b[0m" // Para resetar a cor

#ifdef ARDUINO // Arduino.h

  #ifndef MODO_VOO
    #pragma message("Em MODO_SOLO: prints via USB ativados.")
    // ==============================================================================
    // MACROS PARA PRINT (Texto Simples, sem formatadores %d, %f)
    // ==============================================================================
    #define printRed(...)     do { Serial.print(RED); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printGreen(...)   do { Serial.print(GREEN); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printYellow(...)  do { Serial.print(YELLOW); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printBlue(...)    do { Serial.print(BLUE); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    
    #define printLRed(...)    do { Serial.print(L_RED); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLGreen(...)  do { Serial.print(L_GREEN); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLYellow(...) do { Serial.print(L_YELLOW); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLBlue(...)   do { Serial.print(L_BLUE); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printLCyan(...)   do { Serial.print(L_CYAN); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)
    #define printWhite(...)   do { Serial.print(WHITE); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)

    #define printReset(...)   do { Serial.print(RESET); Serial.print(__VA_ARGS__); Serial.print(RESET); } while(0)

    // ==============================================================================
    // MACROS PARA PRINTLN (Texto Simples com Quebra de Linha)
    // ==============================================================================
    #define printlnRed(...)     do { Serial.print(RED); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnGreen(...)   do { Serial.print(GREEN); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnYellow(...)  do { Serial.print(YELLOW); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnBlue(...)    do { Serial.print(BLUE); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)

    #define printlnLRed(...)    do { Serial.print(L_RED); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLGreen(...)  do { Serial.print(L_GREEN); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLYellow(...) do { Serial.print(L_YELLOW); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLBlue(...)   do { Serial.print(L_BLUE); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnLCyan(...)   do { Serial.print(L_CYAN); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)
    #define printlnWhite(...)   do { Serial.print(WHITE); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)

    #define printlnReset(...)   do { Serial.print(RESET); Serial.print(__VA_ARGS__); Serial.println(RESET); } while(0)

    // ==============================================================================
    // MACROS DE DEBUG PADRÃO (Sem cor)
    // ==============================================================================
    #define printDebug(...)    do { Serial.print(__VA_ARGS__); } while(0)
    #define printlnDebug(...)  do { Serial.println(__VA_ARGS__); } while(0)
    #define printfDebug(...)   do { Serial.printf(__VA_ARGS__); } while(0)

    #if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_STM32)
      // ==============================================================================
      // MACROS PARA PRINTF (Texto Formatado ex: %d, %.2f)
      // Requer suporte de arquitetura 32 bits (ESP32, ESP8266, STM32)
      // Uso: printfYellow("Velocidade: %.2f km/h", velH);
      // ==============================================================================
      #define printfRed(fmt, ...)     Serial.printf(RED fmt RESET, ##__VA_ARGS__)
      #define printfGreen(fmt, ...)   Serial.printf(GREEN fmt RESET, ##__VA_ARGS__)
      #define printfYellow(fmt, ...)  Serial.printf(YELLOW fmt RESET, ##__VA_ARGS__)
      #define printfBlue(fmt, ...)    Serial.printf(BLUE fmt RESET, ##__VA_ARGS__)

      #define printfLRed(fmt, ...)    Serial.printf(L_RED fmt RESET, ##__VA_ARGS__)
      #define printfLGreen(fmt, ...)  Serial.printf(L_GREEN fmt RESET, ##__VA_ARGS__)
      #define printfLYellow(fmt, ...) Serial.printf(L_YELLOW fmt RESET, ##__VA_ARGS__)
      #define printfLBlue(fmt, ...)   Serial.printf(L_BLUE fmt RESET, ##__VA_ARGS__)
      #define printfLCyan(fmt, ...)   Serial.printf(L_CYAN fmt RESET, ##__VA_ARGS__)
      #define printfWhite(fmt, ...)   Serial.printf(WHITE fmt RESET, ##__VA_ARGS__)

      // ==============================================================================
      // MACROS PARA PRINTF COM QUEBRA DE LINHA NO FINAL ("\r\n" automático)
      // Uso: printflnLYellow("Latitude: %.7f", lat);
      // ==============================================================================
      #define printflnRed(fmt, ...)     do { Serial.printf(RED fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
      #define printflnGreen(fmt, ...)   do { Serial.printf(GREEN fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
      #define printflnYellow(fmt, ...)  do { Serial.printf(YELLOW fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
      #define printflnBlue(fmt, ...)    do { Serial.printf(BLUE fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)

      #define printflnLRed(fmt, ...)    do { Serial.printf(L_RED fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
      #define printflnLGreen(fmt, ...)  do { Serial.printf(L_GREEN fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
      #define printflnLYellow(fmt, ...) do { Serial.printf(L_YELLOW fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
      #define printflnLBlue(fmt, ...)   do { Serial.printf(L_BLUE fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
      #define printflnLCyan(fmt, ...)   do { Serial.printf(L_CYAN fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
      #define printflnWhite(fmt, ...)   do { Serial.printf(WHITE fmt, ##__VA_ARGS__); Serial.println(RESET); } while(0)
    #else
      #pragma message("Aviso: Macros de cor printf/printfln desativadas (Arquitetura não suporta Serial.printf nativo). Use print/println.")
    #endif
  #else
    #pragma message("Em MODO_VOO: prints via USB desativados.")
    // ==============================================================================
    // [ MODO VOO ] - O compilador substitui as funções por NADA (Zero overhead)
    // ==============================================================================
    #define printRed(...)     do {} while(0)
    #define printGreen(...)   do {} while(0)
    #define printYellow(...)  do {} while(0)
    #define printBlue(...)    do {} while(0)
    
    #define printLRed(...)    do {} while(0)
    #define printLGreen(...)  do {} while(0)
    #define printLYellow(...) do {} while(0)
    #define printLBlue(...)   do {} while(0)
    #define printLCyan(...)   do {} while(0)
    #define printWhite(...)   do {} while(0)

    #define printReset(...)   do {} while(0)

    // ==============================================================================
    #define printlnRed(...)     do {} while(0)
    #define printlnGreen(...)   do {} while(0)
    #define printlnYellow(...)  do {} while(0)
    #define printlnBlue(...)    do {} while(0)

    #define printlnLRed(...)    do {} while(0)
    #define printlnLGreen(...)  do {} while(0)
    #define printlnLYellow(...) do {} while(0)
    #define printlnLBlue(...)   do {} while(0)
    #define printlnLCyan(...)   do {} while(0)
    #define printlnWhite(...)   do {} while(0)

    #define printlnReset(...)  do {} while(0)

    //================================================================================

    #define printDebug(...)    do {} while(0)
    #define printlnDebug(...)  do {} while(0)
    #define printfDebug(...)   do {} while(0)


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
    #else
      #pragma message("Aviso: Macros de cor printf/printfln desativadas (Arquitetura não suporta Serial.printf nativo). Use print/println.")
    #endif
  #endif
#else
  #error Essa versao do utils.h foi feito visando o Arduino.h!
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


#endif