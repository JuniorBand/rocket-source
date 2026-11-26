/*
  ******************************************************************************
  * @file    main.cpp
  * @brief   Ponto de Entrada Principal do Computador de Voo (STM32F411)
  ******************************************************************************
*/

#include <Arduino.h>
#include <IWatchdog.h>
#include <HardwareTimer.h>

#include <macros_config.h>
#include <utils/utils.h>
#include <config_voo.h>
#include <comunicacao/usb_com.h>
#include <comunicacao/telemetria_com.h>

bool watchdogStarted = false;
static HardwareTimer timerVoo(TIM2);

void feedWatchdog(void);
inline static void onTimerTickVoo(void);
inline static void flushUSB(void);

void setup(void) {
    // 1. Verifica imediatamente se o boot foi causado pelo Watchdog
    const bool watchdogReset = IWatchdog.isReset(true);

    // 2. Inicia USB CDC e aguarda o terminal abrir (em bancada espera até 4s)
    Serial.begin(115200);
    if (!watchdogReset) {
    #ifdef EM_VOO
        const u32 esperaUsbMs = USB_WAIT_MS;
    #else
        const u32 esperaUsbMs = 2000UL;
    #endif
        const u32 t0 = millis();
        while (!Serial && (millis() - t0) < esperaUsbMs) {
            delay(10);
        }
    }

    printlnLCyan("\r\n[BOOT 0/7] Interface USB CDC inicializada.");
    if (watchdogReset) {
        printlnLYellow("[BOOT 0/7] AVISO: Reset anterior causado pelo Independent Watchdog!");
    }
    flushUSB();

    // 3. Ativa o Independent Watchdog (4 segundos)
    IWatchdog.begin(WATCHDOG_TIMEOUT_US);
    watchdogStarted = true;
    feedWatchdog();
    printlnLGreen("[BOOT 1/7] Independent Watchdog ativado (4000 ms).");
    flushUSB();

    // 4. Inicializa UART2 de telemetria
    telemetriaInit();
    flushUSB();

    // 5. Inicializa pinos, sensores, Flash e máquina de estados
    setupVoo(watchdogReset);
    flushUSB();

    // 6. Configura metrônomo determinístico de 20 Hz (50 ms) no TIM2
    printlnLCyan("[BOOT 7/7] Configurando timer de voo TIM2 (20 Hz / 50 ms)...");
    flushUSB();
    timerVoo.setOverflow(SAMPLE_INTERVAL_MS * 1000UL, MICROSEC_FORMAT);
    timerVoo.attachInterrupt(onTimerTickVoo);
    timerVoo.resume();
    printlnLGreen("[BOOT 7/7] Timer TIM2 ativo. Entrando no loop principal.");
    flushUSB();

    usbComImprimirBanner();
    flushUSB();
}

void loop(void) {
    feedWatchdog();

    #ifndef EM_VOO
        // Em bancada: se você abrir o Serial Monitor depois do boot, reimprime o banner e o status
        static bool usbEstavaConectada = false;
        const bool usbConectadaAgora = (bool)Serial;
        if (usbConectadaAgora && !usbEstavaConectada) {
            delay(50);
            usbComImprimirBanner();
            usbComImprimirStatus();
            flushUSB();
        }
        usbEstavaConectada = usbConectadaAgora;
    #endif

    processarLogicaVoo();
    #ifdef RECEBER_COMANDOS_RADIO_TTL
        telemetriaReceber();
    #endif
    #ifdef ENVIAR_DADOS_CONTINUOS_RADIO_TTL
        telemetriaEnviar();
    #endif
    usbComProcessar();
    beeperUpdate();
}

void feedWatchdog(void) {
    if (watchdogStarted) {
        IWatchdog.reload();
    }
}

inline static void onTimerTickVoo(void) {
    flagTickVoo = 1;
}

inline static void flushUSB(void) {
    if (!Serial){ return; }
    const u32 t0 = millis();
    while (Serial.availableForWrite() < 63 && (millis() - t0) < 15UL) {
        feedWatchdog();
    }
}