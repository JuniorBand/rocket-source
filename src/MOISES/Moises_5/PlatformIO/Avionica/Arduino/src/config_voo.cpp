/*
  ******************************************************************************
  * @file    config_voo.cpp
  * @brief   Implementação da Máquina de Estados, Ejeção, Pré-Trigger e Recuperação
  ******************************************************************************
*/

#include <config_voo.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <IWatchdog.h>

const char* PRINT_ESTADO[] = {
    "[CALIBRACAO]",  // 0
    "[PRONTO]",      // 1
    "[VOO]",         // 2
    "[APOGEU]",      // 3
    "[RECUPERACAO]", // 4
    "[POUSO]",       // 5
    "[ERRO CRITICO]" // 6
};

SeguroVoo_t seguroVoo = {0};
DadosVoo_t  dadosVoo = {0};
BMP280_t bmpSensor = {0};
MPU6050_t mpuSensor = {0};
LKF_t filtroKalman = {0};

volatile u8 flagTickVoo = 0;
bool flagGravacaoParada = false;
bool flagFimDeVoo = false;
bool sistemaBloqueadoPorLog = false;
extern bool watchdogStarted;

static u8 pendingLogFlags = 0;
static u8 contadorFalhasSensor = 0;
static u8 launchConfirmCount = 0;
static u8 landingConfirmCount = 0;
static u32 launchCandidateTimeMs = 0;

#ifdef VERBOSE
    static u32  timerVerboseMs = 0;
#endif

static inline bool printsBloqueadosPorVoo(void) {
#ifdef EM_VOO
    return (dadosVoo.estadoAtual >= ESTADO_EM_VOO) &&
           (dadosVoo.estadoAtual <= ESTADO_RECUPERACAO);
#else // EM_SOLO sempre printa
    return false;
#endif
}

// ==============================================================================
// BUFFER CIRCULAR PRÉ-LANÇAMENTO NA RAM
// ==============================================================================
static DadosVoo_t preRing[PRETRIGGER_SAMPLES];
static u16 preRingHead = 0;
static u16 preRingCount = 0;

static inline void preRingClear(void) {
    preRingHead  = 0;
    preRingCount = 0;
}

static inline void preRingPush(const DadosVoo_t &sample) {
    preRing[preRingHead] = sample;
    preRingHead = (preRingHead + 1) % PRETRIGGER_SAMPLES;
    if (preRingCount < PRETRIGGER_SAMPLES) {
        ++preRingCount;
    }
}


// ==============================================================================
// GERENCIADOR DE BUZZER NÃO-BLOQUEANTE
// ==============================================================================
typedef struct {
    u16 onMs;
    u16 offMs;
} BeepStep_t;

static constexpr u8 BEEPER_MAX_STEPS = 16;
static BeepStep_t beeperSteps[BEEPER_MAX_STEPS];
static u8 beeperCount = 0;
static u8 beeperIndex = 0;
static bool beeperActive = false;
static bool beeperOn = false;
static u32 beeperNextMs = 0;

void beeperClear(void) {
    beeperActive = false;
    beeperOn     = false;
    beeperCount  = 0;
    beeperIndex  = 0;
    BUZZER_OFF();
}

void playBeeps(u8 count, u16 onMs, u16 offMs) {
    beeperClear();
    if (count == 0){ return; }
    if (count > BEEPER_MAX_STEPS){ count = BEEPER_MAX_STEPS; }

    for (u8 i = 0; i < count; ++i) {
        beeperSteps[i].onMs  = onMs;
        beeperSteps[i].offMs = (i + 1 < count) ? offMs : 0;
    }
    beeperCount = count;
    beeperIndex = 0;
    beeperActive = true;
    beeperOn = true;
    BUZZER_ON();
    beeperNextMs = millis() + beeperSteps[0].onMs;
}

void beeperUpdate(void) {
    if (!beeperActive){ return; }

    const u32 now = millis();
    if (!timeReached(now, beeperNextMs)){ return; }

    if (beeperOn) {
        BUZZER_OFF();
        beeperOn     = false;
        beeperNextMs = now + beeperSteps[beeperIndex].offMs;
    } else {
        ++beeperIndex;
        if (beeperIndex >= beeperCount) {
            beeperActive = false;
            return;
        }
        BUZZER_ON();
        beeperOn = true;
        beeperNextMs = now + beeperSteps[beeperIndex].onMs;
    }
}

void beepBlocking(u16 durationMs, u8 repetitions) {
    beeperClear();
    for (u8 i = 0; i < repetitions; ++i) {
        BUZZER_ON();
        delay(durationMs);
        BUZZER_OFF();
        if (i + 1 < repetitions){ delay(80); }
        feedWatchdog();
    }
}

// ==============================================================================
// ACIONAMENTO PIROTÉCNICO (J2 DEPLOY = PA0 | J1 MAIN = PB0)
// ==============================================================================
static void acionarDeploy(const char *motivo) {
    if (!seguroVoo.deployAcionado) {
        seguroVoo.deployAcionado = true;
        seguroVoo.deployTimerMs = millis();
        digitalWrite(PIN_PYRO_DEPLOY, HIGH);

        printlnMagenta("\r\n=================================================");
        printflnLCyan("[PYRO J2 - DEPLOY] Acionado a %.2f m | Motivo: %s", dadosVoo.altitudeM, motivo);
        printflnRed("                   Altitude Maxima: %.2f m", seguroVoo.altitudeMaxima);
        printlnMagenta("=================================================");
    }
}

static void acionarMain(const char *motivo) {
    if (!seguroVoo.mainAcionado) {
        seguroVoo.mainAcionado = true;
        seguroVoo.mainTimerMs  = millis();
        digitalWrite(PIN_PYRO_MAIN, HIGH);

        printlnMagenta("\r\n=================================================");
        printflnLCyan("[PYRO J1 - MAIN] Acionado a %.2f m | Motivo: %s", dadosVoo.altitudeM, motivo);
        printlnMagenta("=================================================");
    }
}

static void atualizarPulsosPirotecnicos(u32 nowMs) {
    if (seguroVoo.deployAcionado && digitalRead(PIN_PYRO_DEPLOY) == HIGH) {
        if ((nowMs - seguroVoo.deployTimerMs) >= PYRO_PULSE_DURATION_MS) {
            digitalWrite(PIN_PYRO_DEPLOY, LOW);
        }
    }
    if (seguroVoo.mainAcionado && digitalRead(PIN_PYRO_MAIN) == HIGH) {
        if ((nowMs - seguroVoo.mainTimerMs) >= PYRO_PULSE_DURATION_MS) {
            digitalWrite(PIN_PYRO_MAIN, LOW);
        }
    }
}

// ==============================================================================
// GRAVAÇÃO DE LOGS E TRANSIÇÕES DE VOO
// ==============================================================================
static void registrarLog(const DadosVoo_t &sample, i32 timestampRelativoMs, u8 extraFlags = 0) {
    if (flagGravacaoParada){ return; }

    LogData_t reg;
    memset(&reg, 0, sizeof(reg));

    reg.timestampMs = timestampRelativoMs;
    reg.pressaoPa = sample.pressaoPa;
    reg.altitudeM  = sample.altitudeM;
    reg.velocidadeMs = sample.velocidadeMs;
    reg.temperatura_cC = (i16)lroundf(sample.temperaturaC * 100.0f);

    for (u8 a = 0; a < 3; ++a) {
        reg.accelRaw[a] = sample.accelRaw[a];
        reg.gyroRaw[a]  = sample.gyroRaw[a];
    }

    u8 flags = extraFlags | pendingLogFlags;
    if (sample.imuStatus & MPU_STATUS_VALID) {
        flags |= LOG_FLAG_IMU_VALID;
    }
    reg.stateAndFlags = (u8)((sample.estadoAtual & 0x0F) | (flags & 0xF0));

    if (!w25qAdicionarLog(&reg)) {
        flagGravacaoParada = true;
        printlnLYellow("\r\n[AVISO] Memoria Flash cheia! Gravacao encerrada, voo continua.");
    } else {
        pendingLogFlags = 0;
    }
}

static void iniciarVoo(u32 launchTimeMs) {
    if (!flagGravacaoParada) {
        w25qGravarHeader(bmpSensor.pressaoRefPa,
                         bmpSensor.temperaturaRefC,
                         mpuSensor.gyroBiasRaw,
                         mpuSensor.gyroCalibrated);
        w25qSetStatusVoo(MAGIC_NUMBER_SIRIUS);
    }

    seguroVoo.tempoInicioMs = launchTimeMs;
    seguroVoo.altitudeMaxima = dadosVoo.altitudeM;
    dadosVoo.estadoAtual = ESTADO_EM_VOO;

    // Descarrega o histórico pré-lançamento armazenado na RAM
    for (u16 i = 0; i < preRingCount; ++i) {
        const u16 idx = (preRingHead + PRETRIGGER_SAMPLES - preRingCount + i) % PRETRIGGER_SAMPLES;
        DadosVoo_t hist = preRing[idx];
        const i32 tsRel = (i32)(hist.uptimeMs - launchTimeMs);

        u8 flags = (tsRel < 0) ? LOG_FLAG_PRETRIGGER : 0;
        if (hist.uptimeMs == launchTimeMs) {
            flags |= LOG_FLAG_LAUNCH;
            hist.estadoAtual = ESTADO_EM_VOO;
        }
        registrarLog(hist, tsRel, flags);
    }
    preRingClear();

    printflnLCyan("\r\n>>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_EM_VOO]);
    playBeeps(3, 100, 100);
}

static void encerrarVooComSeguranca(EstadoSistema estadoFinal, const char *mensagem) {
    if (!flagFimDeVoo) {
        dadosVoo.estadoAtual = estadoFinal;
        const i32 ts = (i32)(millis() - seguroVoo.tempoInicioMs);
        registrarLog(dadosVoo, ts);

        w25qDescarregarBuffer();
        w25qSetStatusVoo(0x00000000); // Desativa retomada automática no próximo boot

        flagGravacaoParada = true;
        flagFimDeVoo = true;
        sistemaBloqueadoPorLog = true;

        printlnLGreen("\r\n%s", mensagem);
        printflnLCyan("Altitude Maxima atingida: %.2f m | Registros salvos: %lu",
                      seguroVoo.altitudeMaxima, (unsigned long)w25qGetRecordCount());
    }
}

static void resgatarFoguete(u32 intervaloMs, u32 duracaoBeepMs, u32 *ultimoBeepMs) {
    const u32 now = millis();
    if ((now - *ultimoBeepMs) >= intervaloMs) {
        BUZZER_ON();
        *ultimoBeepMs = now;
    } else if ((now - *ultimoBeepMs) >= duracaoBeepMs) {
        BUZZER_OFF();
    }
}

static void verificarErros(void) {
    const u32 now = millis();
    const bool emVooAtivo = (seguroVoo.tempoInicioMs > 0UL) && !flagFimDeVoo;

    // 1. Falha contínua do barômetro (> 500 ms = 10 ticks sem leitura válida)
    if (contadorFalhasSensor > 10 && emVooAtivo && dadosVoo.estadoAtual != ESTADO_ERRO) {
        if (!seguroVoo.deployAcionado) {
            acionarDeploy("FALHA CRITICA DE COMUNICACAO DO BMP280");
        }
        forcarErroSistema("FALHA CRITICA DO BAROMETRO EM VOO");
    }

    // 2. Divergência numérica (NaN) no Filtro de Kalman
    if ((isnan(dadosVoo.altitudeM) || isnan(dadosVoo.velocidadeMs)) &&
        dadosVoo.estadoAtual != ESTADO_ERRO) {
        if (emVooAtivo && !seguroVoo.deployAcionado) {
            acionarDeploy("DIVERGENCIA LKF (NaN)");
        }
        forcarErroSistema("DIVERGENCIA NUMERICA NO FILTRO DE KALMAN");
    }

    // 3. Failsafe de paraquedas e pouso por tempo máximo (funciona INCLUSIVE em ESTADO_ERRO)
    if (emVooAtivo) {
        const u32 tempoVooMs = now - seguroVoo.tempoInicioMs;

        // Estágio 1 (Drogue - PA0): abre se estourar o tempo máximo de apogeu
        if (!seguroVoo.deployAcionado && tempoVooMs >= TEMPO_MAX_PARAQUEDAS_MS) {
            acionarDeploy("FAILSAFE: TEMPO MAXIMO DE APOGEU");
        }

        // Estágio 2 (Main - PB0): abre em tempo intermediário antes do fim do voo
        if (!seguroVoo.mainAcionado && tempoVooMs >= TEMPO_MAX_MAIN_MS) {
            acionarMain("FAILSAFE: TEMPO MAXIMO DE PARAQUEDAS PRINCIPAL");
        }

        // Encerramento: fecha o log e entra em modo resgate no limite total do voo
        if (tempoVooMs >= TEMPO_MAX_VOO_MS) {
            if (!seguroVoo.deployAcionado) {
                acionarDeploy("TIMEOUT MAXIMO DE VOO");
            }
            if (!seguroVoo.mainAcionado) {
                acionarMain("TIMEOUT MAXIMO DE VOO");
            }
            encerrarVooComSeguranca(ESTADO_POUSADO, "[AVISO] Tempo maximo de voo atingido. Log encerrado.");
        }
    }
    
}

// ==============================================================================
// INICIALIZAÇÃO E REARMAMENTO
// ==============================================================================
static void flushSerialBoot(void) {
    if (Serial) {
        Serial.flush();
    }
}

bool recalibrarERearmarSistema(void) {
    digitalWrite(PIN_PYRO_DEPLOY, LOW);
    digitalWrite(PIN_PYRO_MAIN, LOW);

    memset(&seguroVoo, 0, sizeof(seguroVoo));
    preRingClear();
    launchConfirmCount = 0;
    landingConfirmCount = 0;
    contadorFalhasSensor = 0;
    pendingLogFlags = 0;
    flagFimDeVoo = false;
    sistemaBloqueadoPorLog = false;

#ifdef GRAVAR_LOGS
    flagGravacaoParada = false;
#else
    flagGravacaoParada = true;
#endif

    dadosVoo.estadoAtual = ESTADO_CALIBRACAO;
    acenderLedPlaca();

    printlnLCyan("[CALIB 1/3] Calibrando pressao de referencia do BMP280...");
    flushSerialBoot();
    if (!bmp280CalibrateBase(&bmpSensor)) {
        printlnRed("[CALIB 1/3] FALHA na calibracao do BMP280!");
        flushSerialBoot();
        apagarLedPlaca();
        return false;
    }
    printflnLGreen("[CALIB 1/3] OK | P_ref = %.2f Pa | T_ref = %.2f C",
                   bmpSensor.pressaoRefPa, bmpSensor.temperaturaRefC);
    flushSerialBoot();

    if (mpuSensor.online) {
        printlnLCyan("[CALIB 2/3] Mantenha a placa imovel: calibrando giroscopio MPU6050...");
        flushSerialBoot();
        if (mpu6050CalibrateGyro(&mpuSensor)) {
            printflnLGreen("[CALIB 2/3] OK | Bias Gyro = [%d, %d, %d]",
                           mpuSensor.gyroBiasRaw[0],
                           mpuSensor.gyroBiasRaw[1],
                           mpuSensor.gyroBiasRaw[2]);
        } else {
            printlnLYellow("[CALIB 2/3] AVISO: calibracao rejeitada por movimento; dados brutos mantidos.");
        }
    } else {
        printlnLYellow("[CALIB 2/3] MPU6050 offline - calibracao de giro ignorada.");
    }
    flushSerialBoot();

    printlnLCyan("[CALIB 3/3] Zerando estado do Filtro de Kalman (Alt=0.0m, Vel=0.0m/s)...");
    dadosVoo.altitudeM = 0.0f;
    dadosVoo.velocidadeMs = 0.0f;
    filtroKalman.reiniciar(&filtroKalman, 0.0f, &dadosVoo.velocidadeMs);
    printlnLGreen("[CALIB 3/3] OK | Filtro de Kalman pronto.");
    flushSerialBoot();

    dadosVoo.estadoAtual = ESTADO_PRONTO;
    apagarLedPlaca();

    printflnLGreen("\r\n>>> %s - AGUARDANDO LANCAMENTO <<<", PRINT_ESTADO[ESTADO_PRONTO]);
    flushSerialBoot();
    playBeeps(3, 100, 80);
    return true;
}

void forcarErroSistema(const char *motivo) {
    if (dadosVoo.estadoAtual == ESTADO_ERRO) {
        return;
    }

    dadosVoo.estadoAtual = ESTADO_ERRO;

    printlnRed("\r\n=================================================");
    printflnRed("   >>> %s: %s <<<", PRINT_ESTADO[ESTADO_ERRO], motivo);
    printlnRed("=================================================");

    // Se o foguete já decolou, mantém seguroVoo.tempoInicioMs e flagFimDeVoo intactos
    // para que o failsafe de tempo acione o main e encerre o voo no pouso.
    if (seguroVoo.tempoInicioMs > 0UL && !flagFimDeVoo) {
        const i32 ts = (i32)(millis() - seguroVoo.tempoInicioMs);
        registrarLog(dadosVoo, ts);
        w25qDescarregarBuffer();
    } else {
        flagGravacaoParada = true;
    }
}

void setupVoo(bool watchdogReset) {
    // ETAPA 2/7: GPIOs, Buzzer, LED e Pirotécnicos
    printlnLCyan("[BOOT 2/7] Configurando GPIOs, LED, Buzzer e canais pirotecnicos (LOW)...");
    pinMode(PIN_BUZZER, OUTPUT);
    BUZZER_OFF();
    pinMode(PIN_LED_ONBOARD, OUTPUT);
    apagarLedPlaca();

    pinMode(PIN_PYRO_MAIN, OUTPUT);
    digitalWrite(PIN_PYRO_MAIN, LOW);
    pinMode(PIN_PYRO_DEPLOY, OUTPUT);
    digitalWrite(PIN_PYRO_DEPLOY, LOW);
    printlnLGreen("[BOOT 2/7] OK | Pinos seguros inicializados.");
    flushSerialBoot();
    feedWatchdog();

    // ETAPA 3/7: Filtro de Kalman Linear
    printlnLCyan("[BOOT 3/7] Inicializando Filtro de Kalman Linear (LKF)...");
    LKF_Construtor(&filtroKalman);
    filtroKalman.init(&filtroKalman, 0.0f, &dadosVoo.velocidadeMs);
    printlnLGreen("[BOOT 3/7] OK | LKF inicializado.");
    flushSerialBoot();
    feedWatchdog();

    // ETAPA 4/7: Memória Flash W25Q64
    printlnLCyan("[BOOT 4/7] Inicializando memoria Flash SPI W25Q64...");
    flushSerialBoot();
    const bool flashOk = w25qInit();
    if (flashOk) {
        printflnLGreen("[BOOT 4/7] OK | Flash detectada (%lu KB).",
                       (unsigned long)(w25qGetCapacity() / 1024));
    } else {
        printlnRed("[BOOT 4/7] FALHA | Memoria Flash W25Q64 nao respondeu!");
    }
    flushSerialBoot();
    feedWatchdog();

    // ETAPA 5/7: Barômetro BMP280
    printlnLCyan("[BOOT 5/7] Inicializando barometro BMP280 (SPI3)...");
    flushSerialBoot();
    const bool bmpOk = bmp280Init(&bmpSensor);
    if (bmpOk) {
        printlnLGreen("[BOOT 5/7] OK | BMP280 online.");
    } else {
        printlnRed("[BOOT 5/7] FALHA | Barometro BMP280 nao respondeu!");
    }
    flushSerialBoot();
    feedWatchdog();

    // ETAPA 6/7: IMU MPU6050
    printlnLCyan("[BOOT 6/7] Inicializando IMU MPU6050 (I2C software)...");
    flushSerialBoot();
    mpu6050Init(&mpuSensor);
    if (mpuSensor.online) {
        printlnLGreen("[BOOT 6/7] OK | MPU6050 online.");
    } else {
        printlnLYellow("[BOOT 6/7] AVISO | MPU6050 offline (voo continua apenas com barometro).");
    }
    flushSerialBoot();
    feedWatchdog();

    // Verificação de hardware
    if (!bmpOk) {
        dadosVoo.estadoAtual = ESTADO_ERRO;
        flagGravacaoParada   = true;
        printlnRed("\r\n[ERRO CRITICO] Barometro BMP280 ausente. Entrando em ESTADO_ERRO.");
        flushSerialBoot();
        beepBlocking(500, 3);
        return;
    }

    if (!flashOk) {
        flagGravacaoParada = true;
        printlnLYellow("\r\n[AVISO] Flash W25Q offline! Caixa-preta desativada; voo e paraquedas seguem ativos.");
        flushSerialBoot();
        if (!recalibrarERearmarSistema()) {
            dadosVoo.estadoAtual = ESTADO_ERRO;
            beepBlocking(500, 3);
        } else {
            // Alerta sonoro não-bloqueante diferenciando boot armado sem Flash (4 bipes)
            playBeeps(4, 100, 100);
        }
        return;
    }

    // Inspeção da Caixa-Preta (Setor 0)
    printlnLCyan("[BOOT 6.5/7] Verificando integridade da Caixa-Preta (Setor 0)...");
    flushSerialBoot();
    CaixaPretaInfo_t cp = w25qLerCaixaPreta();
    const bool deveRetomarVoo = watchdogReset &&
                                cp.headerValido &&
                                (cp.statusVoo == MAGIC_NUMBER_SIRIUS) &&
                                !w25qIsFull();

    if (deveRetomarVoo) {
        printlnLYellow("\r\n>>> AVISO: REINICIO EM VOO DETECTADO! RETOMANDO GRAVACAO... <<<");
        flushSerialBoot();

        if (cp.header.pressaoRefPa > PRESSURE_MIN_PA && cp.header.pressaoRefPa < PRESSURE_MAX_PA) {
            bmpSensor.pressaoRefPa = cp.header.pressaoRefPa;
        } else {
            bmpSensor.pressaoRefPa = 101325.0f;
        }

        for (u8 a = 0; a < 3; ++a) mpuSensor.gyroBiasRaw[a] = cp.header.gyroBiasRaw[a];
        mpuSensor.gyroCalibrated = (cp.header.mpuCalibrated != 0);

        for (u8 i = 0; i < 8; ++i) {
            bmp280Read(&bmpSensor);
            delay(10);
            feedWatchdog();
        }

        dadosVoo.altitudeM    = isfinite(bmpSensor.altitudeM) ? bmpSensor.altitudeM : 0.0f;
        dadosVoo.velocidadeMs = 0.0f;
        filtroKalman.reiniciar(&filtroKalman, dadosVoo.altitudeM, &dadosVoo.velocidadeMs);

        i32 ultimoTs = 0;
        const u32 totalRegs = w25qGetRecordCount();
        LogData_t ultimoReg;
        if (totalRegs > 0 && w25qLerRegistro(totalRegs - 1, &ultimoReg)) {
            ultimoTs = ultimoReg.timestampMs;
        }

        seguroVoo.tempoInicioMs = millis() - (u32)(ultimoTs + SAMPLE_INTERVAL_MS);
        seguroVoo.altitudeMaxima = dadosVoo.altitudeM;
        pendingLogFlags = LOG_FLAG_RESUMED;
        dadosVoo.estadoAtual = ESTADO_EM_VOO;

        #ifndef GRAVAR_LOGS
        flagGravacaoParada = true;
        #endif

        printlnLGreen("[RETOMADA] Voo retomado com sucesso.");
        flushSerialBoot();
        return;
    }

    // Se ele nao foi retomado (em voo), ja pousou
    if (cp.headerValido || cp.dadosDesconhecidos || w25qGetRecordCount() > 0) {
        if (cp.headerValido) {
            bmpSensor.pressaoRefPa = cp.header.pressaoRefPa;
            bmpSensor.temperaturaRefC = cp.header.temperaturaRefC;
        } else {
            bmp280CalibrateBase(&bmpSensor);
        }

        sistemaBloqueadoPorLog = true;
        flagGravacaoParada = true;
        dadosVoo.estadoAtual = ESTADO_PRONTO;

        printlnLYellow("\r\n[AVISO] Log de voo anterior detectado na Flash!");
        printlnLYellow("Gravacao bloqueada. Envie 'V'/'C' para ler ou 'A' para apagar e armar.");
        flushSerialBoot();
        beepBlocking(150, 2);
        return;
    }

    printlnLCyan("[BOOT 6.5/7] Flash limpa. Iniciando calibracao automatica...");
    flushSerialBoot();
    if (!recalibrarERearmarSistema()) {
        dadosVoo.estadoAtual = ESTADO_ERRO;
        beepBlocking(500, 3);
    }
}

// ==============================================================================
// LOOP PRINCIPAL DA MÁQUINA DE ESTADOS (20 Hz)
// ==============================================================================

void processarLogicaVoo(void) {
    if (!flagTickVoo){ return; }
    flagTickVoo = 0;

    const u32 now = millis();
    atualizarPulsosPirotecnicos(now);

    // 1. Aquisição dos sensores
    const bool bmpOk = bmp280Read(&bmpSensor);
    mpu6050Acquire(&mpuSensor, now);

    if (!bmpOk) {
        if (contadorFalhasSensor < 255) {
            ++contadorFalhasSensor;
        }
        verificarErros();

        if (dadosVoo.estadoAtual == ESTADO_ERRO) {
            static u32 timerErroBeep = 0;
            resgatarFoguete(6000, 1000, &timerErroBeep);
        }
        return;
    }
    contadorFalhasSensor = 0;

    dadosVoo.uptimeMs = now;
    dadosVoo.pressaoPa = bmpSensor.pressaoPa;
    dadosVoo.temperaturaC = bmpSensor.temperaturaC;
    dadosVoo.imuStatus = mpuSensor.status;
    for (u8 a = 0; a < 3; ++a) {
        dadosVoo.accelRaw[a] = mpuSensor.accelRaw[a];
        dadosVoo.gyroRaw[a]  = mpuSensor.gyroRaw[a];
    }

    // 2. Filtragem de Estado (Kalman)
    #ifdef USE_FILTER
        filtroKalman.update(&filtroKalman, bmpSensor.altitudeM, DT,
                            &dadosVoo.altitudeM, &dadosVoo.velocidadeMs);
    #else
        dadosVoo.altitudeM    = bmpSensor.altitudeM;
        dadosVoo.velocidadeMs = 0.0f;
    #endif

    #ifdef VERBOSE
        if (((now - timerVerboseMs) >= 1000) && !printsBloqueadosPorVoo()) {
            printflnLCyan("[TELEM] %s | Alt: %.2f m | Vel: %.2f m/s | P: %.1f Pa | T: %.2f C",
                        PRINT_ESTADO[dadosVoo.estadoAtual],
                        dadosVoo.altitudeM, dadosVoo.velocidadeMs,
                        dadosVoo.pressaoPa, dadosVoo.temperaturaC);
            timerVerboseMs = now;
        }
    #endif

    verificarErros();

    // 3. Transições da Máquina de Estados
    switch (dadosVoo.estadoAtual) {
        case ESTADO_PRONTO: {
            if (sistemaBloqueadoPorLog){ break; }

            preRingPush(dadosVoo);

            if ((dadosVoo.altitudeM > ALTITUDE_LANCAMENTO) &&
                (dadosVoo.velocidadeMs > VELOCIDADE_LANCAMENTO)) {
                if (launchConfirmCount == 0) {
                    launchCandidateTimeMs = now;
                }
                ++launchConfirmCount;

                if (launchConfirmCount >= LAUNCH_CONFIRM_SAMPLES) {
                    iniciarVoo(launchCandidateTimeMs);
                }
            } else {
                launchConfirmCount = 0;
            }
            break;
        }

        case ESTADO_EM_VOO: {
            const i32 ts = (i32)(now - seguroVoo.tempoInicioMs);
            registrarLog(dadosVoo, ts);

            if (dadosVoo.altitudeM > seguroVoo.altitudeMaxima) {
                seguroVoo.altitudeMaxima = dadosVoo.altitudeM;
            }

            const bool atingiuMeta  = (dadosVoo.altitudeM >= (META_APOGEU - DESVIO_MIN));
            const bool iniciouQueda = (dadosVoo.altitudeM < (seguroVoo.altitudeMaxima - DESCIDA_MINIMA)) &&
                                      (seguroVoo.altitudeMaxima > ALTITUDE_MIN_EJECAO);
            const bool timeoutPara = ((now - seguroVoo.tempoInicioMs) >= TEMPO_MAX_PARAQUEDAS_MS);

            if (atingiuMeta || iniciouQueda || timeoutPara) {
                if (!atingiuMeta && !timeoutPara && !printsBloqueadosPorVoo()) {
                    printlnLRed("[ALERTA] Queda detectada antes da meta de apogeu!");
                } else if (timeoutPara && !printsBloqueadosPorVoo()) {
                    printlnLYellow("[ALERTA] Timeout de paraquedas atingido!");
                }
                dadosVoo.estadoAtual = ESTADO_APOGEU;
            }
            break;
        }

        case ESTADO_APOGEU: {
            const i32 ts = (i32)(now - seguroVoo.tempoInicioMs);
            registrarLog(dadosVoo, ts);

            if (!printsBloqueadosPorVoo()) {
                printlnLYellow("\r\n>>> %s CONFIRMADO <<<", PRINT_ESTADO[ESTADO_APOGEU]);
            }
            acionarDeploy("APOGEU DETECTADO");

            dadosVoo.estadoAtual = ESTADO_RECUPERACAO;
            landingConfirmCount = 0;
            break;
        }

        case ESTADO_RECUPERACAO: {
            const i32 ts = (i32)(now - seguroVoo.tempoInicioMs);
            registrarLog(dadosVoo, ts);

            // Estágio 2: Paraquedas Principal (J1 Main)
            if (!seguroVoo.mainAcionado && (dadosVoo.altitudeM <= ALTITUDE_MAIN_PARACHUTE)) {
                acionarMain("ALTITUDE DE PARAQUEDAS PRINCIPAL ATINGIDA");
            }

            // Detecção sustentada de pouso (2.0 segundos estáveis no solo)
            if ((dadosVoo.altitudeM < ALTITUDE_POUSO) &&
                (fabsf(dadosVoo.velocidadeMs) < VELOCIDADE_POUSO)) {
                ++landingConfirmCount;
                if (landingConfirmCount >= LANDING_CONFIRM_SAMPLES) {
                    encerrarVooComSeguranca(ESTADO_POUSADO, ">>> [POUSO] DETECTADO E LOG SALVO <<<");
                }
            } else {
                landingConfirmCount = 0;
            }
        
            // Se passar do TEMPO_MAX_VOO_MS o verificarErros() forca o ESTADO_POUSO
            break;
        }

        case ESTADO_POUSADO: {
            static u32 timerPousoBeep = 0;
            resgatarFoguete(3000, 800, &timerPousoBeep);
            break;
        }

        case ESTADO_ERRO: {
            static u32 timerErroBeep = 0;
            resgatarFoguete(6000, 1000, &timerErroBeep);
            break;
        }

        default:
            break;
    }
}

void simularVooAoVivoUSB(void) {
    char buffer_saida[256];
    char barra_grafica[55];

    // Banner Inicial 100% simetrico (118 caracteres de largura exatos)
    printlnLCyan("\r\n======================================================================================================================");
    printlnLYellow("                                      SIMULACAO DE VOO (SITL) - TEMPO REAL                                      ");
    printlnLCyan("======================================================================================================================");
    printlnLMagenta("   HORA   |        TELEMETRIA (A/V/P/T)           |     STATUS     | GRAFICO (# = 10m)                                ");
    printlnLCyan("----------|---------------------------------------|----------------|--------------------------------------------------");

    float alt = 0.0f;
    float vel = 0.0f;
    u8 estado = 1; // 1 = ESTADO_PRONTO
    u32 tempo_ms = 0;
    u8 h = 14, m = 30, s = 0;
    u32 marca_pouso = 0;

    // --- RASTREADORES DE ESTATISTICAS ---
    float alt_max = 0.0f;
    u32 tempo_decolagem = 0;
    u32 tempo_toque_chao = 0;

    for (int step = 0; step < 12000; step++) {

        // --- FISICA BASICA DO FOGUETE ---
        if (estado == 1 && tempo_ms > 1000) {
            estado = 2; // Decola apos 1s
            tempo_decolagem = tempo_ms; // <-- Marca a hora que saiu do chao
        } else if (estado == 2) {
            if (tempo_ms < 3000) vel += 45.0f * 0.01f; // Motor
            else vel -= 9.81f * 0.01f; // Gravidade
            if (vel <= 0.0f && alt > 50.0f) estado = 3; // Apogeu
        } else if (estado == 3) {
            estado = 4; vel = -15.0f; // Paraquedas abre
        } else if (estado == 4) {
            vel += ( (-8.0f - vel) * 0.05f ); // Arrasto do paraquedas
            if (alt <= 0.0f) {
                estado = 5; vel = 0.0f; alt = 0.0f;
                tempo_toque_chao = tempo_ms; // <-- Marca a hora exata do pouso
            }
        }

        alt += vel * 0.01f;
        if (alt < 0) alt = 0.0f;

        // Atualiza a altitude maxima da simulacao
        if (alt > alt_max) alt_max = alt;

        float pressao = 1013.25f * powf(1.0f - (alt / 44330.0f), 5.255f);
        float temp = 30.0f - (alt * 0.0065f);

        tempo_ms += 10;
        if (tempo_ms % 1000 == 0) {
            s++;
            if (s > 59) { s = 0; m++; }
            if (m > 59) { m = 0; h++; }
        }

        // --- IMPRESSAO NO TERMINAL A CADA 100ms ---
        if (tempo_ms % 100 == 0) {

            int num_barras = (int)(alt / 10.0f);
            if (num_barras > 50) num_barras = 50;
            if (num_barras < 0) num_barras = 0;

            memset(barra_grafica, 0, sizeof(barra_grafica));
            for (int i = 0; i < num_barras; i++) barra_grafica[i] = '#';

            u8 st = estado;
            if (st > 6) st = 6;

            snprintf(buffer_saida, sizeof(buffer_saida),
                "%s %02d:%02d:%02d %s|"
                "%s A:%7.1f V:%7.1f P:%6.1f T:%6.1f %s|"
                "%s %-14s %s|"
                "%s %s",
                RESET, h, m, s, LMAGENTA,
                LCYAN, alt, vel, pressao, temp, LMAGENTA,
                LYELLOW, PRINT_ESTADO[st], LMAGENTA,
                LGREEN, barra_grafica);

            printf("%s%s\r\n", buffer_saida, RESET);
        }

        delay(10);
        feedWatchdog();

        // Fica imprimindo os logs no chao por 2 segundos antes de encerrar
        if (estado == 5) {
            if (marca_pouso == 0) marca_pouso = tempo_ms;
            if (tempo_ms - marca_pouso > 2000) break;
        }
    }

    // --- CALCULO DAS ESTATISTICAS E RODAPE ---
    u32 tempo_voo_s = (tempo_toque_chao - tempo_decolagem) / 1000;
    u32 runtime_s = tempo_ms / 1000;

    printlnMagenta("======================================================================================================================");
    snprintf(buffer_saida, sizeof(buffer_saida),
            "ESTATISTICAS FINAIS:\r\n- Altitude Maxima: %.2f m; Tempo de Voo: %lu s; Runtime: %lu s.",
            alt_max, tempo_voo_s, runtime_s);
    printlnLGreen("%s", buffer_saida);
    printlnMagenta("======================================================================================================================");
    printlnLGreen("\r\n--- SIMULACAO CONCLUIDA. SISTEMA EM REPOUSO. ---");
}