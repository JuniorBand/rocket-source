/*
  ******************************************************************************
  * @file    config_voo.cpp
  * @date 	 14 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Source code que configura a parte complexa do codigo do foguete.
  ******************************************************************************
*/

#include <macros_config.h>
#include <utils.h>
#include <prints.h>
#include <config_voo.h>
#include <bmp280.h>
#include <mpu6050.h>
#include <telemetria_com.h>
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


// ============================================================
// ESTADO GLOBAL
// ============================================================

EstadoSistema_t currentState = STATE_STOPPED;

CaixaPreta_t caixaPreta;
bool     caixaPretaValida      = false;
bool     unknownFlashData = false;   // dados em formato antigo/desconhecido

float    basePressurePa   = 0.0f;
float    baseTemperatureC = 0.0f;

// Registro ativo pode ser 16 bytes (voo antigo v2) ou 32 bytes (v3).
u32 activeRecordSize = RECORD_SIZE;
u32 flashCapacity    = 0;
u32 maxRecords       = 0;
u32 recordCount      = 0;
u32 flightStartTime  = 0;
u8  pendingRecordFlags = 0;

DadosVoo_t   latest;
bool     hasLatest        = false;
bool     latestStored     = false;
float    verticalSpeed    = 0.0f;
float    prevAltitude     = 0.0f;
u32 prevAltitudeTime = 0;
bool     prevAltitudeValid = false;

u32 nextDadosVooTime        = 0;
u32 lastTelemetryTime     = 0;
u32 lastSensorErrorPrint  = 0;

u8  launchConfirmCount    = 0;
u32 launchCandidateTime   = 0;

DadosVoo_t   preRing[PRETRIGGER_SAMPLES];
u16 preRingHead  = 0;
u16 preRingCount = 0;


u8 flagGravacaoParada = 0;
u8 flagFimDeVoo = 0;
static u16 timer_beep = 0;
static TIM_TypeDef *TIM_MS;
static RTC_HandleTypeDef *HRTC_SYS_PTR;
static u8 contadorFalhasSensor = 0;

#ifdef VERBOSE
	static u32 timer_verbose = 0;
#endif


// ============================================================
// INICIALIZAÇÃO
// ============================================================

// Precisa ser 'volatile' porque é alterada dentro da interrupção e lida no loop()
volatile bool dadosProntos = false;

HardwareTimer *dadosTimer = nullptr;

// Callback da interrupção: roda a exatos 20 Hz (50 ms) no hardware
void onDadosTimerISR() {
    dadosProntos = true;
}

void setupTimer() {
    dadosTimer = new HardwareTimer(TIM2);
    dadosTimer->setOverflow(20, HERTZ_FORMAT); // 20 Hz = período exato de 50 ms
    dadosTimer->attachInterrupt(onDadosTimerISR);
    dadosTimer->resume();
}

void waitForSerial(u32 timeoutMs) {
    const u32 start = millis();
    while (!Serial && millis() - start < timeoutMs) {
        delay(10);
    }
}

void resumeFlight(void) {
    basePressurePa = caixaPreta.basePressurePa;
    baseTemperatureC = caixaPreta.baseTemperatureC;
    resetFlightTracking();
    for (u8 a = 0; a < 3; ++a) gyroBiasRaw[a] = caixaPreta.gyroBiasRaw[a];
    gyroCalibrated = (caixaPreta.mpuCalibrated != 0);

    // Continua a contagem de tempo a partir do último registro salvo.
    int32 lastTimestamp = 0;
    LogRecord record;
    if (recordCount > 0 && readRecord(recordCount - 1, record) && recordCrcOk(record)) {
        lastTimestamp = record.timestampMs;
    }
    flightStartTime = millis() - static_cast<u32>(lastTimestamp + SAMPLE_INTERVAL_MS);

    pendingRecordFlags = FLAG_RESUMED;
    currentState = STATE_RECORDING;

    Serial.println(F("[AVISO] Reinicio pelo watchdog em voo; GRAVACAO RETOMADA."));
    Serial.println(F("Usando a pressao base salva no cabecalho (sem recalibrar)."));
}


// ============================================================
// SENSOR
// ============================================================


void resetFlightTracking(void) {
    launchConfirmCount = 0;
    preRingHead = 0;
    preRingCount = 0;
    prevAltitudeValid = false;
    verticalSpeed = 0.0f;
    pendingRecordFlags = 0;
}

bool calibrateBase(void) {
    Serial.println(F("[SISTEMA] Calibrando pressao de referencia no solo..."));

    const u32 settleStart = millis();
    while (millis() - settleStart < CALIB_SETTLE_MS) {
        feedWatchdog();
        delay(10);
    }

    double pressureSum = 0.0;
    double temperatureSum = 0.0;
    u8 dadosValidos = 0;

    for (u8 i = 0; i < CALIB_SAMPLES; ++i) {
        float pressure;
        float temperature;
        if (readRawSensor(pressure, temperature)) {
            pressureSum += pressure;
            temperatureSum += temperature;
            ++dadosValidos;
        }
        delay(20);
        feedWatchdog();
    }

    if (dadosValidos < CALIB_MIN_VALID) {
        Serial.println(F("[ERRO] Leituras validas insuficientes para calibrar o BMP280."));
        return false;
    }

    basePressurePa = static_cast<float>(pressureSum / dadosValidos);
    baseTemperatureC = static_cast<float>(temperatureSum / dadosValidos);
    resetFlightTracking();

    Serial.print(F("[OK] Pressao base: "));
    Serial.print(basePressurePa, 1);
    Serial.print(F(" Pa | Temperatura: "));
    Serial.print(baseTemperatureC, 2);
    Serial.println(F(" C"));
    return true;
}



// ============================================================
// CICLO DE VOO
// ============================================================

void preRingPush(const DadosVoo_t &dadosVoo) {
    preRing[preRingHead] = dadosVoo;
    preRingHead = (preRingHead + 1) % PRETRIGGER_SAMPLES;
    if (preRingCount < PRETRIGGER_SAMPLES) {
        ++preRingCount;
    }
}

void stopWithError(FlashStringPtr message) {
    Serial.println();
    Serial.print(F("[ERRO] "));
    Serial.println(message);
    currentState = STATE_STOPPED;
    playBeeps(2, 500, 200);
}

void stopRecording(uint8_t flagFimDeVoo, bool writeMarker = true) {
    if (currentState != STATE_RECORDING) {
        Serial.println(F("[STATUS] A gravacao ja nao estava ativa."));
        return;
    }

    bool markerOk = true;

    // Grava um registro extra de encerramento quando a parada ocorre de forma assíncrona
    // (ex.: comando S) e ainda existe espaço livre na Flash.
    if (writeMarker && recordCount < maxRecords) {
        DadosVoo_t endSample = latest;
        if (!hasLatest) {
            endSample.uptimeMs = millis();
            endSample.pressurePa = basePressurePa;
            endSample.temperatureC = baseTemperatureC;
            endSample.altitudeM = 0.0f;
            memset(endSample.accelRaw, 0, sizeof(endSample.accelRaw));
            memset(endSample.gyroRaw, 0, sizeof(endSample.gyroRaw));
            endSample.imuStatus = 0;
        }

        const int32_t timestamp = static_cast<int32_t>(millis() - flightStartTime);
        markerOk = appendRecord(endSample, timestamp, flagFimDeVoo);
    }

    currentState = STATE_STOPPED;

    Serial.println();
    switch (flagFimDeVoo) {
        case FLAG_END_LANDED:
            Serial.println(F("[STATUS] Pouso detectado: gravacao encerrada."));
            break;
        case FLAG_END_MANUAL:
            Serial.println(F("[STATUS] Gravacao encerrada manualmente."));
            break;
        case FLAG_END_FULL:
            Serial.println(F("[AVISO] Memoria Flash cheia: gravacao encerrada."));
            break;
        case FLAG_END_TIMEOUT:
            Serial.println(F("[AVISO] Timeout: gravacao encerrada."));
            break;
        default:
            Serial.println(F("[STATUS] Gravacao encerrada."));
            break;
    }

    if (!markerOk) {
        Serial.println(F("[AVISO] Falha ao gravar o marcador de fim na Flash."));
    }

    Serial.print(F("Total de registros salvos: "));
    Serial.println(recordCount);

    // 3 bipes curtos para encerramento esperado; 2 longos apenas se lotou a Flash
    if (flagFimDeVoo == FLAG_END_FULL) {
        playBeeps(2, 500, 200);
    } else {
        playBeeps(3, 100, 80);
    }
}

void startFlight(u32 launchTime) {
    if (!writeHeader()) {
        stopWithError(F("Falha ao gravar o cabecalho do voo; gravacao nao iniciada."));
        return;
    }

    flightStartTime = launchTime;

    // Descarrega o buffer pré-trigger, do mais antigo para o mais recente.
    for (u16 i = 0; i < preRingCount; ++i) {
        const u16 index =
            (preRingHead + PRETRIGGER_SAMPLES - preRingCount + i) % PRETRIGGER_SAMPLES;
        DadosVoo_t dadosVoo = preRing[index];
        dadosVoo.altitudeM = relativeAltitude(dadosVoo.pressurePa);

        const int32 timestamp = static_cast<int32>(dadosVoo.uptimeMs - launchTime);
        u8 flags = (timestamp < 0) ? FLAG_PRETRIGGER : 0;
        if (dadosVoo.uptimeMs == launchTime) {
            flags |= FLAG_LAUNCH;
        }

        if (!appendRecord(dadosVoo, timestamp, flags)) {
            stopWithError(F("Falha ao gravar o buffer pre-trigger."));
            return;
        }
    }
    preRingCount = 0;
    latestStored = true;

    currentState = STATE_RECORDING;
    playLaunchPattern();

    Serial.println();
    Serial.println(F(">>> LANCAMENTO DETECTADO - GRAVACAO INICIADA <<<"));
}



void handleIdleSample(const DadosVoo_t &dadosVoo) {
    preRingPush(dadosVoo);

    if (dadosVoo.altitudeM >= ALTITUDE_MIN_EJECAO) {
        if (launchConfirmCount == 0) {
            launchCandidateTime = dadosVoo.uptimeMs;
        }
        ++launchConfirmCount;

        if (launchConfirmCount >= QUANTIDADE_MIN_DADOS_EJECAO) {
            startFlight(launchCandidateTime);
        }
        return;
    }

    // O zero fica fixo desde a calibração (como no código original).
    launchConfirmCount = 0;
}



void processarDadosSensores(void) {

    latestStored = false;

    if (!readRawSensor(sensor.pressao, sensor.temperatura)) {
        if (now - lastSensorErrorPrint >= 1000) {
            Serial.println(F("[ERRO] Leitura invalida do BMP280; amostra ignorada."));
            lastSensorErrorPrint = now;
        }
        return;
    }

    sensor_t sensor;
    sensor.uptimeMs = millis();
    sensor.altitude = relativeAltitude(pressao);
    mpuAcquire(sensor, now);

    #ifdef USE_PASSA_BAIXA_FILTER
        // Velocidade vertical estimada (filtrada) para a telemetria.
        if (prevAltitudeValid && now != prevAltitudeTime) {
            const float dt = (now - prevAltitudeTime) / 1000.0f;
            const float rawSpeed = (sensor.altitudeM - prevAltitude) / dt;
            verticalSpeed += VZ_FILTER_ALPHA * (rawSpeed - verticalSpeed);
        }
        prevAltitude = sensor.altitudeM;
        prevAltitudeTime = now;
        prevAltitudeValid = true;
    #endif

    latest = sensor;
    hasLatest = true;
    return;
}

    if (currentState == ESTADO_PRONTO) {
        handleIdleSample(dadosVoo);
    } else if (currentState == STATE_RECORDING) {
        handleRecordingSample(dadosVoo);
    }

void processarLogicaVoo(void) { // Acionado no loop pelo timer

    #ifdef USE_KALMAN_FILTER
        static u32 timer_estabilizacao_kalman = 0;
    #endif
    

    // Agora sim, executa a logica pesada de sensores e flash com tempo garantido
    #ifndef EM_VOO
        #pragma message("Utilizando conexao via USB.")
        processarComandosUSB();
    #endif

    processarDadosSensores();

    #ifdef USE_KALMAN_FILTER
        if (sensor.pressao > 300.0f && sensor.pressao < 1200.0f && sensor.temperatura > -40.0f && sensor.temperatura < 85.0f) {
            filtroKalman.update(&filtroKalman, sensor.altitude, DT, &(dadosVoo.altitudeAtual), &(dadosVoo.velocidadeAtual));
            dadosVoo.temperaturaAtual = sensor.temperatura;
            dadosVoo.pressaoAtual = sensor.pressao;
        }
    #else
        dadosVoo.temperaturaAtual = sensor.temperatura;
        dadosVoo.pressaoAtual = sensor.pressao;
        dadosVoo.altitudeAtual = sensor.altitude;
        dadosVoo.velocidadeAtual = 0.0f;
    #endif

    #ifdef VERBOSE
        if(HAL_GetTick() - timer_verbose >= 1000){
            printlnLCyan("\r\nDADOS FILTRADOS: P: %.2f; T: %.2f; A: %.2f; V: %.2f; E: %s", dadosVoo.pressaoAtual, \
                    dadosVoo.temperaturaAtual, dadosVoo.altitudeAtual, dadosVoo.velocidadeAtual, PRINT_ESTADO[dadosVoo.estadoAtual]);
            timer_verbose = HAL_GetTick();
        }
    #endif

    //verificarErros();

    switch (dadosVoo.estadoAtual) {

        case ESTADO_CALIBRACAO:
            {
                    static u16 ciclos_aquecimento = 0;
                    static double soma_pressao_ref = 0.0f; // Usando double para nao perder precisao decimal!
                    static u16 amostras_validas = 0;

                    // Fase 1: Ignora os primeiros 200 ticks (2 SEGUNDOS INTEIROS) pro chip esquentar
                    if (ciclos_aquecimento < 200) {
                        ciclos_aquecimento++;
                    }
                    // Fase 2: Coleta 100 amostras (mais 1 segundo tirando a média perfeita)
                    else if (amostras_validas < 100) {
                        // MURALHA DE FERRO DA CALIBRAÇAO:
                        // So aceita somar na média se a pressao for da Terra (entre 300 e 1200)
                        if (sensor.pressao > 300.0f && sensor.pressao < 1200.0f) {
                            soma_pressao_ref += (double)sensor.pressao;
                            amostras_validas++;
                        }
                        // Se vier glitch > 1200, ele simplesmente ignora o loop e tenta de novo 10ms depois
                    }
                    // Fase 3: Calibraçao finalizada!
                    else {
                        // Calcula a média com precisao maxima e converte de volta pra float
                        sensor.pressao_ref = (float)(soma_pressao_ref / 100.0);

                        // Força o calculo da altitude agora para cravar o 0.00m
                        sensor.altitude = 44330.0f * (1.0f - powf((sensor.pressao / sensor.pressao_ref), 0.190295f));

                        sensor.altitude = 0.0f;
                        dadosVoo.velocidadeAtual = 0.0f; // MATA A VELOCIDADE FANTASMA AQUI!

                        // ESSENCIAL: O Filtro de Kalman acumulou lixo durante esses 3 segundos.
                        // Avisamos ele: "Apaga tudo! A altitude oficial agora é 0.00m e estamos parados".
                        filtroKalman.reiniciar(&filtroKalman, &(sensor.altitude), &(dadosVoo.velocidadeAtual));

                        printlnLCyan("\r\n>>> CALIBRACAO CONCLUIDA! Pref: %.2f hPa <<<", sensor.pressao_ref);
                        BUZZER_ON();
                        timer_beep = 50;

                        dadosVoo.estadoAtual = ESTADO_PRONTO;
                        registrarLogVoo();

                        ciclos_aquecimento = 0;
                        soma_pressao_ref = 0.0f;
                        amostras_validas = 0;
                        timer_estabilizacao_kalman = 0;
                    }
                    break;
                }

        case ESTADO_PRONTO:
                {
                    // 1. Espera 1 segundo para o Kalman estabilizar (sem tentar decolar)
                    if (timer_estabilizacao_kalman < 100) {
                        timer_estabilizacao_kalman++;
                        break;
                    }

                    // 2. TIMEOUT DE GRAVAÇAO (Pad Idle Logging Rate)
                    // A logica roda a 100Hz. Vamos gravar apenas a 2Hz (2 vezes por segundo).
                    static u8 timer_gravacao_pronto = 0;
                    timer_gravacao_pronto++;

                    if (timer_gravacao_pronto >= 50) { // 50 ticks de 10ms = 500ms
                        registrarLogVoo();
                        timer_gravacao_pronto = 0;
                    }

                    // 3. Regras de Lançamento (Blindadas - Rodando a 100Hz)
                    if ((dadosVoo.altitudeAtual > ALTITUDE_LANCAMENTO) && (dadosVoo.velocidadeAtual > VELOCIDADE_LANCAMENTO)) {

                        dadosVoo.estadoAtual = ESTADO_EM_VOO;
                        seguroVoo.tempo_inicio_ms = HAL_GetTick();

                        // Zera o timer para nao interferir em um proximo ciclo caso o sistema seja reiniciado
                        timer_gravacao_pronto = 0;

                        // Força um registro imediato no exato milissegundo da decolagem
                        registrarLogVoo();
                        salvarCaixaPretaW25Q(sensor.pressao_ref, MAGIC_NUMBER_SIRIUS);

                        printlnLCyan("\r\n>>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_EM_VOO]);
                        BUZZER_ON();
                        timer_beep = 50;
                    }
                    break;
                }

        case ESTADO_EM_VOO:
            {
                registrarLogVoo();
                // Atualiza altitude maxima
                if (dadosVoo.altitudeAtual > seguroVoo.altitude_maxima) {
                    seguroVoo.altitude_maxima = dadosVoo.altitudeAtual;
                    #ifdef VERBOSE
                        printlnGreen("RAW: %.2f | KALMAN: %.2f | VEL: %.2f", sensor.altitude, dadosVoo.altitudeAtual, dadosVoo.velocidadeAtual);
                    #endif
                }

                // =========================================================================
                // CORREÇAO: ALTITUDE LOCKOUT (TRAVA DE APOGEU NA BANCADA)
                // So permite acionar apogeu se ja passamos de ALTITUDE_MIN_EJECAO na vida real!
                // =========================================================================
                else if ( (dadosVoo.altitudeAtual >= (META_APOGEU - DESVIO_MIN)) ||
                            ( (dadosVoo.altitudeAtual < (seguroVoo.altitude_maxima - DESCIDA_MINIMA))
                            && (seguroVoo.altitude_maxima > ALTITUDE_MIN_EJECAO) )
                            ) {

                    if(!(dadosVoo.altitudeAtual >= (META_APOGEU - DESVIO_MIN))){ // Nao atingiu a meta de apogeu
                        printlnLRed("ATENCAO: QUEDA ANTES DA META DE APOGEU!!!");
                    }

                    dadosVoo.estadoAtual = ESTADO_APOGEU;
                    BUZZER_ON();
                    timer_beep = 50;
                }

                // Verifica falha de tempo primeiro (prioridade maxima de erro)
                if (HAL_GetTick() - seguroVoo.tempo_inicio_ms > TEMPO_MAX_PARAQUEDAS_MS) {
                    dadosVoo.estadoAtual = ESTADO_APOGEU;
                    registrarLogVoo();
                    acionarEjecao("\r\nTIMEOUT DO PARAQUEDAS - ACIONAMENTO FORCADO");
                }

                // Verifica falha de tempo primeiro (prioridade maxima de erro)
                if (HAL_GetTick() - seguroVoo.tempo_inicio_ms > TEMPO_MAX_VOO_MS) {
                    dadosVoo.estadoAtual = ESTADO_ERRO;
                    registrarLogVoo();
                    acionarEjecao("\r\nTIMEOUT DE VOO - APOGEU NAO DETECTADO");
                }

                break;
            }

        case ESTADO_APOGEU:
            {
                registrarLogVoo();
                printlnLYellow("\r\n>>> %s CONFIRMADO <<<", PRINT_ESTADO[ESTADO_APOGEU]);
                acionarEjecao(PRINT_ESTADO[ESTADO_APOGEU]);
                dadosVoo.estadoAtual = ESTADO_RECUPERACAO;
                BUZZER_OFF();
                break;
            }

        case ESTADO_RECUPERACAO:
            {
                // Detecta Pouso (Baixa alt e Baixa dadosVoo.velocidadeAtual )
                if ( (dadosVoo.altitudeAtual < ALTITUDE_POUSO) && ( (dadosVoo.velocidadeAtual > -VELOCIDADE_POUSO)
                        && (dadosVoo.velocidadeAtual < VELOCIDADE_POUSO) ) ) {
                    dadosVoo.estadoAtual = ESTADO_POUSADO;
                }
                else {
                    registrarLogVoo();
                }
                break;
            }

        case ESTADO_POUSADO:
            {
                if (!flagFimDeVoo) {
                    registrarLogVoo();
                    printlnLGreen("\r\n>>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_POUSADO]);
                    // -> CAIXA PRETA: Pousou seguro! Desativa a flag para nao entrar em recuperaçao no proximo boot.
                    salvarCaixaPretaW25Q(1013.25f, 0x00000000);
                    descarregarBuffer();
                    flagGravacaoParada = 1;
                    flagFimDeVoo = 1;
                }
                static u32 timerPouso = 0;
                resgatarFoguete(3000, 1000, &timerPouso); // A cada 2 segundos.
                processarComandosUSB();
                break;
            }

        case ESTADO_ERRO:
            {

                if (!flagFimDeVoo) {
                    printlnRed("\r\n=================================================");
                    printlnRed("           >>>   %s!   <<<", PRINT_ESTADO[ESTADO_ERRO]);
                    printlnRed("           >>> SISTEMA PARALISADO <<<");
                    printlnRed("=================================================");
                    descarregarBuffer();
                    flagGravacaoParada = 1;
                    flagFimDeVoo = 1;
                }
                
                static u32 timerErro = 0;
                resgatarFoguete(6000, 1000, &timerErro); // A cada 5 segundos.
                break;
            }

        default: { break; }
    }
}



static inline void resgatarFoguete(u32 intervalo, u32 tempo_ligado, u32 *ultimoBeep) {
    u32 tempo_atual = millis();

    // Se passou do intervalo, liga o buzzer e atualiza a marcaçao de tempo
    if (tempo_atual - *ultimoBeep >= intervalo) {
    	BUZZER_ON();
        *ultimoBeep = tempo_atual;
    }
    // Desliga o buzzer se ja passou o tempo_ligado desde a ultima marcaçao
    else if (tempo_atual - *ultimoBeep >= tempo_ligado) {
        writePinLow(BUZZER_PORT, BUZZER_PIN);
        BUZZER_OFF();
    }
}

static inline void registrarLogVoo(void) {
    if (!flagGravacaoParada) {
        adicionarLogW25Q(HRTC_SYS_PTR, &dadosVoo);
    }
}

#if defined(MOSFET_PORT) && defined(MOSFET_PIN)
	#pragma message("MOSFET_PORT e MOSFET_PIN foram definidos.")

	static void acionarEjecao(const char *razao_missao){

		printlnMagenta("\r\n=================================================");
		printlnLCyan("ACIONANDO EJECAO EM %.2f m. MOTIVO: %s", dadosVoo.altitudeAtual, razao_missao);
		printlnRed("           >>> Altitude Maxima: %.2f m!   <<<", seguroVoo.altitude_maxima);
		printlnMagenta("=================================================");
		writePinHigh(MOSFET_PORT, MOSFET_PIN);
		return;
	}
#else
	#pragma message("MOSFET_PORT e MOSFET_PIN nao foram definidos no main.h!")

	static void acionarEjecao(const char *razao_missao){
			printlnMagenta("\r\n=========================================================");
			printlnLCyan("[SIMULACAO] ACIONANDO EJECAO EM %.2f m. MOTIVO: %s", dadosVoo.altitudeAtual, razao_missao);
			printlnRed("           >>> Altitude Maxima: %.2f m!   <<<", seguroVoo.altitude_maxima);
			printlnMagenta("=========================================================");
			return;
	}
#endif