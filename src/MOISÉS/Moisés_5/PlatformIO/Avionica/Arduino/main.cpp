#include <macros_config.h>
#include <utils.h>
#include <prints.h>
#include <config_voo.h>
#include <bmp280.h>
#include <mpu6050.h>
#include <w25q.h>
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

void setup() {
    const bool watchdogReset = IWatchdog.isReset(true);

    pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);
    pinMode(W25Q_CS, OUTPUT); digitalWrite(W25Q_CS, HIGH);
    pinMode(BMP_CS, OUTPUT); digitalWrite(BMP_CS, HIGH);

    Serial.begin(115200);
    SerialTTL.begin(UART_TTL_BAUD);  // USART2 PA3=RX e PA2=TX, 8N1

    // --- Flash primeiro: decide rápido se é preciso retomar um voo ---
    SPI.begin();
    const bool flashOk = flash.begin();
    bool resumeCandidate = false;
    bool flightEnded = false;

    if (flashOk) {
        flashCapacity = flash.getCapacity();
        if (flashCapacity > DATA_START_ADDR + RECORD_SIZE) {
            caixaPretaValida = loadHeader();
            maxRecords = (flashCapacity - DATA_START_ADDR) / activeRecordSize; // Quantidade maxima de structs
            u32 slots = 0;
            const bool scanned = findFirstErasedSlot(DATA_START_ADDR, maxRecords,
                                                     activeRecordSize, slots);
            if (scanned) { recordCount = slots; }
            if (!caixaPretaValida) {
                unknownFlashData = !scanned || (recordCount > 0) || !caixaPretaAreaErased();
                if (unknownFlashData) recordCount = 0;
            } else if (recordCount > 0) {
                if (caixaPreta.version == HEADER_VERSION) {
                    LogRecord last;
                    if (readRecord(recordCount - 1, last) && recordCrcOk(last))
                        flightEnded = (last.flags & FLAG_END_MASK) != 0;
                } else {
                    LogRecordV2 last;
                    if (readLegacyRecord(recordCount - 1, last) && recordCrcOk(last))
                        flightEnded = (last.flags & FLAG_END_MASK) != 0;
                }
            }
            // Apenas um voo v3 inacabado pode ser retomado apos watchdog.
            resumeCandidate = watchdogReset && caixaPretaValida &&
                              caixaPreta.version == HEADER_VERSION && !flightEnded &&
                              recordCount < maxRecords;
        }
    }

    // Em retomada de voo não há tempo a perder esperando o USB.
    if (!resumeCandidate) {
        waitForSerial(USB_WAIT_MS);
        beepBlocking(100, 3);
    }

    Serial.println();
    Serial.println(F("Inicializando avionica..."));
    if (watchdogReset) {
        Serial.println(F("[AVISO] Reinicio causado pelo watchdog."));
    }

    // --- Flash ---
    if (!flashOk) {
        haltWithError(F("Memoria SPI Flash nao encontrada."), 100, 2, 1000);
    }
    if (maxRecords == 0) {
        haltWithError(F("Capacidade da Flash insuficiente."), 500, 2, 1000);
    }

    Serial.print(F("[OK] Flash: JEDEC 0x"));
    Serial.print(flash.getJEDECID(), HEX);
    Serial.print(F(", "));
    Serial.print(flashCapacity);
    Serial.println(F(" bytes"));

    bmpSetup();
    mpuSetup();

    

    Serial.print(F("Registros encontrados: "));
    Serial.println(recordCount);

    // --- Decide o estado inicial ---
    if (resumeCandidate) {
        resumeFlight();
    } else if (unknownFlashData) {
        currentState = STATE_STOPPED;
        Serial.println(F("[AVISO] Flash contem dados em formato antigo/desconhecido."));
        Serial.println(F("Envie A para apagar e armar."));
    } else if (caixaPretaValida) {
        basePressurePa = caixaPreta.basePressurePa;
        baseTemperatureC = caixaPreta.baseTemperatureC;
        currentState = STATE_STOPPED;
        Serial.println(F("[AVISO] Existe um log salvo; gravacao bloqueada para nao misturar voos."));
        if (caixaPreta.version == LEGACY_VERSION) Serial.println(F("[INFO] Voo antigo v2 legivel pelo comando V."));
        Serial.println(F("Envie V para ver os dados ou A para apagar e armar."));
        // A telemetria usa a pressão atual do solo como referência.
        if (!calibrateBase()) {
            Serial.println(F("[AVISO] Calibracao falhou; telemetria usara a base do log."));
            basePressurePa = caixaPreta.basePressurePa;
        }
    } else {
        if (!calibrateBase()) {
            haltWithError(F("Nao foi possivel calibrar o BMP280."), 500, 2, 1000);
        }
        if (mpuAvailable) {
            Serial.println(F("[MPU] Mantenha a placa parada: calibrando giroscopio..."));
            if (mpuCalibrateGyro()) Serial.println(F("[OK] Offset do giroscopio calibrado."));
            else Serial.println(F("[AVISO] Calibracao de giro rejeitada; giro sem correcao."));
        }
        currentState = STATE_IDLE;
    }

    IWatchdog.begin(WATCHDOG_TIMEOUT_US);
    watchdogStarted = true;

    nextSampleTime = millis();
    lastChirpTime = millis();

    if (!resumeCandidate) {
        beepBlocking(500);
    }

    Serial.println();
    Serial.println(F(">>> SISTEMA INICIALIZADO <<<"));
    switch (currentState) {
        case ESTADO_PRONTO:
            Serial.println(F(">>> Armado: aguardando subida de +3 m <<<"));
            break;
        case ESTADO_VOO:
            Serial.println(F(">>> Gravando (retomado) <<<"));
            break;
        default:
            Serial.println(F(">>> Log preservado: V para ver, A para apagar e armar <<<"));
            break;
    }
    Serial.println(F("Comandos USB: V = ver dados | A = apagar e armar | S = parar"));
    Serial.println(F("[FIRMWARE] UART BINARIA v2 - S REMOTO + V + A HABILITADOS"));
    Serial.println(F("UART2 PA2 TX / PA3 RX: binario 115200, 20 Hz, sem modo Tera Term"));
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    feedWatchdog();
    beeperUpdate();
    processSerialCommands();
    ttlReceiveCommands();        // Pacotes binarios; respostas ACK/NACK

    // Timer de amostragem do BMP280 e MPU6050 (20ms = 100Hz)
    if (dadosProntos) {
        dadosProntos = false;
        const u32 now = millis();
        processarLogicaVoo();
        
        if (hasLatest && latest.uptimeMs == now) {
            ttlSendTelemetry();
        }
    }

    ttlStreamLog();               // transferencia binaria de logs
    printTelemetry(now);          // USB CDC continua igual nos dois modos
    updateLed(now);

    // Sinais sonoros de estado (não bloqueantes).
    if (!beeperActive) {
        if (currentState == STATE_IDLE && READY_CHIRP_INTERVAL_MS > 0 &&
            now - lastChirpTime >= READY_CHIRP_INTERVAL_MS) {
            lastChirpTime = now;
            playBeeps(1, 30, 0);
        }
    }
}
