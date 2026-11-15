
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// ===================================
// STM32F411CEU6 + LoRa RA-02
// SPI2
// ===================================

#define LORA_CS   PB12
#define LORA_SCK  PB13
#define LORA_MISO PB14
#define LORA_MOSI PB15
#define LORA_RST  PB0
#define LORA_DIO0 PB1

#define LED_PIN PC13
#define BTN_PIN PA0

#define LORA_FREQ 433E6

// Instancia da SPI2
SPIClass spiLoRa(
    LORA_MOSI,
    LORA_MISO,
    LORA_SCK
);

// Variaveis
bool ledLigado = false;
bool loraOK = false;

bool ultimaLeitura = HIGH;
bool estadoBotao = HIGH;
bool enviarAgora = false;

uint32_t ultimoDebounce = 0;
uint32_t ultimoTX = 0;
uint32_t contador = 0;

// ===================================
// LEITURA DO REGVERSION
// ===================================

uint8_t lerVersao(uint32_t velocidade) {

    spiLoRa.beginTransaction(
        SPISettings(
            velocidade,
            MSBFIRST,
            SPI_MODE0
        )
    );

    digitalWrite(LORA_CS, LOW);
    delayMicroseconds(5);

    spiLoRa.transfer(0x42);
    uint8_t valor = spiLoRa.transfer(0x00);

    digitalWrite(LORA_CS, HIGH);

    spiLoRa.endTransaction();

    return valor;
}

// ===================================
// CONTROLE DO BOTAO
// ===================================

void verificarBotao() {

    bool leitura = digitalRead(BTN_PIN);

    if (leitura != ultimaLeitura) {
        ultimoDebounce = millis();
    }

    if (millis() - ultimoDebounce > 40) {

        if (leitura != estadoBotao) {

            estadoBotao = leitura;

            if (estadoBotao == LOW) {

                ledLigado = !ledLigado;

                digitalWrite(
                    LED_PIN,
                    ledLigado ? LOW : HIGH
                );

                Serial.print("[BOTAO] LED ");

                Serial.println(
                    ledLigado ? "LIGADO" : "DESLIGADO"
                );

                // Transmite imediatamente
                enviarAgora = true;
            }
        }
    }

    ultimaLeitura = leitura;
}

// ===================================
// SETUP
// ===================================

void setup() {

    pinMode(LED_PIN, OUTPUT);
    pinMode(BTN_PIN, INPUT_PULLUP);

    digitalWrite(LED_PIN, HIGH);

    Serial.begin(115200);

    uint32_t inicio = millis();

    while (!Serial && millis() - inicio < 3000) {
        delay(10);
    }

    Serial.println();
    Serial.println("==========================");
    Serial.println(" STM32F411 - LORA SPI2");
    Serial.println("==========================");

    // CS
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    // RESET
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW);
    delay(20);
    digitalWrite(LORA_RST, HIGH);
    delay(100);

    // Inicializa SPI2
    spiLoRa.begin();

    Serial.println("[SPI2] SCK  = PB13");
    Serial.println("[SPI2] MISO = PB14");
    Serial.println("[SPI2] MOSI = PB15");
    Serial.println("[SPI2] CS   = PB12");

    // Testa comunicacao SPI
    uint8_t v100k = lerVersao(100000);
    uint8_t v1M = lerVersao(1000000);

    Serial.print("[SPI 100k] RegVersion = 0x");
    Serial.println(v100k, HEX);

    Serial.print("[SPI 1M] RegVersion = 0x");
    Serial.println(v1M, HEX);

    if (v100k != 0x12 && v1M != 0x12) {

        Serial.println(
            "[ERRO] SX1278 nao reconhecido!"
        );

        return;
    }

    Serial.println("[OK] SX1278 detectado!");

    spiLoRa.end();

    // Configura LoRa na SPI2
    LoRa.setSPI(spiLoRa);

    LoRa.setPins(
        LORA_CS,
        LORA_RST,
        LORA_DIO0
    );

    LoRa.setSPIFrequency(1000000);

    // Inicializa radio
    if (!LoRa.begin(LORA_FREQ)) {

        Serial.println(
            "[ERRO] Falha ao iniciar LoRa"
        );

        return;
    }

    // Parametros de comunicacao
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12);
    LoRa.setPreambleLength(8);
    LoRa.setTxPower(17);
    LoRa.enableCrc();

    loraOK = true;

    Serial.println("[OK] LoRa inicializado!");
    Serial.println("Frequencia: 433 MHz");
    Serial.println("SF: 7");
    Serial.println("BW: 125 kHz");
    Serial.println("CR: 4/5");
    Serial.println("CRC: ATIVADO");
    Serial.println("[TX] Sistema pronto");
}

// ===================================
// LOOP
// ===================================

void loop() {

    verificarBotao();

    if (!loraOK) {
        return;
    }

    if (enviarAgora ||
        millis() - ultimoTX >= 1000) {

        enviarAgora = false;
        ultimoTX = millis();

        char mensagem[80];

        snprintf(
            mensagem,
            sizeof(mensagem),
            "STM32,SEQ=%lu,LED=%s",
            (unsigned long)contador,
            ledLigado ? "ON" : "OFF"
        );

        if (LoRa.beginPacket()) {

            LoRa.print(mensagem);

            int resultado = LoRa.endPacket();

            if (resultado == 1) {

                Serial.print("[TX OK] ");
                Serial.println(mensagem);

            } else {

                Serial.println("[ERRO] Falha TX");
            }

        } else {

            Serial.println("[ERRO] Radio ocupado");
        }

        contador++;
    }
}
