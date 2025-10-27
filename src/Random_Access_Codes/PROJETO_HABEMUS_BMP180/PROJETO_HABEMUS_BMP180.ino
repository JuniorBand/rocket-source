/*
============================================================================
CÓDIGO DO FOGUETE - VERSÃO PARA ARDUINO MINI COM SENSOR BMP180
COM SAÍDA DETALHADA E DETECÇÃO DE DESCONEXÃO DO SENSOR EM VOO

 1. Detecção de Desconexão:
   - Adicionada uma verificação de comunicação I2C no início de cada loop.
   - Se o sensor parar de responder (ex: fio solto), o sistema de recuperação
     é acionado imediatamente por segurança.
============================================================================
*/

#include <Wire.h>
#include <Adafruit_BMP085.h>

// Definições de Pinos para Arduino Mini / Uno / Nano
#define pinoBuzzer 4
#define pinoMosfet1 9
#define pinoMosfet2 10
#define pinoLed 13 // Pino do LED indicador (LED integrado na placa)

// Pinos I2C: A4 (SDA) e A5 (SCL) são usados por padrão pela biblioteca Wire

// Constantes de Voo e Segurança
const float LIMITE_SALTO_ANOMALO = 90.0;
const float ALTITUDE_DE_LANCAMENTO = 0.5;
const int BMP180_I2C_ADDRESS = 0x77; // Endereço I2C padrão do sensor BMP180

// Instância de Objeto de Biblioteca
Adafruit_BMP085 bmp;

// Variáveis Globais de Estado
float apogeu = 0.0;
bool acionamentoIniciado = false;
bool foguete_lancado = false;
bool primeira_medicao_pos_calibracao = true;
int contadorLeiturasInvalidas = 0;
float ultima_altitude_relativa = 0.0;
float leitura_inicial_fixa = 0.0;

// Parâmetros da Média Móvel
const int quantidade_leituras_media = 5;
float buffer_leituras[quantidade_leituras_media];
int indice_buffer = 0;

// Protótipos de Funções
void verificaAltitude(float altitude);
void finalizarMissao(const __FlashStringHelper* razao);


void setup() {
  Serial.begin(115200);
  Serial.println(F("\n\n==============================================="));
  Serial.println(F("Iniciando Computador de Voo - Habemos 1.2"));
  Serial.println(F("==============================================="));

  Serial.println(F("Configurando pinos de saida..."));
  pinMode(pinoBuzzer, OUTPUT);
  pinMode(pinoMosfet1, OUTPUT);
  pinMode(pinoMosfet2, OUTPUT);
  pinMode(pinoLed, OUTPUT);

  digitalWrite(pinoMosfet1, LOW);
  digitalWrite(pinoMosfet2, LOW);

  digitalWrite(pinoBuzzer, HIGH);
  delay(150);
  digitalWrite(pinoBuzzer, LOW);
  Serial.println(F("Pinos configurados."));

  Serial.println(F("Inicializando comunicacao I2C..."));
  Wire.begin();

  // --- ETAPA 1: VERIFICAÇÃO DE SENSOR ---
  Serial.println(F("\n[ETAPA 1/2] Verificando sensor BMP180..."));
  if (!bmp.begin()) {
    Serial.println(F("ERRO CRITICO: Sensor BMP180 nao encontrado."));
    finalizarMissao(F("FALHA SENSOR NA INICIALIZACAO"));
  }
  Serial.println(F("Sensor BMP180 encontrado com sucesso."));

  // --- ETAPA 2: CALIBRAÇÃO DE ALTITUDE ---
  Serial.println(F("\n[ETAPA 2/2] Iniciando calibracao de altitude..."));
  digitalWrite(pinoBuzzer, HIGH); delay(100); digitalWrite(pinoBuzzer, LOW); delay(100);
  digitalWrite(pinoBuzzer, HIGH); delay(100); digitalWrite(pinoBuzzer, LOW);

  float soma_calibracao = 0.0;
  for (int i = 0; i < quantidade_leituras_media; i++) {
    buffer_leituras[i] = bmp.readAltitude();
    soma_calibracao += buffer_leituras[i];
    Serial.print(F("Leitura de calibracao "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(buffer_leituras[i]);
    delay(200);
  }
  leitura_inicial_fixa = soma_calibracao / quantidade_leituras_media;

  Serial.println(F("\nCalibracao concluida."));
  Serial.print(F("Altitude de referencia (zero) definida em: "));
  Serial.print(leitura_inicial_fixa);
  Serial.println(F(" m"));

  // --- FINALIZAÇÃO DO SETUP ---
  Serial.println(F("\n==============================================="));
  Serial.println(F("SISTEMA ARMADO. Aguardando lancamento..."));
  Serial.println(F("==============================================="));

  digitalWrite(pinoBuzzer, HIGH); delay(300); digitalWrite(pinoBuzzer, LOW); delay(100);
  digitalWrite(pinoBuzzer, HIGH); delay(300); digitalWrite(pinoBuzzer, LOW);
}


void loop() {
  // --- NOVA VERIFICAÇÃO DE CONEXÃO DO SENSOR A CADA CICLO ---
  // Tenta comunicar com o endereço do sensor. Se não houver resposta, há desconexão.
  Wire.beginTransmission(BMP180_I2C_ADDRESS);
  byte error = Wire.endTransmission();
  if (error != 0) {
    // Se 'error' não for 0, o sensor não respondeu. Finaliza a missão.
    finalizarMissao(F("FALHA DE COMUNICACAO COM SENSOR EM VOO"));
  }

  // Se o sensor respondeu, prossegue com a leitura normal
  float altitude_absoluta_atual = bmp.readAltitude();

  digitalWrite(pinoLed, HIGH);
  delay(10);
  digitalWrite(pinoLed, LOW);

  Serial.println(F("--- Novo Ciclo de Leitura ---"));
  Serial.print(F("Leitura absoluta do sensor: "));
  Serial.println(altitude_absoluta_atual);

  // Verificação de validade dos dados (segunda camada de segurança)
  if (isnan(altitude_absoluta_atual) || altitude_absoluta_atual == 0.0) {
    contadorLeiturasInvalidas++;
    Serial.print(F("AVISO: Leitura invalida detectada! Contagem: "));
    Serial.println(contadorLeiturasInvalidas);
  } else {
    contadorLeiturasInvalidas = 0;
  }

  if (contadorLeiturasInvalidas >= 3) {
    finalizarMissao(F("3 LEITURAS INVALIDAS CONSECUTIVAS"));
  }

  buffer_leituras[indice_buffer] = altitude_absoluta_atual;
  indice_buffer = (indice_buffer + 1) % quantidade_leituras_media;

  float soma_buffer = 0.0;
  for (int i = 0; i < quantidade_leituras_media; i++) {
    soma_buffer += buffer_leituras[i];
  }
  ///////////////////////////
  float nova_media_continua = soma_buffer / quantidade_leituras_media;
  ///////////////////////////
  Serial.print(F("Media movel calculada: "));
  Serial.println(nova_media_continua);

  float altitude_relativa_media_movel = nova_media_continua - leitura_inicial_fixa;

  Serial.print(F(">>> ALTITUDE RELATIVA ATUAL: "));
  Serial.print(altitude_relativa_media_movel);
  Serial.println(F(" m <<<\n"));

  if (!foguete_lancado) {
    if (altitude_relativa_media_movel > ALTITUDE_DE_LANCAMENTO) {
      foguete_lancado = true;
      Serial.println(F(">>> LANCAMENTO DETECTADO! SISTEMA DE APOGEU ATIVADO. <<<"));
    }
  } else {
    if (primeira_medicao_pos_calibracao) {
      ultima_altitude_relativa = altitude_relativa_media_movel;
      primeira_medicao_pos_calibracao = false;
    } else {
      float delta_leitura_bruta = abs(altitude_relativa_media_movel - ultima_altitude_relativa);
      if (delta_leitura_bruta >= LIMITE_SALTO_ANOMALO) {
        finalizarMissao(F("SALTO DE LEITURA ANOMALO"));
      }
      ultima_altitude_relativa = altitude_relativa_media_movel;
    }

    verificaAltitude(altitude_relativa_media_movel);
  }
}

void finalizarMissao(const __FlashStringHelper* razao) {
  Serial.println(F(""));
  Serial.println(F("*************************************************"));
  Serial.print(F("FINALIZANDO MISSAO. MOTIVO: "));
  Serial.println(razao);
  Serial.println(F("*************************************************"));

  // Para qualquer tipo de falha, aciona a recuperação
  Serial.println(F("Acionando sistema de recuperacao (MOSFETs)..."));
  analogWrite(pinoMosfet1, 255);
  analogWrite(pinoMosfet2, 255);
  delay(3000);
  Serial.println(F("Sistema de recuperacao desligado."));
  analogWrite(pinoMosfet1, 0);
  analogWrite(pinoMosfet2, 0);


  Serial.println(F("Entrando em modo de localizacao (bipes continuos)..."));
  while (true) {
    digitalWrite(pinoBuzzer, HIGH);
    delay(300);
    digitalWrite(pinoBuzzer, LOW);
    delay(300);
  }
}

void verificaAltitude(float altitude) {
  if (altitude > apogeu) {
    apogeu = altitude;
    Serial.print(F("--- NOVO APOGEU REGISTRADO: "));
    Serial.print(apogeu);
    Serial.println(F(" m ---"));
  }

  if (apogeu > ALTITUDE_DE_LANCAMENTO) {
    bool condicao_voo_normal = (apogeu > 3 && altitude <= apogeu - 1);
    bool condicao_falha_motor = (apogeu <= 3 && altitude <= apogeu - 1);

    if (!acionamentoIniciado && (condicao_voo_normal || condicao_falha_motor)) {
      acionamentoIniciado = true;
      if (condicao_voo_normal) {
        finalizarMissao(F("ACIONAMENTO NORMAL POS-APOGEU"));
      } else {
        finalizarMissao(F("ACIONAMENTO DE SEGURANCA (BAIXO APOGEU)"));
      }
    }
  }
}