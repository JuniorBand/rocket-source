#include <Wire.h>
#include <Adafruit_BMP280.h>

/* Definições de pinos */
#define pinoBuzzer 11
#define pinoRele 9

/* Limites e configurações de voo */
const float LIMITE_SALTO_ANOMALO = 75.0;
const float ALTITUDE_DE_LANCAMENTO = 30.0; // Altitude para considerar o foguete lançado.

Adafruit_BMP280 bmp;

float apogeu = 0;
bool releAcionado = false;

/* Variáveis para a calibração inicial e média móvel */
float leitura_inicial_fixa = 0;
bool calibracao_inicial_concluida = false;

/* Parâmetros e buffer para a média móvel */
const int quantidade_leituras_media = 5;
float buffer_leituras[quantidade_leituras_media];
int indice_buffer = 0;
int contagem_leituras_pre_calibracao = 0;

/* Variáveis para detectar saltos de altitude e estado de voo */
float ultima_altitude_relativa = 0.0f;
bool primeira_medicao_pos_calibracao = true;
int contador_leituras_zero = 0;
bool foguete_lancado = false; // Flag para controlar o estado da missão.

/* Protótipos das funções */
void verificaAltitude(float altitude);
void escanearI2C();
void finalizarMissao(const __FlashStringHelper *razao);


void setup() {
  Serial.begin(115200);
  pinMode(pinoBuzzer, OUTPUT);
  pinMode(pinoRele, OUTPUT);
  digitalWrite(pinoRele, LOW);

  digitalWrite(pinoBuzzer, HIGH); delay(250); digitalWrite(pinoBuzzer, LOW); delay(250);
  Wire.begin();

  Serial.println(F("Tentando inicializar BMP280 no endereco 0x76..."));
  if (!bmp.begin(0x76)) {
    Serial.println(F("================================================="));
    Serial.println(F("ERRO CRÍTICO: Nao foi possivel encontrar um sensor BMP280 valido no 0x76!"));
    escanearI2C();
    finalizarMissao(F("ERRO CRITICO NA INICIALIZACAO DO SENSOR"));
  } else {
    Serial.println(F("BMP280 encontrado com sucesso no endereco 0x76!"));
  }
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);

  Serial.println(F("Iniciando fase de calibracao de altitude..."));

  digitalWrite(pinoBuzzer, HIGH); delay(250); digitalWrite(pinoBuzzer, LOW); delay(250);
  digitalWrite(pinoBuzzer, HIGH); delay(250); digitalWrite(pinoBuzzer, LOW);
}

void loop() {
  float altitude_absoluta_atual = bmp.readAltitude(1013.25);

  /* VERIFICAÇÕES DE VALIDADE DA LEITURA */
  if (isnan(altitude_absoluta_atual)) {
    finalizarMissao(F("ERRO DE LEITURA DO SENSOR (NaN)"));
  }
  if (altitude_absoluta_atual == 0.0) {
    contador_leituras_zero++;
  } else {
    contador_leituras_zero = 0;
  }
  if (contador_leituras_zero >= 3) {
    finalizarMissao(F("TRES LEITURAS ZERADAS CONSECUTIVAS"));
  }
  
  if (!calibracao_inicial_concluida) {
    // --- Lógica de Calibração ---
    buffer_leituras[indice_buffer] = altitude_absoluta_atual;
    indice_buffer = (indice_buffer + 1) % quantidade_leituras_media;
    if (contagem_leituras_pre_calibracao < quantidade_leituras_media) {
      contagem_leituras_pre_calibracao++;
    }
    if (contagem_leituras_pre_calibracao == quantidade_leituras_media) {
      float soma_buffer_calibracao = 0;
      for (int i = 0; i < quantidade_leituras_media; i++) {
        soma_buffer_calibracao += buffer_leituras[i];
      }
      leitura_inicial_fixa = soma_buffer_calibracao / quantidade_leituras_media;
      calibracao_inicial_concluida = true;
      Serial.println(F("================================================="));
      Serial.print(F("Calibracao concluida. Altitude de referencia (zero): "));
      Serial.print(leitura_inicial_fixa);
      Serial.println(F(" m."));
      Serial.println(F("AGUARDANDO LANCAMENTO..."));
      Serial.println(F("================================================="));
    }
    delay(100); 
    return;
  }
  
  /* --- Fase de Operação Normal (Após a Calibração Inicial) --- */

  buffer_leituras[indice_buffer] = altitude_absoluta_atual;
  indice_buffer = (indice_buffer + 1) % quantidade_leituras_media;
  
  float soma_buffer = 0;
  for (int i = 0; i < quantidade_leituras_media; i++) {
    soma_buffer += buffer_leituras[i];
  }
  float nova_media_continua = soma_buffer / quantidade_leituras_media;
  float altitude_relativa_media_movel = nova_media_continua - leitura_inicial_fixa;

  Serial.print(F("Altitude = "));
  Serial.print(altitude_relativa_media_movel);
  Serial.println(F(" m"));

  /* --- Lógica de voo dividida por estado (Aguardando/Em Voo) --- */

  // Se o foguete ainda não foi lançado, apenas verifica se ultrapassou a altitude de lançamento.
  if (!foguete_lancado) {
    if (altitude_relativa_media_movel > ALTITUDE_DE_LANCAMENTO) {
      foguete_lancado = true;
      Serial.println(F(">>> LANCAMENTO DETECTADO! ARMANDO SISTEMA DE APOGEU. <<<"));
    }
  } 
  // Se o foguete já foi lançado, executa a lógica de voo normal.
  else {
    // Filtro de Salto Anômalo
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

    // Atualiza o apogeu
    if (altitude_relativa_media_movel > apogeu) {
      apogeu = altitude_relativa_media_movel;
    }

    // Verifica a condição de ejeção
    verificaAltitude(altitude_relativa_media_movel);
  }
}

void finalizarMissao(const __FlashStringHelper *razao) {
  Serial.println(F(""));
  Serial.println(F("================================================="));
  Serial.print(F("FINALIZANDO MISSAO. MOTIVO: "));
  Serial.println(razao);
  Serial.println(F("================================================="));

  digitalWrite(pinoRele, HIGH);
  delay(3000);
  digitalWrite(pinoRele, LOW);

  while (true) {
    digitalWrite(pinoBuzzer, HIGH);
    delay(300);
    digitalWrite(pinoBuzzer, LOW);
    delay(300);
  }
}

void verificaAltitude(float altitude) {
  // Condição ajustada para o foguete de alto desempenho.
  if (!releAcionado && (apogeu > 100 && altitude <= apogeu - 20)) {
    releAcionado = true;
    finalizarMissao(F("ACIONAMENTO NORMAL POS-APOGEU"));
  }
}

void escanearI2C() {
  byte erro, endereco;
  int numDispositivos;
  Serial.println(F("Iniciando Escaner I2C..."));
  numDispositivos = 0;
  for (endereco = 1; endereco < 127; endereco++) {
    Wire.beginTransmission(endereco);
    erro = Wire.endTransmission();
    if (erro == 0) {
      Serial.print(F("Dispositivo I2C encontrado no endereco 0x"));
      if (endereco < 16) { Serial.print(F("0")); }
      Serial.print(endereco, HEX);
      Serial.println(F("   !"));
      numDispositivos++;
    } else if (erro == 4) {
      Serial.print(F("Erro desconhecido no endereco 0x"));
      if (endereco < 16) { Serial.print(F("0")); }
      Serial.println(endereco, HEX);
    }
  }
  if (numDispositivos == 0) {
    Serial.println(F("Nenhum dispositivo I2C encontrado."));
  }
  Serial.println(F("================================================="));
  delay(1000);
}