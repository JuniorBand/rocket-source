// CÓDIGO OTIMIZADO - FOGUETE RELAMPAGO MARQUINHOS
// Otimizações: Lógica não-bloqueante com millis(), tipos de dados eficientes e controle de serial.

#include <Wire.h>
#include <Adafruit_BMP280.h>

// --- OTIMIZAÇÃO: Usando 'const uint8_t' para valores fixos e pequenos. ---
// Isso economiza memória e ajuda o compilador a otimizar o código.
const uint8_t BUZZER_PIN = 4;
const uint8_t RELE_PIN = 9;
const uint8_t MEDIAS_INICIAIS = 15; // Número de leituras para calcular a média.

Adafruit_BMP280 bmp;

// --- Variáveis de Estado do Foguete ---
float apogeu = 0.0f;
float leitura_inicial = 0.0f;
bool releAcionado = false;
bool primeira_leitura = true;

// --- Variáveis de Controle ---
uint8_t contador_leituras = 0; // Otimização: uint8_t é suficiente (0-255)
float soma_altitudes = 0.0f;

// --- OTIMIZAÇÃO: Variáveis para a lógica não-bloqueante com millis() ---
unsigned long tempoAcionamentoRele = 0;
bool releEmContagem = false;
bool missaoFinalizada = false; // Flag para iniciar o buzzer final sem parar o código

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELE_PIN, OUTPUT);
  digitalWrite(RELE_PIN, LOW);

  // Sinal sonoro de inicialização
  digitalWrite(BUZZER_PIN, HIGH);
  delay(250);
  digitalWrite(BUZZER_PIN, LOW);

  if (!bmp.begin(0x77)) {
    Serial.println("Falha ao encontrar sensor BMP280, verificando novamente...");
    delay(3000);
    if (!bmp.begin(0x77)) {
      Serial.println("Falha crítica no BMP280!");
      // A função verificabmp() original entrava em loop infinito, parando tudo.
      // Agora, apenas sinalizamos o erro de forma contínua sem travar.
      missaoFinalizada = true; // Impede a lógica de voo de ser executada
    }
  }

  // Sinal sonoro de sistema pronto
  if (!missaoFinalizada) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void loop() {
  // Se a missão terminou (após acionar o relê ou por falha no sensor), apenas apita.
  if (missaoFinalizada) {
    sinalizarBuzzer(500, 500); // Apita a cada 1 segundo
    return; // Não executa o resto do código
  }

  // --- Lógica de voo principal ---
  float altitude = bmp.readAltitude(1013.25);

  // 1. Calibração da altitude inicial no solo
  if (primeira_leitura) {
    if (contador_leituras < MEDIAS_INICIAIS) {
      soma_altitudes += altitude;
      contador_leituras++;
      delay(20); // Um pequeno delay aqui é aceitável para espaçar as leituras de calibração
      return;
    } else {
      leitura_inicial = soma_altitudes / MEDIAS_INICIAIS;
      primeira_leitura = false;
      Serial.print("Calibracao finalizada. Altitude base: ");
      Serial.println(leitura_inicial);
    }
  }

  // 2. Cálculo da altitude relativa e detecção de apogeu
  altitude -= leitura_inicial;

  Serial.print("Altitude = ");
  Serial.print(altitude);
  Serial.println(" m");

  if (altitude > apogeu) {
    apogeu = altitude;
  }

  // 3. Verificação da condição de acionamento do relê (pós-apogeu)
  // A condição foi movida para dentro do loop para maior clareza.
  bool condicaoApogeuAlto = (apogeu > 200 && altitude <= 200);
  bool condicaoApogeuBaixo = (apogeu <= 200 && altitude <= apogeu - 5); // Aumentei a margem para 5m para mais segurança

  if ((condicaoApogeuAlto || condicaoApogeuBaixo) && !releAcionado) {
    Serial.println("ACIONANDO SISTEMA DE RECUPERACAO!");
    digitalWrite(RELE_PIN, HIGH);
    releAcionado = true;      // Garante que o relê só seja acionado uma vez
    releEmContagem = true;    // Inicia a contagem para desligar o relê
    tempoAcionamentoRele = millis(); // Armazena o momento exato do acionamento
  }

  // 4. OTIMIZAÇÃO: Lógica não-bloqueante para desligar o relê
  // Este bloco é verificado a cada loop, mas só faz algo quando a contagem está ativa.
  if (releEmContagem && (millis() - tempoAcionamentoRele > 5000)) { // 5000 ms = 5 segundos
    digitalWrite(RELE_PIN, LOW);
    releEmContagem = false;   // Para de verificar o tempo
    missaoFinalizada = true;  // A missão de voo ativo terminou, agora é só recuperar
    Serial.println("Recuperacao finalizada. Aguardando resgate.");
  }

  delay(100); // Controla a frequência do loop principal para ~10Hz
}

// Função auxiliar para o buzzer, para não repetir código
void sinalizarBuzzer(int tempoLigado, int tempoDesligado) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(tempoLigado);
  digitalWrite(BUZZER_PIN, LOW);
  delay(tempoDesligado);
}