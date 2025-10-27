// CÓDIGO MODIFICADO DO FOGUETE RELAMPAGO MARQUINHOS:
// Versão otimizada mantendo a estrutura original.
// Otimização principal: Substituição do delay(5000) por lógica não-bloqueante.

#include <Wire.h>
#include <Adafruit_BMP280.h>

// OTIMIZAÇÃO: Usando 'const uint8_t' para pinos e valores fixos.
// Isso economiza um pouco de memória e é uma boa prática de programação.
const uint8_t buzzer = 4;
const uint8_t relePin = 9;

Adafruit_BMP280 bmp;

// --- Variáveis de estado (sem alterações) ---
float apogeu = 0;
bool releAcionado = false;
float leitura_inicial = 0;
bool primeira_leitura = true;
float soma_altitudes = 0;
const int medias = 15; // Mantido como 'int' para máxima compatibilidade com o original.
uint8_t contador = 0; // OTIMIZAÇÃO: 'uint8_t' é mais eficiente que 'int' para um contador que vai até 15.

// OTIMIZAÇÃO: Variáveis para controle de tempo não-bloqueante
bool releEmContagem = false;      // Flag para saber se o relê está no período de 5s ativo
unsigned long tempoInicioRele = 0; // Armazena o tempo em que o relê foi ligado

// Declaração das funções para que o compilador as encontre
void verificabmp(void);
void verificaAltitude(float altitude);

void setup() {
  Serial.begin(9600);
  pinMode(buzzer, OUTPUT);
  pinMode(relePin, OUTPUT);
  digitalWrite(relePin, LOW);

  digitalWrite(buzzer, HIGH);
  delay(250);
  digitalWrite(buzzer, LOW);

  if (!bmp.begin(0x77)) {
    Serial.println("Não foi possível encontrar um sensor BMP280 válido, verifique a fiação!");
    delay(3000);
    if (!bmp.begin(0x77)) {
      verificabmp(); // Esta função intencionalmente trava o código em caso de falha, mantido original.
    }
  }

  digitalWrite(buzzer, HIGH);
  delay(250);
  digitalWrite(buzzer, LOW);
  digitalWrite(buzzer, HIGH);
  delay(250);
  digitalWrite(buzzer, LOW);
}

void loop() {
  float altitude = bmp.readAltitude(1013.25);

  if (contador < medias) {
    soma_altitudes += altitude;
    contador++;
    return;
  } else if (primeira_leitura) {
    leitura_inicial = soma_altitudes / medias;
    primeira_leitura = false;
  }

  altitude -= leitura_inicial;

  Serial.print("Altitude = "); Serial.print(altitude); Serial.println(" m");

  if (altitude > apogeu) {
    apogeu = altitude;
  }

  // A chamada para a função e a lógica do loop permanecem idênticas.
  // A otimização está dentro da função 'verificaAltitude'.
  verificaAltitude(altitude);

  delay(10);
}

void verificaAltitude(float altitude) {
  // A condição de acionamento é a mesma do código original.
  bool condicaoDeDescida = (apogeu > 200 && altitude <= 200) || (apogeu < 200 && altitude <= apogeu - 3);

  // OTIMIZAÇÃO - Início: Lógica não-bloqueante
  // Etapa 1: Aciona o relê e inicia o contador de tempo (só acontece uma vez)
  if (condicaoDeDescida && !releAcionado) {
    digitalWrite(relePin, HIGH);   // Liga o relê
    releAcionado = true;           // Garante que não será acionado de novo
    releEmContagem = true;         // Informa ao código que o tempo de 5s começou
    tempoInicioRele = millis();    // Grava o momento exato da ligação
  }

  // Etapa 2: Verifica continuamente se os 5 segundos já passaram
  if (releEmContagem && (millis() - tempoInicioRele >= 5000)) {
    digitalWrite(relePin, LOW);    // Desliga o relê após 5 segundos
    releEmContagem = false;        // Para a verificação do tempo

    // Etapa 3: Entra no loop infinito de bipe, como no código original.
    // O comportamento final é idêntico: o sistema para e fica apitando.
    while (true) {
      digitalWrite(buzzer, HIGH);
      delay(1000);
      digitalWrite(buzzer, LOW);
      delay(1000);
    }
  }
  // OTIMIZAÇÃO - Fim
}

// Esta função não foi alterada. Seu propósito é travar o programa em caso de
// falha crítica na inicialização, então o uso de 'delay' e 'while(true)' é apropriado aqui.
void verificabmp(void) {
  delay (4000);
  if (!releAcionado) {
    digitalWrite(relePin, HIGH);
    delay(2000);
    digitalWrite(relePin, LOW);
    releAcionado = true;
    while (true) {
      digitalWrite(buzzer, HIGH);
      delay(500);
      digitalWrite(buzzer, LOW);
      delay(500);
    }
  }
}