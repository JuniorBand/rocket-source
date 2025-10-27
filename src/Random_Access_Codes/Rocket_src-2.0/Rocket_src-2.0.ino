#include <Wire.h>
#include <Adafruit_BMP280.h>

#define buzzer 11
#define relePin 9

Adafruit_BMP280 bmp;

// ====== PARÂMETROS AJUSTÁVEIS ===== 
float apogeuEscolhido = 5.0;          // Altura de apogeu desejado
float limiteAlturaEspera = 1.0;       // Altura mínima para iniciar contagem de tempo
float quedaApogeu = 3.0;              // Distância que caiu após apogeu para acionar
float tempo_de_espera = 5000;         // Tempo em milissegundos

// ====== CONTROLE DE ESTADO ======
float leitura_inicial = 0;
float apogeuRegistrado = 0;
bool primeiraLeitura = true;
bool releAcionado = false;
bool passouLimite = false;
unsigned long tempoUltrapassouLimite = 0;

int contador = 0;
float somaAltitudes = 0;
const int medias = 15;

void acionarReleEBuzzer() {
  if (!releAcionado) {
    digitalWrite(relePin, LOW); // Aciona o relé
    releAcionado = true;

    while (true) { // Buzzer contínuo
      digitalWrite(buzzer, HIGH);
      delay(1000);
      digitalWrite(buzzer, LOW);
      delay(1000);
    }
  }
}

void apenasBuzzerErroSensor() {
  while (true) {
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    delay(500);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(relePin, OUTPUT);
  digitalWrite(relePin, HIGH); // Desliga o relé
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 não encontrado.");
    delay(2000);
    apenasBuzzerErroSensor(); // Apenas buzina
  }

  digitalWrite(buzzer, HIGH);
  delay(500);
  digitalWrite(buzzer, LOW);
}

void loop() {
  float altitude = bmp.readAltitude(1013.25);

  if (contador < medias) {
    somaAltitudes += altitude;
    contador++;
    return;
  } else if (primeiraLeitura) {
    leitura_inicial = somaAltitudes / medias;
    primeiraLeitura = false;
  }

  altitude -= leitura_inicial;
  Serial.print("Altitude ajustada: ");
  Serial.print(altitude);
  Serial.println(" m");

  if (altitude > apogeuRegistrado) {
    apogeuRegistrado = altitude;
  }

  // Condição 1: atingiu ou passou o apogeu escolhido
  if (altitude >= apogeuEscolhido && !releAcionado) {
    acionarReleEBuzzer();
  }

  // Condição 2: caiu quedaApogeu metros após o maior ponto registrado
  if (!releAcionado && apogeuRegistrado < apogeuEscolhido && altitude <= apogeuRegistrado - quedaApogeu) {
    acionarReleEBuzzer();
  }

  // Condição 3: passou da altura mínima e permaneceu por tempo_de_espera
  if (altitude > limiteAlturaEspera) {
    if (!passouLimite) {
      passouLimite = true;
      tempoUltrapassouLimite = millis();
    } else if (millis() - tempoUltrapassouLimite >= tempo_de_espera && !releAcionado) {
      acionarReleEBuzzer();
    }
  }

  delay(100);
}