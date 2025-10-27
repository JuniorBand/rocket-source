// Versão: Rocket_src-3.0
// *Modificações: substituição do relé por dois mosfets, modularização e nova organização na declaração de variáveis e funções.

#include <Wire.h> // Inclui a biblioteca para comunicação I2C.
#include <Adafruit_BMP280.h> // Inclui a biblioteca para o sensor de pressão BMP280.


// ====== PARÂMETROS AJUSTÁVEIS =====
// *[ALTERAÇÃO/SUBSTITUIÇÃO]* Os #define foram substituidos por uint8_t (0-255) por motivos de segurança, mesmo ocupando um pequeno espaço.
const uint8_t BUZZER_PIN =  11;       // Pino do buzzer.
const uint8_t MOSFET_PIN1 = 13;       //  *[ALTERAÇÃO/SUBSTITUIÇÃO]* Saída PWM do mosfet 1.
const uint8_t MOSFET_PIN2 = 12;       //  *[ALTERAÇÃO/SUBSTITUIÇÃO]* Saída PWM do mosfet 2.
const float QUEDA = 3.0f;             //  *[ALTERAÇÃO (declaração da queda mínima de segurança)]* Distância de segurança que caiu (antes do apogeu desejado) para acionar o paraquedas.
const float APOGEU = 5.0f;            // *[ALTERAÇÃO]* Armazena a meta de apogeu desejado.
const uint8_t MEDIAS = 15;            // Número de leituras para calcular a média.

// ====== CONTROLE DE ESTADO ======
// *[REMOVIDO]* bool primeiraLeitura = true; // Indica se é a primeira leitura.
// *[REMOVIDO]* float apogeu = 0; // Variável para armazenar o valor máximo de altitude alcançada.
Adafruit_BMP280 bmp;                 // Cria um objeto para o sensor BMP280.
float leitura_inicial = 0.0f;        // Armazena a leitura inicial da altitude.
float alturaMax = 0.0f;              // Armazena o valor máximo de altitude alcançada.
bool mosfetAcionado = false;         // Indica se os mosfets já foram acionados.
uint8_t contador = 0;                // Contador de leituras para média de altitude.
float somaAltitudes = 0.0f;          // Soma das altitudes lidas para cálculo da médias.

// ====== FUNÇÕES ======  
// *[ALTERAÇÃO]* Declaração das funções.
// *[REMOVIDO]* void acionarBuzzer(); // Buzzer contínuo.
void acionarMosfet(void);                     // Acionar mosfets.
void acionarBuzzer(int timeOn, int timeOff);  // Buzzer contínuo de timeOn em timeOff ms.
void verificarAltitude(float altitude);       // Verifica se o foguete chegou ao apogeu.

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT); // Define o pino do buzzer como saída.
  digitalWrite(BUZZER_PIN, LOW); // Desliga o buzzer.
  pinMode(MOSFET_PIN1, OUTPUT); // *[ALTERAÇÃO]* Define o mosfetPin1 como saída.
  digitalWrite(MOSFET_PIN1, LOW); // Desliga o mosfet 1.
  pinMode(MOSFET_PIN2, OUTPUT); // *[ALTERAÇÃO]* Define o mosfetPin2 como saída.
  digitalWrite(MOSFET_PIN2, LOW); // Desliga o mosfet 2.
  
  digitalWrite(BUZZER_PIN, HIGH); // Liga o buzzer por 250 ms para indicar a inicialização.
  delay(250);
  digitalWrite(BUZZER_PIN, LOW);

  if (!bmp.begin(0x76)) { // Se o sensor BMP280 não for encontrado:
    Serial.println("BMP280 não encontrado.");
    delay(2000);
    acionarBuzzer(500, 500);
  }

  digitalWrite(BUZZER_PIN, HIGH); // Liga o buzzer por mais 2x para indicar a inicialização completa.
  delay(250);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(250);
  digitalWrite(BUZZER_PIN, LOW);

  }


void loop() {
  float altitude = bmp.readAltitude(1013.25);

  if (contador < MEDIAS) { // Se o número de leituras for menor que o definido:
    somaAltitudes += altitude; // Soma as primeiras leituras de altitude.
    contador++; // Incrementa o contador de 0 até 14.
    return;
  } else if (leitura_inicial == 0.0f) { // *[ALTERAÇÃO]* Define altura inicial.
    leitura_inicial = somaAltitudes / MEDIAS; // Calcula a média das leituras iniciais.
    // *[REMOVIDO]* primeiraLeitura = false; // Marca primeira leitura como concluída.
  }

  altitude -= leitura_inicial; // Ajusta a altitude subtraindo da leitura inicial.
  Serial.print("Altitude ajustada: ");
  Serial.print(altitude); // Imprime altitude no serial.
  Serial.println(" m");

  if (altitude > alturaMax) { // Armazena cada altura crescente até o apogeu.
    alturaMax = altitude; // Nova altura máxima definida.
  }

  verificarAltitude(altitude); // Verifica se pode acionar o paraquedas.

  delay(100); // Delay de 100 ms antes de repetir o loop.
}

void verificarAltitude(float altitude) { // Verifica se a altitude atingiu o apogeu e está descendo (com uma margem de erro de QUEDA).
  
/* *[REMOVIDO]* // (lembrando que apogeu dessa função removida equivale a alturaMax)
  if ((apogeu > 200 && altitude <= 200) || (apogeu < 200 && altitude <= apogeu - 3)) { // Verifica se a altitude atingiu o apogeu e está descendo ( Com uma margem de erro de 3m )
    if (!releAcionado) { // *[SUBSTITUIÇÃO]* Controle do relê no lugar do servo.
      digitalWrite(relePin, HIGH); // *[SUBSTITUIÇÃO]* Aciona o relê (antes: servo.write(90)).
      delay(5000); // Aguarda 5 segundos.
      digitalWrite(relePin, LOW); // *[SUBSTITUIÇÃO]* Desliga o relê (antes: servo.write(0)).
      releAcionado = true; // Marca o relê como acionado.

      while (true) { // Loop infinito para acionar o buzzer.
        digitalWrite(buzzer, HIGH); // Liga o buzzer.
        delay(1000); // Aguarda 1 segundo.
        digitalWrite(buzzer, LOW); // Desliga o buzzer.
        delay(1000); // Aguarda 1 segundo.
      }
    }
  }
*/
  
  if ((alturaMax > APOGEU && altitude <= APOGEU) || (alturaMax < APOGEU && altitude <= alturaMax - QUEDA)) {
    
    acionarMosfet(); // *[ALTERAÇÃO]* Aciona os mosfets e o paraquedas.
    
  }
}

void acionarMosfet(void) {  //  *[ALTERAÇÃO/SUBSTITUIÇÃO]* Função para acionar mosfets.
  if (!mosfetAcionado) { // Redundância.
    digitalWrite(MOSFET_PIN1, HIGH); // Aciona o mosfet 1.
    digitalWrite(MOSFET_PIN2, HIGH); // Aciona o mosfet 2.
    delay(5000); // Aguarda 5 segundos.
    digitalWrite(MOSFET_PIN1, LOW); // Desliga o mosfet 1.
    digitalWrite(MOSFET_PIN2, LOW); // Desliga o mosfet 2.
    
    mosfetAcionado = true; // Marca os mosfets como acionados.

    acionarBuzzer(1000, 1000); // Aciona buzzer de timeOn em timeOff ms.
  }
}

void acionarBuzzer(int timeOn, int timeOff) { //  *[ALTERAÇÃO]* Buzzer contínuo de timeOn em timeOff ms.
  
  while(true) {
    digitalWrite(BUZZER_PIN, HIGH); // Aciona o buzzer.
    delay(timeOn);
    digitalWrite(BUZZER_PIN, LOW); // Desliga o buzzer.
    delay(timeOff);
  }
  
}
