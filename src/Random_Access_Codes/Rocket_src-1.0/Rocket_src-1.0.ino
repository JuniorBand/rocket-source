#include <Wire.h> // Inclui a biblioteca para comunicação I2C.
#include <Adafruit_BMP280.h> // Inclui a biblioteca para o sensor de pressão BMP280.

#define buzzer 4 // Define o pino 4 como o pino do buzzer.
#define relePin 9 // Define o pino 9 como o pino do relé.

Adafruit_BMP280 bmp; // Cria um objeto para o sensor BMP280.

float apogeu = 0; // Variável para armazenar o valor máximo de altitude alcançada.
bool releAcionado = false; // Variável de controle para saber se o relé já foi acionado.
float leitura_inicial = 0; // Variável para armazenar a leitura inicial da altitude.
bool primeira_leitura = true; // Variável de controle para saber se é a primeira leitura.
int contador = 0; // Contador de leituras para média de altitude.
float soma_altitudes = 0; // Soma das altitudes lidas para cálculo da médias.
const int medias = 15; // Número de leituras para calcular a média.

void verificabmp(void){
    delay (4000); // Aguarda 4 segundos.

    if (!releAcionado) { // Se o relé não foi acionado:
      digitalWrite(relePin, LOW);//Aciona o relé.
      delay(1000); // Aguarda 1 segundo.
      digitalWrite(relePin, HIGH);//Desliga o relé.
      delay(1000); // Aguarda 1 segundo.
      digitalWrite(relePin, LOW);//Aciona o relé.
      releAcionado = true; // Marca o relé como acionado.

      while (true) { // Loop infinito para acionar o buzzer.
        digitalWrite(buzzer, HIGH); // Liga o buzzer.
        delay(1000); // Aguarda 1 segundo.
        digitalWrite(buzzer, LOW); // Desliga o buzzer.
        delay(1000); // Aguarda 1 segundo.
     }
    }    
  }

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial a 9600 bps.
  pinMode(relePin, OUTPUT); // Define o pino do relé como saída.
  digitalWrite(relePin, HIGH);// Confirma que o relé não está acionado (supondo NO desligado em HIGH) / caso seja o contrário, troque os HIGH p/ LOW e vice-versa.
  pinMode(buzzer, OUTPUT); // Define o pino do buzzer como saída.

  if (!bmp.begin(0x77)) { // Se o sensor BMP280 não for encontrado:
    Serial.println("Não foi possível encontrar um sensor BMP280 válido, verifique a fiação!"); // Mensagem de erro.
    delay(2000); // Aguarda 2 segundos.
    if (!bmp.begin(0x77)) { // Tenta inicializar novamente o sensor.
      verificabmp(); // Chama a função para acionar o relé e o buzzer.
    }
  }

  digitalWrite(buzzer, HIGH); // Liga o buzzer por 250 ms para indicar a inicialização.
  delay(250);
  digitalWrite(buzzer, LOW);
}

void verificaAltitude(float altitude) {
  if ((apogeu > 200 && altitude <= 200) || (apogeu < 200 && altitude <= apogeu - 3)) { // Verifica se a altitude atingiu o apogeu e está descendo.
    
    
    if (!releAcionado) { // Se o relé não foi acionado:
      digitalWrite(relePin, LOW);//Aciona o relé.   
      delay(1000); // Aguarda 1 segundo.
      digitalWrite(relePin, HIGH);//Desliga o relé.
      delay(1000); // Aguarda 1 segundo.
      digitalWrite(relePin, LOW);//Aciona o relé.
      releAcionado = true; // Marca o relé como acionado.

      while (true) { // Loop infinito para acionar o buzzer.
        digitalWrite(buzzer, HIGH); // Liga o buzzer.
        delay(1000); // Aguarda 1 segundo.
        digitalWrite(buzzer, LOW); // Desliga o buzzer.
        delay(1000); // Aguarda 1 segundo.
      }
    }
  
  }
}

void loop() {
  float altitude = bmp.readAltitude(1013.25); // Lê a altitude do sensor BMP280.

  if (contador < medias) { // Se o número de leituras for menor que o definido:
    soma_altitudes += altitude; // Soma as leituras de altitude.
    contador++; // Incrementa o contador.
    return; // Sai da função loop.
  } else if (primeira_leitura) { // Se for a primeira leitura:
    leitura_inicial = soma_altitudes / medias; // Calcula a média das leituras iniciais.
    primeira_leitura = false; // Marca que a primeira leitura foi feita.
  }

  altitude -= leitura_inicial; // Ajusta a altitude subtraindo a leitura inicial.

  Serial.print("Altitude = "); Serial.print(altitude); Serial.println(" m"); // Imprime a altitude ajustada no serial.

  if (altitude > apogeu) { // Se a altitude atual for maior que o apogeu registrado:
    apogeu = altitude; // Atualiza o valor do apogeu.
  }


  verificaAltitude(altitude); // Verifica se a altitude atingiu o apogeu e está descendo.

  delay(10); // Aguarda 10 ms antes de repetir o loop.
}