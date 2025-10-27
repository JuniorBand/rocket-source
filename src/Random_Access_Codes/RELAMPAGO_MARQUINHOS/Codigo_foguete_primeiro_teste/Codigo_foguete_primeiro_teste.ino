// CÓDIGO MODIFICADO DO FOGUETE RELAMPAGO MARQUINHOS:
// *Modificações: Substituição do servo motor por módulo relê (mesmo pino) e remoção de toda lógica do cartão SD.*

#include <Wire.h> // Inclui a biblioteca para comunicação I2C.
#include <Adafruit_BMP280.h> // Inclui a biblioteca para o sensor de pressão BMP280.
//#include <Servo.h> // *[REMOVIDO]* Biblioteca Servo removida pois o servo não é mais utilizado.
//#include <SD.h> // *[REMOVIDO]* Biblioteca SD removida pois o módulo SD não é mais utilizado.

#define buzzer 4 // Define o pino 4 como o pino do buzzer.
#define relePin 9 // *[SUBSTITUIÇÃO]* Define o pino 9 como o pino do módulo relê (antes usado pelo servo).
//#define chipSelect 10 *[REMOVIDO]* Pino do SD removido pois o módulo SD não é mais utilizado.

Adafruit_BMP280 bmp; // Cria um objeto para o sensor BMP280.
//Servo servo; // *[REMOVIDO]* Objeto Servo removido pois não é mais utilizado.
//File dataFile; // *[REMOVIDO]* Objeto File removido pois não é mais utilizado.

float apogeu = 0; // Variável para armazenar o valor máximo de altitude alcançada.
bool releAcionado = false; // *[SUBSTITUIÇÃO]* Variável de controle para saber se o relê já foi acionado (antes: servoAcionado).
float leitura_inicial = 0; // Variável para armazenar a leitura inicial da altitude.
bool primeira_leitura = true; // Variável de controle para saber se é a primeira leitura.
int contador = 0; // Contador de leituras para média de altitude.
float soma_altitudes = 0; // Soma das altitudes lidas para cálculo da média.
const int medias = 15; // Número de leituras para calcular a média.

void verificabmp(void);
void verificaAltitude(float altitude);

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial a 9600 bps.
  pinMode(buzzer, OUTPUT); // Define o pino do buzzer como saída.
  pinMode(relePin, OUTPUT); // *[SUBSTITUIÇÃO]* Define o pino do relê como saída (antes: servo.attach()).
  digitalWrite(relePin, LOW); // *[SUBSTITUIÇÃO]* Garante que o relê inicia desligado (antes: servo.write(0)).


  digitalWrite(buzzer, HIGH); // Liga o buzzer por 250 ms para indicar a inicialização.
  delay(250);
  digitalWrite(buzzer, LOW);

  if (!bmp.begin(0x77)) { // Se o sensor BMP280 não for encontrado:
    Serial.println("Não foi possível encontrar um sensor BMP280 válido, verifique a fiação!"); // Mensagem de erro.
    delay(3000); // Aguarda 3 segundos.
    if (!bmp.begin(0x77)) { // Tenta inicializar novamente o sensor.
      verificabmp(); // Chama a função para acionar o relê e o buzzer.
    }
  }

  /* if (!SD.begin(chipSelect)) { // *[REMOVIDO]* Inicialização do cartão SD removida.
       Serial.println("Falha na inicialização do cartão SD!"); // Mensagem de erro.
     }*/

  digitalWrite(buzzer, HIGH); 
  delay(250);
  digitalWrite(buzzer, LOW);
  digitalWrite(buzzer, HIGH); // Liga o buzzer por mais 2x para indicar a inicialização completa.
  delay(250);
  digitalWrite(buzzer, LOW);
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
    /* *[REMOVIDO]* Gravação do apogeu no SD removida.
       if (SD.exists("dados.txt")) {
         dataFile = SD.open("dados.txt", FILE_WRITE);
       if (dataFile) {
           dataFile.println("Apogeu: " + String(apogeu));
           dataFile.close();
         }
       }
  }*/

  /* *[REMOVIDO]* Gravação dos dados de voo no SD removida.
     if (SD.exists("dados.txt")) {
     dataFile = SD.open("dados.txt", FILE_WRITE);
       if (dataFile) {
         dataFile.print("Altitude: " + String(altitude) + ", ");
         dataFile.print("Pressão: " + String(bmp.readPressure()) + ", ");
         dataFile.println("Temperatura: " + String(bmp.readTemperature()));
         dataFile.close();
       }
     }*/

  verificaAltitude(altitude); // Verifica se a altitude atingiu o apogeu e está descendo.

  delay(10); // Aguarda 10 ms antes de repetir o loop.
  }
}
void verificaAltitude(float altitude) {
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
}
void verificabmp(void){
    delay (4000); // Aguarda 4 segundos.
    if (!releAcionado) { // *[SUBSTITUIÇÃO]* Controle do relê no lugar do servo.
      digitalWrite(relePin, HIGH); // *[SUBSTITUIÇÃO]* Aciona o relê (antes: servo.write(90)).
      delay(2000); // Aguarda 2 segundos.
      digitalWrite(relePin, LOW); // *[SUBSTITUIÇÃO]* Desliga o relê (antes: servo.write(0)).
      releAcionado = true; // Marca o relê como acionado.
      while (true) { // Loop infinito para acionar o buzzer.
        digitalWrite(buzzer, HIGH); // Liga o buzzer.
        delay(500); // Aguarda 0,5 segundo.
        digitalWrite(buzzer, LOW); // Desliga o buzzer.
        delay(500); // Aguarda 0,5 segundo.
     }
    }    
  }