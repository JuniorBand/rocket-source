//CÓDIGO FINAL DO FOGUETE RELAMPAGO MARQUINHOS:

#include <Wire.h> // Inclui a biblioteca para comunicação I2C.
#include <Adafruit_BMP280.h> // Inclui a biblioteca para o sensor de pressão BMP280.
#include <Servo.h> // Inclui a biblioteca para controle de servos.
#include <SD.h> // Inclui a biblioteca para leitura/escrita em cartão SD.

#define buzzer 4 // Define o pino 4 como o pino do buzzer.
#define servoPin 9 // Define o pino 9 como o pino do servo.
#define chipSelect 10 // Define o pino 10 como o pino de seleção do chip do cartão SD.

Adafruit_BMP280 bmp; // Cria um objeto para o sensor BMP280.
Servo servo; // Cria um objeto para o servo.
File dataFile; // Cria um objeto para o arquivo de dados no cartão SD.

float apogeu = 0; // Variável para armazenar o valor máximo de altitude alcançada.
bool servoAcionado = false; // Variável de controle para saber se o servo já foi acionado.
float leitura_inicial = 0; // Variável para armazenar a leitura inicial da altitude.
bool primeira_leitura = true; // Variável de controle para saber se é a primeira leitura.
int contador = 0; // Contador de leituras para média de altitude.
float soma_altitudes = 0; // Soma das altitudes lidas para cálculo da média.
const int medias = 15; // Número de leituras para calcular a média.

void verificabmp(void){
    delay (4000); // Aguarda 4 segundos.
    if (!servoAcionado) { // Se o servo não foi acionado:
      servo.write(90); // Move o servo para 90 graus.
      delay(2000); // Aguarda 2 segundos.
      servo.write(0); // Retorna o servo para 0 graus.
      servoAcionado = true; // Marca o servo como acionado.
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
  pinMode(buzzer, OUTPUT); // Define o pino do buzzer como saída.
  servo.attach(servoPin); // Anexa o servo ao pino especificado.
  servo.write(0); // Inicializa o servo na posição 0.

  if (!bmp.begin(0x77)) { // Se o sensor BMP280 não for encontrado:
    Serial.println("Não foi possível encontrar um sensor BMP280 válido, verifique a fiação!"); // Mensagem de erro.
    delay(2000); // Aguarda 2 segundos.
    if (!bmp.begin(0x77)) { // Tenta inicializar novamente o sensor.
      verificabmp(); // Chama a função para acionar o servo e o buzzer.
    }
  }

  if (!SD.begin(chipSelect)) { // Se a inicialização do cartão SD falhar:
    Serial.println("Falha na inicialização do cartão SD!"); // Mensagem de erro.
  }

  digitalWrite(buzzer, HIGH); // Liga o buzzer por 250 ms para indicar a inicialização.
  delay(250);
  digitalWrite(buzzer, LOW);
}

void verificaAltitude(float altitude) {
  if ((apogeu > 200 && altitude <= 200) || (apogeu < 200 && altitude <= apogeu - 3)) { // Verifica se a altitude atingiu o apogeu e está descendo.
    if (!servoAcionado) { // Se o servo não foi acionado:
      servo.write(90); // Move o servo para 90 graus.
      delay(5000); // Aguarda 5 segundos.
      servo.write(0); // Retorna o servo para 0 graus.
      servoAcionado = true; // Marca o servo como acionado.

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
    if (SD.exists("dados.txt")) { // Se o arquivo "dados.txt" existir no cartão SD:
      dataFile = SD.open("dados.txt", FILE_WRITE); // Abre o arquivo para escrita.
      if (dataFile) { // Se o arquivo foi aberto com sucesso:
        dataFile.println("Apogeu: " + String(apogeu)); // Registra o apogeu no arquivo.
        dataFile.close(); // Fecha o arquivo.
      }
    }
  }

  if (SD.exists("dados.txt")) { // Se o arquivo "dados.txt" existir no cartão SD:
    dataFile = SD.open("dados.txt", FILE_WRITE); // Abre o arquivo para escrita.
    if (dataFile) { // Se o arquivo foi aberto com sucesso:
      dataFile.print("Altitude: " + String(altitude) + ", "); // Registra a altitude no arquivo.
      dataFile.print("Pressão: " + String(bmp.readPressure()) + ", "); // Registra a pressão no arquivo.
      dataFile.println("Temperatura: " + String(bmp.readTemperature())); // Registra a temperatura no arquivo.
      dataFile.close(); // Fecha o arquivo.
    }
  }

  verificaAltitude(altitude); // Verifica se a altitude atingiu o apogeu e está descendo.

  delay(10); // Aguarda 10 ms antes de repetir o loop.
}