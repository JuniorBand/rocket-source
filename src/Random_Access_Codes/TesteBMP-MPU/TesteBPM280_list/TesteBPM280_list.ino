//Incluindo Bibliotecas
#include <Wire.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; //I2C


const uint8_t relePin = 12; //coloque o pino do relé
const float THRESHOLD = 5; //limite da diferença das médias de alturas
const int MSEC = 1000; //milissegundos de delay
const uint8_t  WINDOW = 5; //quantidade de valores para média
int counter = 1; //conta qual média está sendo lida
float medium[WINDOW]; //array estático
float m1 = 0.0; //primeira média (definida uma vez)
float m2 = 0.0; //p/ as médias posteriores à primeira (variável)
float diff = 0.0; //diferença absoluta entre as altitudes
  

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Inicia a conexão
  pinMode(relePin, OUTPUT); //Relé como output
  digitalWrite(relePin, LOW);

  Serial.println(F("BMP280 teste"));// Imprimindo Mensagem de teste no Monitor Serial
  
  if (!bmp.begin(0x76)) { /*Definindo o endereço I2C como 0x76. Mudar, se necessário, para (0x77)*/
    
    //Imprime mensagem de erro no caso de endereço inválido ou não localizado. Modifique o valor 
    Serial.println(F("Não foi possível encontrar um sensor BMP280 válido, verifique a fiação ou tente outro endereço!"));
    while (1) delay(10);
  }

  //medir os 5 primeiros valores para definir a primeira média m1
  m1 = calcularMedia(medium, WINDOW);

  //Primeira média (fixa)
  printMedium(m1, counter);
  counter++;
  diff = 0.0; //garantir diff = 0

}

void loop() {
  // put your main code here, to run repeatedly:

  // Obtém nova média a cada "WINDOW" valores completos lidos
  m2 = calcularMedia(medium, WINDOW);
  printMedium(m2, counter); //printa a nova média.
  diff = diffMed(m1, m2); //diff armazena a diferença entre as médias
  counter++;


  //Aciona o relé e para o loop
  if (diff > THRESHOLD){ //diferença de médias maior que 5, se THRESHOLD = 5
    Serial.println("Altura 0 registrada, acionando relé: ");
    delay(MSEC);
    digitalWrite(relePin, HIGH);
    while (true) {
      delay(MSEC)// pode deixar vazio ou colocar algum código que precisa rodar parado
    }
  }

  //else: continua o loop
  Serial.println("Insuficiente!");

  //delay(MSEC); usar delay(MSEC) se precisar

}

void printMedium(float m, int counter){ //Printa o valor atual da média
  if (counter == 1){
    Serial.print("Primeira média: ");
  } else {
    Serial.print("Média (");
    Serial.print(counter); 
    Serial.print("): ");
  }
  Serial.print(m);
  Serial.println(" m.");

}

float diffMed(float m1, float m2){
  //Tirar diferença das médias
  float diff = abs(m2 - m1);
  Serial.print("Diferença atual de médias: ");
  Serial.print(diff);
  Serial.println(" m.");
  return diff;
}

//Calcular média
float calcularMedia(float arr[], uint8_t tamanho) {
  float soma = 0.0;
  for (uint8_t i = 0; i < tamanho; i++) {
    arr[i] = bmp.readAltitude(1013.25); //lê o valor recebido pelo sensor
    soma += arr[i];
    //delay(); usar delay() se precisar
  }
  return soma / tamanho;
}
