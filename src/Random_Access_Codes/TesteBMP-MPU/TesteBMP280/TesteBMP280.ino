//Incluindo Bibliotecas
#include <Wire.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; //I2C

#define relePin 12 //O pino do relé
#define MSEC 1000; //Milissegundos de delay
#define WINDOW 5 //Quantidade de valores para média


const float THRESHOLD = 5.0; //Limite da diferença das médias de alturas
uint8_t counter = 0; //Conta o numero de iterações do loop (0 - 5) p/ localizar qual média está sendo lida
float soma_altitudes = 0.0; //Soma das altitudes
bool first_read = true; //Booleano p/ registrar primeira média
float prim_media = 0.0; //Armazena a primeira média



void setup() {

  Serial.begin(9600); //Inicia a conexão
  pinMode(relePin, OUTPUT); //Relé como output
  digitalWrite(relePin, HIGH); //Confirmar desligamento do relé (supondo relé desligado em HIGH)

  Serial.println(F("BMP280 teste"));// Imprimindo Mensagem de teste no Monitor Serial (opcional)
  
 if (!bmp.begin(0x76)) { /*Definindo o endereço I2C como 0x76. Mudar, se necessário, para (0x77)*/
    
    //Imprime mensagem de erro no caso de endereço inválido ou nãoo localizado. Modifique o valor 
    Serial.println(F("Não foi possível encontrar um sensor BMP280 válido, verifique a fiação ou tente outro endereço!"));
    delay(2000); // Aguarda 2 segundos.

    if (!bmp.begin(0x77)) { // Tenta inicializar novamente o sensor.
    while (1) {delay(10);}
    }

  }

}


//Comparar médias
void comparar_media(float soma_altitudes){
  float media_atual = soma_altitudes/WINDOW; //Média de altitude atual
  float diff = media_atual - prim_media; //Altitude do foguete com prim_media como ponto 0: altura

  Serial.print("Média Atual: "); Serial.println(media_atual);
  Serial.print("Altura (média atual - inicial) = "); Serial.println(diff);

  if(diff > THRESHOLD) { //Passou do limite 
    Serial.println("Diferença passou do limite! O foguete subiu!");
    Serial.println("Acionando o relé..."); //opcional
    digitalWrite(relePin, LOW);//Liga o relé (supondo que ele ligue em LOW) 
    while (true) { //para o loop
      delay(MSEC);
    }
  }

  //else:
  Serial.println("Insuficiente!");

}



void loop(){

  float altitude = bmp.readAltitude(1013.25); //Altitude atual

  soma_altitudes += altitude; //Incrementa cada altitude na soma_altitudes
  counter++;//Cada contagem até 5 é registrada

  if (counter < WINDOW){ //counter < 5
    return; //termina a atual iteração
  } else if (first_read) {
    prim_media = (soma_altitudes/WINDOW); //Fixa a primeira média
    first_read = false;
    counter = 0; //Volta a contagem
    soma_altitudes = 0; //Esvazia a soma_altitudes
    return; //termina a atual iteração
  }

  //else: não é a primeira média e temos o valor da próxima média:
  comparar_media(soma_altitudes); //Compara com a primeira e para o loop se altura > limite
  
  counter = 0; //Volta a contagem
  soma_altitudes = 0; //Esvazia a soma_altitudes 
  
  delay(10);//adicionar delay se necessário
}

