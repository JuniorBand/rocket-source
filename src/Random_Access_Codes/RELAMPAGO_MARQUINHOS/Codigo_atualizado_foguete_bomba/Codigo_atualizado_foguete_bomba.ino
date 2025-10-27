/*============================================================================================================================
  HISTÓRICO DE MODIFICAÇÕES DO CÓDIGO DO FOGUETE RELÂMPAGO MARQUINHOS:
    1.  Configuração do Sensor BMP280:
      - Implementação da comunicação I2C utilizando os pinos A4 (SDA) e A5 (SCL).
      - O sensor BMP280 agora é inicializado diretamente no endereço I2C 0x76 (com base em depurações anteriores).
    2.  Melhorias na Inicialização e Feedback:
      - O sistema agora toca 1 bip ao iniciar o programa.
      - Após a tentativa de inicialização do BMP280, o sistema toca 2 bips para indicar sucesso e início da calibração.
      - Se o BMP280 não for encontrado durante a inicialização, um aviso é impresso no Monitor Serial e a função
      `escanearI2C()` é executada para ajudar na depuração. O programa, no entanto, continua sua execução normal
      (não trava, nem aciona o relê/buzzer por falta do sensor nesse estágio), embora as leituras de altitude
      sejam inválidas.
    3.  Lógica de Calibração e Medição de Altitude:
      - A altitude inicial é calibrada fazendo uma média das primeiras 15 leituras do BMP280. Este valor
      (`leitura_inicial_fixa`) se torna o ponto de referência "zero" para todas as altitudes subsequentes.
      - Após a calibração inicial, a altitude exibida é a **média móvel das últimas 15 leituras** do sensor,
      subtraída da `leitura_inicial_fixa`. Isso garante que:
      - A altitude comece em 0 (ou muito próximo) no ponto de lançamento.
      - A altitude se atualize continuamente e de forma mais suave, refletindo o movimento real do foguete.
      - Adicionadas validações de leitura (`isnan()` e `== 0`) para evitar usar dados inválidos do sensor,
      especialmente durante a fase de calibração.
    4.  Saída do Terminal (Monitor Serial):
      - A saída foi simplificada para mostrar apenas a altitude de referência (zero) uma vez, e depois,
      continuamente, as altitudes atualizadas (média móvel relativa ao zero). Mensagens de depuração internas
      foram removidas para clareza da operação.
    5.  Funções de Erro/Depuração:
      - A antiga função `verificabmp_erro()` (que travava o programa em caso de erro do BMP no setup) foi removida.
      - A função `escanearI2C()` foi mantida exclusivamente como uma ferramenta de depuração (chamada apenas uma vez no setup
      se o BMP não for encontrado, sem travar o programa).
    6.  OTIMIZAÇÃO DE MEMÓRIA (RAM):
      - Todas as strings literais impressas no Serial Monitor foram movidas para a memória Flash (PROGMEM) usando a macro F().
      Isso reduz significativamente o uso da memória RAM dinâmica.
=============================================================================================================================*/

#include <Wire.h>             /* Inclui a biblioteca para comunicação I2C. */
#include <Adafruit_BMP280.h>  /* Inclui a biblioteca para o sensor de pressão BMP280. */

/* Definições de pinos */
#define pinoBuzzer 11         /* Define o pino 4 como o pino do buzzer. */
#define pinoRele 9            /* Define o pino 9 como o pino do módulo relê. */
#define LED_EMBUTIDO 13       /* Define o pino 13 como o LED embutido do Arduino. */

Adafruit_BMP280 bmp; /* Cria um objeto para o sensor BMP280. */

float apogeu = 0;             /* Variável para armazenar o valor máximo de altitude alcançada. */
bool releAcionado = false;    /* Variável de controle para saber se o relê já foi acionado. */

/* Variáveis para a calibração inicial e média móvel */
float leitura_inicial_fixa = 0;        /* A média das primeiras 15 leituras, que será o ponto zero de referência. */
bool calibracao_inicial_concluida = false; /* Flag que indica se a calibração inicial já terminou. */

/* Parâmetros e buffer para a média móvel das leituras de altitude */
const int quantidade_leituras_media = 15; /* Número de leituras para calcular a média. */
float buffer_leituras[quantidade_leituras_media]; /* Array para armazenar as últimas 'quantidade_leituras_media' leituras. */
int indice_buffer = 0;                     /* Índice atual no buffer (para controle circular). */
int contagem_leituras_pre_calibracao = 0;  /* Contador para as leituras durante a fase de calibração inicial. */

/* Protótipos das funções */
void verificaAltitude(float altitude);
void escanearI2C();

void setup() {
  Serial.begin(9600); /* Inicializa a comunicação serial a 9600 bps. */
  pinMode(pinoBuzzer, OUTPUT);  /* Define o pino do buzzer como saída. */
  pinMode(pinoRele, OUTPUT);    /* Define o pino do relê como saída. */
  pinMode(LED_EMBUTIDO, OUTPUT); /* Define o pino do LED embutido como saída. */
  digitalWrite(pinoRele, LOW);  /* Garante que o relê inicia desligado. */

  /* 1. Toca 1 bip para indicar o início do programa */
  digitalWrite(pinoBuzzer, HIGH);
  delay(250);
  digitalWrite(pinoBuzzer, LOW);
  delay(250);

  /* 2. Inicializa a comunicação I2C (pinos A4 e A5 no Arduino Uno/Nano). */
  Wire.begin();

  /* 3. Tenta inicializar o BMP280 no endereço 0x76. */
  Serial.println(F("Tentando inicializar BMP280 no endereco 0x76..."));
  if (!bmp.begin(0x76)) { /* Se o sensor BMP280 não for encontrado: */
    Serial.println(F("================================================="));
    Serial.println(F("AVISO: Nao foi possivel encontrar um sensor BMP280 valido no 0x76!"));
    Serial.println(F("Verifique a fiacao (SDA->A4, SCL->A5, VCC->5V/3.3V, GND->GND) e o modulo."));
    Serial.println(F("Executando Escaner I2C para ajudar a depurar..."));
    Serial.println(F("================================================="));
    escanearI2C(); /* Executa o escaner I2C para depuração. O programa continua. */
  } else {
    Serial.println(F("BMP280 encontrado com sucesso no endereco 0x76!"));
  }
  Serial.println(F("Iniciando fase de calibracao de altitude (aguarde 15 leituras para definir o ponto zero)..."));

  /* 4. Toca 2 bips para indicar inicialização do sensor e início da calibração. */
  digitalWrite(pinoBuzzer, HIGH);
  delay(250);
  digitalWrite(pinoBuzzer, LOW);
  delay(250);
  digitalWrite(pinoBuzzer, HIGH);
  delay(250);
  digitalWrite(pinoBuzzer, LOW);
}

void loop() {
  float altitude_absoluta_atual = bmp.readAltitude(1013.25); /* Lê a altitude absoluta do sensor BMP280. */

  /* --- Validação da leitura do sensor --- */
  if (isnan(altitude_absoluta_atual) || altitude_absoluta_atual == 0) {
    if (calibracao_inicial_concluida) {
      Serial.println(F("AVISO: Leitura de altitude invalida. Verifique o sensor."));
    } else {
      Serial.println(F("Aguardando leitura valida para calibracao..."));
    }
    delay(100); /* Pequeno atraso para não sobrecarregar o sensor/serial se a leitura for inválida. */
    return; /* Pula esta iteração do loop se a leitura for inválida. */
  }

  /* --- Fase de Calibração Inicial e Média Móvel ---
      Adiciona a nova leitura ao buffer circular */
  buffer_leituras[indice_buffer] = altitude_absoluta_atual;
  indice_buffer = (indice_buffer + 1) % quantidade_leituras_media; /* Avança o índice circularmente. */

  /* Atualiza a contagem de leituras iniciais (usada apenas até o buffer encher pela primeira vez) */
  if (contagem_leituras_pre_calibracao < quantidade_leituras_media) {
    contagem_leituras_pre_calibracao++;
  }

  /* Calcula a soma das leituras presentes no buffer */
  float soma_buffer = 0;
  for (int i = 0; i < contagem_leituras_pre_calibracao; i++) {
    soma_buffer += buffer_leituras[i];
  }

  /* Calcula a média atual das leituras no buffer (média móvel) */
  float nova_media_continua;
  if (contagem_leituras_pre_calibracao > 0) {
     nova_media_continua = soma_buffer / contagem_leituras_pre_calibracao;
  } else {
     nova_media_continua = 0; /* Evita divisão por zero se a contagem for 0. */
  }

  /* Define a 'leitura_inicial_fixa' APENAS UMA VEZ, quando o buffer estiver cheio pela primeira vez. */
  if (!calibracao_inicial_concluida && contagem_leituras_pre_calibracao == quantidade_leituras_media) {
    leitura_inicial_fixa = nova_media_continua; /* A primeira média se torna a referência fixa para o ponto zero. */
    calibracao_inicial_concluida = true;       /* Marca a calibração como concluída. */
    Serial.print(F("Calibracao inicial concluida. Altitude de referencia (zero): "));
    Serial.print(leitura_inicial_fixa);
    Serial.println(F(" m."));
    Serial.println(F("Programa principal iniciado. Altitudes serao relativas ao ponto de inicio (0m)."));
  }

  /* --- Fase de Operação Normal (Após a Calibração Inicial) --- */
  if (!calibracao_inicial_concluida) {
    delay(50); /* Aguarda a calibração inicial ser concluída. */
    return; /* Pula o restante do loop até a calibração ser concluída. */
  }

  /* Calcula a altitude relativa usando a média móvel contínua e a leitura_inicial_fixa. */
  float altitude_relativa_media_movel = nova_media_continua - leitura_inicial_fixa;

  /* Imprime apenas a altitude relativa para o usuário. */
  Serial.print(F("Altitude = "));
  Serial.print(altitude_relativa_media_movel);
  Serial.println(F(" m"));

  /* Atualiza o apogeu se a altitude atual for maior. */
  if (altitude_relativa_media_movel > apogeu) {
    apogeu = altitude_relativa_media_movel;
  }

  /* Verifica as condições para acionar o relê (paraquedas). */
  verificaAltitude(altitude_relativa_media_movel);

  delay(10); /* Pequeno atraso entre as iterações do loop. */
}

/* Implementação da função verificaAltitude: verifica as condições para acionar o relê. */
void verificaAltitude(float altitude) {
  /* Condições para acionamento do relê (paraquedas):
    1. Se o apogeu ultrapassou 200m E a altitude atual desceu para 200m ou menos.
                                       OU
    2. Se o apogeu foi menor que 200m E a altitude atual desceu 3m abaixo do apogeu.
  */
  if ((apogeu > 200 && altitude <= 200) || (apogeu < 200 && altitude <= apogeu - 10)) {
    if (!releAcionado) { /* Garante que o relê seja acionado apenas uma vez. */
      digitalWrite(pinoRele, HIGH); /* Aciona o relê. */
      delay(5000);                /* Mantém o relê acionado por 5 segundos. */
      digitalWrite(pinoRele, LOW);  /* Desliga o relê. */
      releAcionado = true;        /* Marca que o relê já foi acionado. */

      /* Entra em um loop infinito para acionar o buzzer continuamente após o acionamento do paraquedas.
         Isso sinaliza que o sistema de recuperação foi ativado e o programa não avançará mais.*/
      while (true) {
        digitalWrite(pinoBuzzer, HIGH);
        delay(1000);
        digitalWrite(pinoBuzzer, LOW);
        delay(1000);
      }
    }
  }
}

/* Implementação da função escanearI2C: varre endereços I2C para depuração. */
void escanearI2C() {
  byte erro, endereco;
  int numDispositivos;

  Serial.println(F("Iniciando Escaner I2C..."));

  numDispositivos = 0;
  for (endereco = 1; endereco < 127; endereco++) {
    /* Começa a transmissão para o endereço I2C atual. */
    Wire.beginTransmission(endereco);
    /* Envia a transmissão e verifica o código de erro.
       Retorna 0 para sucesso (dispositivo encontrado).*/
    erro = Wire.endTransmission();

    if (erro == 0) {
      Serial.print(F("Dispositivo I2C encontrado no endereco 0x"));
      if (endereco < 16) { /* Adiciona '0' se o endereço for de um dígito hexadecimal. */
        Serial.print(F("0"));
      }
      Serial.print(endereco, HEX); /* Imprime o endereço em formato hexadecimal. */
      Serial.println(F("  !"));
      numDispositivos++;
    } else if (erro == 4) { /* Erro desconhecido durante a varredura. */
      Serial.print(F("Erro desconhecido no endereco 0x"));
      if (endereco < 16) {
        Serial.print(F("0"));
      }
      Serial.println(endereco, HEX); /* Imprime o endereço em formato hexadecimal. */
    }
  }
  if (numDispositivos == 0) {
    Serial.println(F("Nenhum dispositivo I2C encontrado."));
  } else {
    Serial.println(F("Varredura I2C concluida."));
  }
  Serial.println(F("================================================="));
  delay(5000); /* Pausa para o usuário ler o resultado no Monitor Serial. */
}