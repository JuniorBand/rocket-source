#include <Wire.h>
#include <Adafruit_BMP280.h> // Inclui a biblioteca do altímetro/barômetro.
#include <Adafruit_MPU6050.h> // Inclui a biblioteca do acelerômetro/giroscópio
#include <SD.h> // Inclui a biblioteca para leitura/escrita em cartão SD.
#include <Functions.h> // Inclui o cabeçalho de funções personalizadas.

/* Definições de pinos/endereços */
const uint8_t PINO_BUZZER = 12;
const uint8_t PINO_MOSFET_1 = 11;
const uint8_t PINO_MOSFET_2 = 10;
const uint8_t PINO_CHIP = 9;
#define BMP280_1_ADDRESS 0x76 // SDO do BMP280 1 está em LOW, então o endereço é 0x76. (Padrão)
#define BMP280_2_ADDRESS 0x77 // SDO do BMP280 2 está em HIGH, então o endereço é 0x77.
#define MPU6050_1_ADDRESS 0x68 // AD0 do MPU6050 1 está em LOW, então o endereço é 0x68. (Padrão)
#define MPU6050_2_ADDRESS 0x69 // AD0 do MPU6050 2 está em HIGH, então o endereço é 0x69.
#define META_APOGEU 100.0f // Meta de apogeu desejada em metros (ajustável).


/* Limites e configurações de voo */
const float LIMITE_SALTO_ANOMALO = 75.0f; // Limite anormal de salto entre leituras seguidas.
const float ALTITUDE_DE_LANCAMENTO = 30.0f; // Altitude para considerar o foguete lançado.
const float QUEDA = 3.0f; // Queda de segurança para o acionamento do para-quedas.
Adafruit_BMP280 bmp1(BMP280_1_ADDRESS); // Cria instância do sensor BMP280 no endereço respectivo (no próprio construtor).
Adafruit_BMP280 bmp2(BMP280_2_ADDRESS);
Adafruit_MPU6050 mpu1; // Cria instância, mas não aceita o endereço no construtor (só no .begin()).
Adafruit_MPU6050 mpu2;
File dataFile; // Cria um objeto para o arquivo de dados no cartão SD.

// Variáveis globais para controle do estado da missão
float apogeu = 0.0f;
bool paraquedasAcionado = false;
bool mosfetsAcionados = false;
float leitura_inicial = 0.0f;
const uint8_t JANELA_LEITURAS = 5;
float buffer_leituras[JANELA_LEITURAS];
uint8_t indice_buffer = 0; // já serve como contador de leituras
uint8_t contador_leituras_zero = 0;


/* Protótipos das funções */
void verificarAltitude(float altitude); // Verifica se a altitude atingiu o apogeu.
void finalizarMissao(const __FlashStringHelper *razao); // Finaliza a missão com uma razão específica.
void acionarBuzzer(int timeOn, int timeOff);  // Buzzer contínuo de timeOn em timeOff ms.
float calcularMedia(float buffer_leituras[]); // Calcula a média das leituras de altitude.

void setup() {
  Serial.begin(115200);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_MOSFET_1, OUTPUT);
  digitalWrite(PINO_MOSFET_1, LOW);
  pinMode(PINO_MOSFET_2, OUTPUT);
  digitalWrite(PINO_MOSFET_2, LOW);
  
  // Inicialização do Bluetooth Low Energy (BLE)
  // O nome do dispositivo ("HABEMUS_S3_ROCKET") será visível para outros dispositivos BLE
  inicializarBLE("HABEMUS_S3_ROCKET"); // Inicializa o BLE e configura o servidor e as características.

  // Inicialização do SD
  if (!SD.begin(PINO_CHIP)) {
    Serial.println("Falha na inicialização do cartão SD!");
    finalizarMissao(F("ERRO CRITICO NA INICIALIZACAO DO CARTAO SD")); 
  } else {
    Serial.println("Cartão SD inicializado com sucesso!");
    dataFile = SD.open("dados.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Registro de Dados de Voo - HABEMUS");
    }
  }

  acionarBuzzer(250, 250); // Buzzer para indicar a inicialização.
  Wire.begin();

  verificarSensores(BMP280_1_ADDRESS, BMP280_2_ADDRESS, MPU6050_1_ADDRESS, MPU6050_2_ADDRESS);

  // Definindo modos de leituras do BMP280
  bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);
  bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X4,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);

  //Determina range de aceleração e ângulos do MPU
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu2.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu1.setFilterBandwidth(MPU6050_BAND_260_HZ); // Define a largura de banda do filtro do MPU6050 para 260Hz.
  mpu2.setFilterBandwidth(MPU6050_BAND_260_HZ);
  mpu1.setCycleRate(MPU6050_CYCLE_40_HZ); // Define a taxa de amostragem do MPU6050 para 40Hz.
  mpu2.setCycleRate(MPU6050_CYCLE_40_HZ);

  Serial.println(F("Iniciando fase de calibracao de altitude..."));

  acionarBuzzer(250, 250); // Buzzer para indicar início da calibração.
}

void loop() {
  /* Variáveis para a calibração inicial e média móvel */
  static bool calibracao_concluida = false;
  
  /* Variáveis para detectar saltos de altitude e estado de voo */
  static float ultima_altitude_relativa = 0.0f;
  static bool primeira_medicao_pos_calibracao = true;
  static bool foguete_lancado = false; // Flag para controlar o estado da missão.

  // Lógica de reconexão BLE
  // Se o dispositivo estava conectado e agora não está, reinicia o advertising para permitir uma nova conexão.
  if (!deviceConnected && oldDeviceConnected) {
      delay(100); // Pequeno atraso para dar tempo ao stack Bluetooth se reorganizar.
      pServer->startAdvertising(); // Reinicia o advertising (anúncio) para que o dispositivo possa ser redescoberto.
      Serial.println("Reiniciando advertising BLE");
      oldDeviceConnected = deviceConnected;
  }

  // Lógica de conexão BLE.
  // Se o dispositivo está conectado e não estava antes, atualiza o status.
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  // Calcula o tempo decorrido em segundos.
  uint16_t tempoDecorrido = millis() / 1000; 

  // Lendo os valores de altura, aceleração e velocidade angular.
  float altitude_atual_1 = bmp1.readAltitude(1013.25);
  float altitude_atual_2 = bmp2.readAltitude(1013.25);

  sensors_event_t a1, g1, t1; // t1 será ignorado.
  sensors_event_t a2, g2, t2; // t2 será ignorado.
  mpu1.getEvent(&a1, &g1, &t1);
  mpu2.getEvent(&a2, &g2, &t2);

  /* VERIFICAÇÕES DE VALIDADE DA LEITURA */
  // Validação só para a altitude, pois ela é primordial para o acionamento do para-quedas.
  // Retorna se há falha de leitura dos sensores, se ambos os sensores estão medindo (0.0 e 0.0) ou (0.0 e NaN).
  // Se os sensores estão medindo (NaN e NaN), finaliza a missão automaticamente.
  SensorStatus sensor_valido = validarSensores(altitude_atual_1, altitude_atual_2);

  if (sensor_valido != SENSOR_OK) {
    switch (sensor_valido) {
      case SENSOR_FALHA_AMBOS_NAN:
        finalizarMissao(F("ERRO DE LEITURA DO SENSOR BMP 1 e 2 (NaN)")); // Loop infinito.
        break;
      case SENSOR_FALHA_CONSECUTIVA_ZERADA:
        finalizarMissao(F("TRES LEITURAS ZERADAS CONSECUTIVAS")); // Loop infinito.
        break;
      default: // SENSOR_FALHA_ZERADO_AMBOS_OU_UM_NAN: Sensores (0.0f) e (NaN) ou (0.0) e (0.0)
        break;
    }
    return; // Se os sensores não estão medindo corretamente, não continua a execução do loop.
  }

  // Se os sensores estão medindo corretamente, atualiza o buffer de leituras e procede com a lógica de voo.

  if (!calibracao_concluida) {
    // --- Lógica de Calibração ---
    if (indice_buffer == JANELA_LEITURAS) {
      leitura_inicial = calcularMedia(buffer_leituras); // Calcula a média das leituras do buffer.
      calibracao_concluida = true;

      Serial.println(F("================================================="));
      Serial.print(F("Calibracao concluida. Altitude de referencia (zero): "));
      Serial.print(leitura_inicial);
      Serial.println(F(" m."));
      Serial.println(F("AGUARDANDO LANCAMENTO..."));
      Serial.println(F("================================================="));
      
    }

    delay(10); 
    return;
  }
  
  /* --- Fase de Operação Normal (Após a Calibração Inicial) --- */  
  float altitude_relativa = calcularMedia(buffer_leituras);
  printDados(tempoDecorrido, altitude_relativa, a1, a2, g1, g2);

  /* --- Lógica de voo dividida por estado (Aguardando/Em Voo) --- */

  // Se o foguete ainda não foi lançado, apenas verifica se ultrapassou a altitude de lançamento.
  if (!foguete_lancado) {
    // !!! [PERIGOSO]: se o foguete cair sem para-quedas antes de atingir a altitude de lançamento !!!
    if (altitude_relativa > ALTITUDE_DE_LANCAMENTO) {
      foguete_lancado = true;
      Serial.println(F(">>> LANCAMENTO DETECTADO! ARMANDO SISTEMA DE APOGEU. <<<"));
    }
    return; // Se ainda não foi lançado, não executa a lógica de voo normal.
  } 
  // Se o foguete já foi lançado, executa a lógica de voo normal.

  // Filtro de Salto Anômalo (Compara a última medição de altura com a atual)
  if (primeira_medicao_pos_calibracao) {
    ultima_altitude_relativa = altitude_relativa;
    primeira_medicao_pos_calibracao = false;
  } else {
    float delta_leitura_bruta = abs(altitude_relativa - ultima_altitude_relativa);
    
    if (delta_leitura_bruta >= LIMITE_SALTO_ANOMALO) {
      finalizarMissao(F("SALTO DE LEITURA ANOMALO"));
    }

    ultima_altitude_relativa = altitude_relativa;
  }

  // Atualiza o apogeu
  if (altitude_relativa > apogeu) {
    apogeu = altitude_relativa;
  }

  // Verifica a condição de ejeção
  verificarAltitude(altitude_relativa);

  delay(10);
}

void finalizarMissao(const __FlashStringHelper *razao) { // Finaliza missão em caso de erro ou sucesso.
  Serial.println(F("\n================================================="));
  Serial.print(F("FINALIZANDO MISSAO. MOTIVO: "));
  Serial.println(razao);
  Serial.println(F("================================================="));

  digitalWrite(PINO_MOSFET_1, HIGH);
  digitalWrite(PINO_MOSFET_2, HIGH);
  delay(3000);
  digitalWrite(PINO_MOSFET_1, LOW);
  digitalWrite(PINO_MOSFET_2, LOW);

  paraquedasAcionado = true;

  if (dataFile) {
    dataFile.println("Apogeu Final: " + String(apogeu) + " m");
    dataFile.close();
    Serial.println(F("Arquivo SD fechado."));
  }

  while (true) {
    acionarBuzzer(300, 300);
  }
}

// Calcula a média de 5 valores válidos de altura.
float calcularMedia(float buffer_leituras[]) { 
  
  float soma_buffer = 0.0f;

  for (int i = 0; i < JANELA_LEITURAS; i++) {
    soma_buffer += buffer_leituras[i];
  }

  float nova_media = soma_buffer /  JANELA_LEITURAS; // Usa número real de leituras 
  float altitude_relativa = nova_media - leitura_inicial;
  return altitude_relativa; // Retorna a altitude relativa ajustada.
}



void verificarAltitude(float altitude) {
  // Condição ajustada para o foguete de alto desempenho.
  if (!mosfetsAcionados){
    if ((apogeu >= META_APOGEU && altitude <= apogeu - 6*QUEDA) 
      || (apogeu < META_APOGEU && altitude <= apogeu - QUEDA)) {
      // Aciona os mosfets se superar meta de apogeu e estiver descendo ou
      // se não atingir a meta e estiver descendo com uma margem de erro de QUEDA.
      mosfetsAcionados = true;

      finalizarMissao(F("ACIONAMENTO NORMAL POS-APOGEU"));
    }
  }
}


void acionarBuzzer(int timeOn, int timeOff) { // Aciona o buzzer por timeOn milissegundos e desliga por timeOff milissegundos.
  digitalWrite(PINO_BUZZER, HIGH); 
  delay(timeOn);
  digitalWrite(PINO_BUZZER, LOW);  
  delay(timeOff); 
} 