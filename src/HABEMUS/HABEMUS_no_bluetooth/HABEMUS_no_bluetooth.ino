#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <SD.h> // Inclui a biblioteca para leitura/escrita em cartão SD.


/* Definições de pinos/endereços */
// *[ALTERAÇÃO/SUBSTITUIÇÃO]* Os const uint8_t foram substituidos por uint8_t (0-255) por motivos de segurança, mesmo ocupando um pequeno espaço.
const uint8_t PINO_BUZZER = 12;
const uint8_t PINO_MOSFET_1 = 11;
const uint8_t PINO_MOSFET_2 = 10;
const uint8_t PINO_CHIP = 9;
#define BMP280_1_ADDRESS 0x76
#define BMP280_2_ADDRESS 0x77
#define MPU6050_1_ADDRESS 0x68
#define MPU6050_2_ADDRESS 0x69
#define META_APOGEU 100.0f // Meta de apogeu desejada em metros (ajustável).


/* Limites e configurações de voo */
const float LIMITE_SALTO_ANOMALO = 75.0f;
const float ALTITUDE_DE_LANCAMENTO = 30.0f; // Altitude para considerar o foguete lançado.
const float QUEDA = 3.0f; 
Adafruit_BMP280 bmp1(BMP280_1_ADDRESS);
Adafruit_BMP280 bmp2(BMP280_2_ADDRESS);
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
File dataFile; // Cria um objeto para o arquivo de dados no cartão SD.

enum SensorStatus {
  SENSOR_OK,
  SENSOR_FALHA_AMBOS_NAN,
  SENSOR_FALHA_ZERADO_AMBOS_OU_UM_NAN,
  SENSOR_FALHA_CONSECUTIVA_ZERADA,
  SENSOR_FALHA_UM_SENSOR
};

float apogeu = 0.0f;
bool mosfetsAcionados = false;
bool paraquedasAcionado = false;

/* Variáveis para a calibração inicial e média móvel */
float leitura_inicial = 0.0f;
bool calibracao_concluida = false;

/* Parâmetros e buffer para a média móvel */
const uint8_t JANELA_LEITURAS = 5;
float buffer_leituras[JANELA_LEITURAS];
uint8_t indice_buffer = 0; // já serve como contador de leituras

/* Variáveis para detectar saltos de altitude e estado de voo */
float ultima_altitude_relativa = 0.0f;
bool primeira_medicao_pos_calibracao = true;
uint8_t contador_leituras_zero = 0;
bool foguete_lancado = false; // Flag para controlar o estado da missão.

/* Protótipos das funções */
void verificarAltitude(float altitude); // Verifica se a altitude atingiu o apogeu.
void escanearI2C(void); // Escaneia o barramento I2C para detectar dispositivos.
void finalizarMissao(const __FlashStringHelper *razao); // Finaliza a missão com uma razão específica.
void verificarSensores(uint8_t ADDRESS_1, uint8_t ADDRESS_2, uint8_t ADDRESS_3, uint8_t  ADDRESS_4); // Verifica se os sensores estão funcionando corretamente nos seus endereços.
SensorStatus validarSensores(float altitude_atual_1, float altitude_atual_2); // Valida as leituras dos sensores.
void printDados(float altitude, sensors_event_t a1, sensors_event_t a2, sensors_event_t g1, sensors_event_t g2); // Imprime os dados no Serial e no cartão SD. 
void acionarBuzzer(int timeOn, int timeOff);  // Buzzer contínuo de timeOn em timeOff ms.
float calcularMedia(float buffer_leituras[]); // Calcula a média das leituras de altitude.
void atualizarBuffer(float leitura); // Atualiza o buffer de leituras com a nova leitura de altitude.

void setup() {
  Serial.begin(115200);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_MOSFET_1, OUTPUT);
  digitalWrite(PINO_MOSFET_1, LOW);
  pinMode(PINO_MOSFET_2, OUTPUT);
  digitalWrite(PINO_MOSFET_2, LOW);
  Serial.println("Inicializando. Conecte-se ao dispositivo via Serial.");

  // Inicialização do SD
  if (!SD.begin(PINO_CHIP)) {
    Serial.println("Falha na inicialização do cartão SD!");
    finalizarMissao(F("ERRO CRITICO NA INICIALIZACAO DO CARTAO SD")); // Adicionado: Finaliza a missão se o SD falhar
  } else {
    Serial.println("Cartão SD inicializado com sucesso!");
  }

  acionarBuzzer(250, 250); // Buzzer para indicar a inicialização.
  Wire.begin();

  verificarSensores(BMP280_1_ADDRESS, BMP280_2_ADDRESS, MPU6050_1_ADDRESS, MPU6050_2_ADDRESS);

  //determinar isso
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
  //Lendo os valores de altura, aceleração e angulação
  float altitude_atual_1 = bmp1.readAltitude(1013.25);
  float altitude_atual_2 = bmp2.readAltitude(1013.25);

  sensors_event_t a1, g1, t1; // t1 será ignorado
  sensors_event_t a2, g2, t2; // t2 será ignorado
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
        finalizarMissao(F("ERRO DE LEITURA DO SENSOR BMP 1 e 2 (NaN)"));
        break;
      case SENSOR_FALHA_CONSECUTIVA_ZERADA:
        finalizarMissao(F("TRES LEITURAS ZERADAS CONSECUTIVAS"));
        break;
      default:
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
      
      dataFile = SD.open("dados.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println("Registro de Dados de Voo - HABEMUS");
      }
    }

    delay(100); 
    return;
  }
  
  /* --- Fase de Operação Normal (Após a Calibração Inicial) --- */  
  float altitude_relativa = calcularMedia(buffer_leituras);
  printDados(altitude_relativa, a1, a2, g1, g2);

  /* --- Lógica de voo dividida por estado (Aguardando/Em Voo) --- */

  // Se o foguete ainda não foi lançado, apenas verifica se ultrapassou a altitude de lançamento.
  if (!foguete_lancado) {
    // !!Perigoso se o foguete cair antes de atingir a altitude de lançamento!!
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

  delay(100);
}

void finalizarMissao(const __FlashStringHelper *razao) {
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

float calcularMedia(float buffer_leituras[]) {
  
  float soma_buffer = 0.0f;

  for (int i = 0; i < JANELA_LEITURAS; i++) {
    soma_buffer += buffer_leituras[i];
  }

  return soma_buffer / JANELA_LEITURAS;
}

void atualizarBuffer(float leitura) {
  if (leitura != 0.0f && !isnan(leitura)) { // Prevenir leituras inválidas.
    for (int i = 0; i < JANELA_LEITURAS - 1; i++) {
      buffer_leituras[i] = buffer_leituras[i + 1]; // Move os valores para a esquerda.
    }
    buffer_leituras[JANELA_LEITURAS - 1] = leitura; // Adiciona a nova leitura no final.
    if (indice_buffer < JANELA_LEITURAS) { // Incrementa o contador até o buffer estar cheio.
      indice_buffer++;
    }
  }
}

void verificarAltitude(float altitude) {
  // Condição ajustada para o foguete de alto desempenho.
  if (foguete_lancado && !paraquedasAcionado && (apogeu - altitude >= QUEDA)) {
    Serial.println(F("CONDICAO DE EJECÃO ATINGIDA: QUEDA DETECTADA APOS APOGEU."));
    finalizarMissao(F("APOGEU DETECTADO COM QUEDA"));
  }
}

void escanearI2C() {
  byte erro, endereco;
  int numDispositivos;

  Serial.println(F("Escaneando barramento I2C..."));
  numDispositivos = 0;
  for (endereco = 1; endereco < 127; endereco++) {
    Wire.beginTransmission(endereco);
    erro = Wire.endTransmission();

    if (erro == 0) {
      Serial.print(F("Dispositivo I2C encontrado no endereco 0x"));
      if (endereco < 16) {
        Serial.print(F("0"));
      }
      Serial.print(endereco, HEX);
      Serial.println();
      numDispositivos++;
    } else if (erro == 4) {
      Serial.print(F("Erro desconhecido no endereco 0x"));
      if (endereco < 16) {
        Serial.print(F("0"));
      }
      Serial.println(endereco, HEX);
    }
  }
  if (numDispositivos == 0) {
    Serial.println(F("Nenhum dispositivo I2C encontrado."));
    finalizarMissao(F("NENHUM DISPOSITIVO I2C ENCONTRADO"));
  } else {
    Serial.print(numDispositivos);
    Serial.println(F(" dispositivo(s) I2C encontrado(s)."));
  }
  Serial.println(F("Concluido."));
}


void verificarSensores(uint8_t ADDRESS_1, uint8_t ADDRESS_2, uint8_t ADDRESS_3, uint8_t  ADDRESS_4){
  //Tentando inicializar os BMPs
  if (!bmp1.begin(ADDRESS_1)) {
    Serial.println(F("Não foi possível encontrar um sensor BMP280 válido 1, verifique a fiação!"));
    finalizarMissao(F("FALHA SENSOR BMP1"));
  }
  if (!bmp2.begin(ADDRESS_2)) {
    Serial.println(F("Não foi possível encontrar um sensor BMP280 válido 2, verifique a fiação!"));
    finalizarMissao(F("FALHA SENSOR BMP2"));
  }

  //Tentando inicializar os MPUs
  if (!mpu1.begin(ADDRESS_3)) {
    Serial.println(F("Não foi possível encontrar um sensor MPU6050 válido 1, verifique a fiação!"));
    finalizarMissao(F("FALHA SENSOR MPU1"));
  }
  if (!mpu2.begin(ADDRESS_4)) {
    Serial.println(F("Não foi possível encontrar um sensor MPU6050 válido 2, verifique a fiação!"));
    finalizarMissao(F("FALHA SENSOR MPU2"));
  }

  Serial.println(F("Sensores BMP e MPU inicializados e funcionando."));
}

SensorStatus validarSensores(float altitude_atual_1, float altitude_atual_2){
  // Se os sensores retornarem NaN, retorna o status de falha.
  if (isnan(altitude_atual_1) && isnan(altitude_atual_2)) {
    Serial.println(F("ERRO: Leituras de altitude do BMP 1 e 2 invalidas (NaN)!"));
    return SENSOR_FALHA_AMBOS_NAN;
  }

  // Se os sensores retornarem 0.0 ou 0.0 e NaN, incrementa o contador de leituras zero.
  // Se o contador de leituras zero for maior ou igual a 3, retorna o status de falha consecutivo.
  if ((altitude_atual_1 == 0.0f && altitude_atual_2 == 0.0f) ||
      (isnan(altitude_atual_1) && altitude_atual_2 == 0.0f) ||
      (altitude_atual_1 == 0.0f && isnan(altitude_atual_2))) {
    Serial.println(F("ALERTA: Leituras de altitude zeradas detectadas!"));
    contador_leituras_zero++;
    if (contador_leituras_zero >= 3) {
      return SENSOR_FALHA_CONSECUTIVA_ZERADA;
    }
  } else {
    contador_leituras_zero = 0; // Reseta o contador se uma leitura válida for obtida.
  }

  // Se apenas um sensor estiver retornando NaN ou 0.0, ainda é possível continuar, mas com um alerta.
  if (isnan(altitude_atual_1) || isnan(altitude_atual_2) || altitude_atual_1 == 0.0f || altitude_atual_2 == 0.0f) {
    Serial.println(F("ALERTA: Um dos sensores de altitude está retornando 0.0 ou NaN."));
    return SENSOR_FALHA_UM_SENSOR;
  }

  return SENSOR_OK; // Indica que os sensores estão medindo corretamente.
}

void printDados(float altitude_relativa, sensors_event_t a1, sensors_event_t a2, sensors_event_t g1, sensors_event_t g2){
  Serial.print(F("Altitude = "));
  Serial.print(altitude_relativa);
  Serial.print(F(" m 	"));

  //Calculando as médias das acelerações e giroscópios
  float media_ac_x = (a1.acceleration.x + a2.acceleration.x) / 2.0;
  float media_ac_y = (a1.acceleration.y + a2.acceleration.y) / 2.0;
  float media_ac_z = (a1.acceleration.z + a2.acceleration.z) / 2.0;

  float media_gyro_x = (g1.gyro.x + g2.gyro.x) / 2.0;
  float media_gyro_y = (g1.gyro.y + g2.gyro.y) / 2.0;
  float media_gyro_z = (g1.gyro.z + g2.gyro.z) / 2.0;

  Serial.print(F("Acceleration X: "));
  Serial.print(media_ac_x);
  Serial.print(F(" m/s^2 Y: "));
  Serial.print(media_ac_y);
  Serial.print(F(" m/s^2 Z: "));
  Serial.print(media_ac_z);
  Serial.println(F(" m/s^2"));

  Serial.print(F("Rotation X: "));
  Serial.print(media_gyro_x);
  Serial.print(F(" rad/s Y: "));
  Serial.print(media_gyro_y);
  Serial.print(F(" rad/s Z: "));
  Serial.print(media_gyro_z);
  Serial.println(F(" rad/s"));

  //Print no cartão SD
  if (dataFile) { // Se o arquivo foi aberto com sucesso:
    dataFile.print("Altitude: " + String(altitude_relativa) + ", "); // Registra F(a altitude no ar)quivo.
    
    //Print das acelerações na memória Flash 
    dataFile.print("Acceleration X: "+ String(media_ac_x) + " m/s^2");
    dataFile.print("; Y: " + String(media_ac_y) + " m/s^2");
    dataFile.print("; Z: " + String(media_ac_z) + " m/s^2");

    //Print das angulações na memória Flash 
    dataFile.print("Rotation X: " + String(media_gyro_x) + " rad/s");
    dataFile.print("; Y: " + String(media_gyro_y) + " rad/s");
    dataFile.print("; Z: " + String(media_gyro_z) + " rad/s");
    dataFile.println(); // Nova linha para a próxima entrada de dados.
  }
}

void acionarBuzzer(int timeOn, int timeOff) { // *[ALTERAÇÃO]* Buzzer contínuo de timeOn em timeOff ms.
  digitalWrite(PINO_BUZZER, HIGH); // Aciona o buzzer.
  delay(timeOn);
  digitalWrite(PINO_BUZZER, LOW);  // Desliga o buzzer.
  delay(timeOff); 
} 