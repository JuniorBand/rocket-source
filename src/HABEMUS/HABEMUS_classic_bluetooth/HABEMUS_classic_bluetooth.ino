#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <BluetoothSerial.h>
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
BluetoothSerial serialBT;
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
void enviarTabelaBluetooth(float altitude, float acel_x, float acel_y, float acel_z, 
                           float gyro_x, float gyro_y, float gyro_z, bool paraquedasAcionado); // Envia os dados para o Bluetooth.
void atualizarBuffer(float leitura); // Atualiza o buffer de leituras com a nova leitura de altitude.

void setup() {
  Serial.begin(115200);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_MOSFET_1, OUTPUT);
  digitalWrite(PINO_MOSFET_1, LOW);
  pinMode(PINO_MOSFET_2, OUTPUT);
  digitalWrite(PINO_MOSFET_2, LOW);
  serialBT.begin("ESP32-BT");
  Serial.println("Bluetooth inicializado. Conecte-se ao dispositivo via Serial Bluetooth Terminal.");

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

  Serial.println(F("Iniciando fase de calibracao de altitude..."));

  acionarBuzzer(250, 250); // Buzzer para indicar início da calibração.
}

void loop() {
  //Lendo os valores de altura, aceleração e angulação
  float altitude_atual_1 = bmp1.readAltitude(1013.25);
  float altitude_atual_2 = bmp1.readAltitude(1013.25);

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

      // if(SerialBT.available()){
      //     serialBT.write()
      // }

    }

    delay(100); 
    return;
  }
  
  /* --- Fase de Operação Normal (Após a Calibração Inicial) --- */  
  float altitude_relativa = calcularMedia(buffer_leituras);
  printDados(altitude_relativa, a1, a2, g1, g2);

  //(OK)
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

  float nova_media = soma_buffer /  JANELA_LEITURAS; // Usa número real de leituras 
  float altitude_relativa = nova_media - leitura_inicial;
  return altitude_relativa; // Retorna a altitude relativa ajustada.
}

void atualizarBuffer(float leitura) {
  if (leitura != 0.0f && !isnan(leitura)) { // Prevenir leituras inválidas.
    buffer_leituras[indice_buffer] = leitura;
    indice_buffer = (indice_buffer + 1) % JANELA_LEITURAS;
  }
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
    return SENSOR_FALHA_ZERADO_AMBOS_OU_UM_NAN;
  } else {
    contador_leituras_zero = 0;
  }

  // Se um dos sensores está falhando, usa a leitura do outro sensor.
  if ((isnan(altitude_atual_2) || (altitude_atual_2 == 0.0f)) &&
      (!isnan(altitude_atual_1) && !(altitude_atual_1 == 0.0f))){
    Serial.println(F("ALERTA: O sensor 2 esta falhando, usando dados do sensor 1!"));
    atualizarBuffer(altitude_atual_1);
    return SENSOR_OK;
  }

  if ((isnan(altitude_atual_1) || (altitude_atual_1 == 0.0f)) &&
      (!isnan(altitude_atual_2) && !(altitude_atual_2 == 0.0f))){
    Serial.println(F("ALERTA: O sensor 1 esta falhando, usando dados do sensor 2!"));
    atualizarBuffer(altitude_atual_2);
    return SENSOR_OK;
  }

  // Se ambos os sensores estão funcionando, atualiza o buffer com a média.
  atualizarBuffer((altitude_atual_1 + altitude_atual_2)/2);
  return SENSOR_OK; // Indica que os sensores estão medindo corretamente.
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

void printDados(float altitude_relativa, sensors_event_t a1, sensors_event_t a2, sensors_event_t g1, sensors_event_t g2){
  Serial.print(F("Altitude = "));
  Serial.print(altitude_relativa);
  Serial.println(F(" m"));

  float media_ac_x = (a1.acceleration.x + a2.acceleration.x)/2;
  float media_ac_y = (a1.acceleration.y + a2.acceleration.y)/2;
  float media_ac_z = (a1.acceleration.z + a2.acceleration.z)/2;
  float media_gyro_x = (g1.acceleration.x + g2.acceleration.x)/2;
  float media_gyro_y = (g1.acceleration.y + g2.acceleration.y)/2;
  float media_gyro_z = (g1.acceleration.z + g2.acceleration.z)/2;

  //Print das acelerações na memória Flash 
  //printMedidas(media_ac_x, media_ac_y, media_ac_z, media_gyro_x, media_gyro_y, media_gyro_z)
  Serial.print(F("Acceleration X: ")); Serial.print(media_ac_x); Serial.print(F(" m/s^2"));
  Serial.print(F("; Y: ")); Serial.print(media_ac_y); Serial.print(F(" m/s^2"));
  Serial.print(F("; Z: ")); Serial.print(media_ac_z); Serial.println(F(" m/s^2."));

  //Print das angulações na memória Flash 
  Serial.print(F("Rotation X: ")); Serial.print(media_gyro_x); Serial.print(F(" rad/s"));
  Serial.print(F("; Y: ")); Serial.print(media_gyro_y); Serial.print(F(" rad/s"));
  Serial.print(F("; Z: ")); Serial.print(media_gyro_z); Serial.println(F(" rad/s."));

  //Print no cartão SD
  if (dataFile) { // Se o arquivo foi aberto com sucesso:
    dataFile.println("Altitude: " + String(altitude_relativa) + ", "); // Registra F(a altitude no ar)quivo.
    
    //Print das acelerações na memória Flash 
    dataFile.print("Acceleration X: "+ String(media_ac_x) + " m/s^2");
    dataFile.print("; Y: " + String(media_ac_y) + " m/s^2");
    dataFile.println("; Z: " + String(media_ac_z) + " m/s^2. ");

    //Print das angulações na memória Flash 
    dataFile.print("Rotation X: " + String(media_gyro_x) + " rad/s");
    dataFile.print("; Y: " + String(media_gyro_y) + " rad/s");
    dataFile.print("; Z: " + String(media_gyro_z) + " rad/s");
    dataFile.println(); // Nova linha para a próxima entrada de dados.
  }

  // Envia os dados via Bluetooth
  enviarTabelaBluetooth(altitude_relativa, media_ac_x, media_ac_y, media_ac_z, 
                        media_gyro_x, media_gyro_y, media_gyro_z, paraquedasAcionado);

}

void enviarTabelaBluetooth(float altitude, float acel_x, float acel_y, float acel_z, 
                           float gyro_x, float gyro_y, float gyro_z, bool paraquedasAcionado) {
  // Move o cursor para o início da tela e limpa a tela
  serialBT.print("\033[H"); // Move o cursor para o início da tela
  serialBT.print("\033[J"); // Limpa a tela

  // Calcula o tempo decorrido em segundos
  uint16_t tempoDecorrido = millis() / 1000; // Converte para segundos

  // Envia os nomes das variáveis (cabeçalho da tabela)
  serialBT.println(F("Tempo | Altitude | Aceleração (X, Y, Z) | Giroscópio (X, Y, Z) | Paraquedas"));
  serialBT.println(F("------|----------|----------------------|----------------------|-----------"));

  // Envia os valores atualizados
  serialBT.print(tempoDecorrido); serialBT.print(F(" s | "));
  serialBT.print(altitude, 2); serialBT.print(F(" m | "));
  serialBT.print(acel_x, 2); serialBT.print(F(", "));
  serialBT.print(acel_y, 2); serialBT.print(F(", "));
  serialBT.print(acel_z, 2); serialBT.print(F(" m/s^2 | "));
  serialBT.print(gyro_x, 2); serialBT.print(F(", "));
  serialBT.print(gyro_y, 2); serialBT.print(F(", "));
  serialBT.print(gyro_z, 2); serialBT.print(F(" rad/s | "));
  serialBT.println(paraquedasAcionado ? F("SIM") : F("NAO"));
}

void verificarSensores(uint8_t ADDRESS_1, uint8_t ADDRESS_2, uint8_t ADDRESS_3, uint8_t  ADDRESS_4){
  //Tentando inicializar os BMPs
  Serial.println(F("Tentando inicializar o primeiro BMP280 no endereco 0x76..."));
  if (!bmp1.begin(ADDRESS_1)) {
    Serial.println(F("================================================="));
    Serial.println(F("ERRO CRÍTICO: Nao foi possivel encontrar o primeiro sensor BMP280 valido no 0x76!"));
    escanearI2C();
    finalizarMissao(F("ERRO CRITICO NA INICIALIZACAO DO SENSOR"));
  } else {
    Serial.println(F("Primeiro BMP280 encontrado com sucesso no endereco 0x76!"));
  }


  Serial.println(F("Tentando inicializar o segundo BMP280 no endereco 0x77..."));
  if (!bmp2.begin(ADDRESS_2)) {
    Serial.println(F("================================================="));
    Serial.println(F("ERRO CRÍTICO: Nao foi possivel encontrar o segundo sensor BMP280 valido no 0x77!"));
    escanearI2C();
    finalizarMissao(F("ERRO CRITICO NA INICIALIZACAO DO SENSOR"));
  } else {
    Serial.println(F("Segundo BMP280 encontrado com sucesso no endereco 0x77!"));
  }

  //Tentando inicializar os MPUs
  Serial.println(F("Tentando inicializar o primeiro MPU6050 no endereco 0x68..."));
  if (!mpu1.begin(ADDRESS_3)) {
    Serial.println(F("================================================="));
    Serial.println(F("ERRO CRÍTICO: Nao foi possivel encontrar o primeiro sensor MPU6050 valido no 0x68!"));
    escanearI2C();
    finalizarMissao(F("ERRO CRITICO NA INICIALIZACAO DO SENSOR"));
  } else {
  Serial.println(F("Primeiro MPU6050 encontrado com sucesso no endereco 0x68!"));
  }

  Serial.println(F("Tentando inicializar o primeiro MPU6050 no endereco 0x69..."));
  if (!mpu2.begin(ADDRESS_4)) {
    Serial.println(F("================================================="));
    Serial.println(F("ERRO CRÍTICO: Nao foi possivel encontrar o segundo sensor MPU6050 valido no 0x69!"));
    escanearI2C();
    finalizarMissao(F("ERRO CRITICO NA INICIALIZACAO DO SENSOR"));
  } else {
  Serial.println(F("Segundo MPU6050 encontrado com sucesso no endereco 0x69!"));
  }
}

void acionarBuzzer(int timeOn, int timeOff) { // *[ALTERAÇÃO]* Buzzer contínuo de timeOn em timeOff ms.
  digitalWrite(PINO_BUZZER, HIGH); // Aciona o buzzer.
  delay(timeOn);
  digitalWrite(PINO_BUZZER, LOW); // Desliga o buzzer.
  delay(timeOff); 
}

void escanearI2C() {
  byte erro, endereco;
  int numDispositivos;
  Serial.println(F("Iniciando Escaner I2C..."));
  numDispositivos = 0;
  
  for (endereco = 1; endereco < 127; endereco++) {
    Wire.beginTransmission(endereco);
    erro = Wire.endTransmission();
    
    if (erro == 0) {
      Serial.print(F("Dispositivo I2C encontrado no endereco 0x"));
      
      if (endereco < 16) { Serial.print(F("0")); }
      Serial.print(endereco, HEX);
      Serial.println(F("   !"));
      numDispositivos++;

    } else if (erro == 4) {

      Serial.print(F("Erro desconhecido no endereco 0x"));

      if (endereco < 16) { Serial.print(F("0")); }
      Serial.println(endereco, HEX);

    }
  }

  if (numDispositivos == 0) {
    Serial.println(F("Nenhum dispositivo I2C encontrado."));
  }

  Serial.println(F("================================================="));
  delay(1000);
}