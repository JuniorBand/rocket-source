#include <Wire.h>
#include <Adafruit_BMP280.h> // Inclui a biblioteca do altímetro/barômetro.
#include <Adafruit_MPU6050.h> // Inclui a biblioteca do acelerômetro/giroscópio
#include <SD.h> // Inclui a biblioteca para leitura/escrita em cartão SD.
#include <BLEDevice.h> // Esse e os headers abaixo incluem as bibliotecas do Bluetooth Low Energy
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// É crucial usar UUIDs únicos para o serviço e características BLE.
// Você pode gerar UUIDs aleatórios em https://www.uuidgenerator.net/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // UUID do Serviço Principal

// Características para cada tipo de dado a ser transmitido
#define ALTITUDE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8" // UUID para a característica de Altitude
#define ACCEL_CHARACTERISTIC_UUID    "a0b2c3d4-e5f6-7890-1234-567890abcdef" // UUID para a característica de Aceleração e Giroscópio
#define PARACHUTE_CHARACTERISTIC_UUID "f0e9d8c7-b6a5-4321-fedc-ba9876543210" // UUID para a característica de Status do Paraquedas

/* Definições de pinos/endereços */
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
const float LIMITE_SALTO_ANOMALO = 75.0f; // Limite anormal de salto entre leituras seguidas.
const float ALTITUDE_DE_LANCAMENTO = 30.0f; // Altitude para considerar o foguete lançado.
const float QUEDA = 3.0f; // Queda de segurança para o acionamento do para-quedas.
Adafruit_BMP280 bmp1(BMP280_1_ADDRESS); // Cria instância do sensor BMP280 no endereço respectivo (no próprio construtor).
Adafruit_BMP280 bmp2(BMP280_2_ADDRESS);
Adafruit_MPU6050 mpu1; // Cria instância, mas não aceita o endereço no construtor (só no .begin()).
Adafruit_MPU6050 mpu2;
File dataFile; // Cria um objeto para o arquivo de dados no cartão SD.

// Status dos sensores. 
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
void printDados(uint16_t tempoDecorrido, float altitude, sensors_event_t a1, sensors_event_t a2, sensors_event_t g1, sensors_event_t g2); // Imprime os dados no Serial e no cartão SD. 
void acionarBuzzer(int timeOn, int timeOff);  // Buzzer contínuo de timeOn em timeOff ms.
float calcularMedia(float buffer_leituras[]); // Calcula a média das leituras de altitude.
void enviarTabelaBluetooth(uint16_t tempoDecorrido, float altitude, float acel_x, float acel_y, float acel_z, 
                           float gyro_x, float gyro_y, float gyro_z, bool paraquedasAcionado); // Envia os dados para o Bluetooth.
void atualizarBuffer(float leitura); // Atualiza o buffer de leituras com a nova leitura de altitude.

// Definições das características BLE
BLECharacteristic* pAltitudeCharacteristic;
BLECharacteristic* pAcelGyroCharacteristic;
BLECharacteristic* pParachuteCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;

// Callback para eventos de conexão/desconexão BLE
// Esta classe define o comportamento do servidor BLE quando um dispositivo cliente se conecta ou desconecta.
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Dispositivo BLE conectado.");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Dispositivo BLE desconectado.");
      // Após a desconexão, reinicia o advertising para permitir novas conexões
      // Usa pServer->startAdvertising() que é o método correto para reiniciar o anúncio a partir do servidor.
      pServer->startAdvertising(); 
    }
};

BLEServer* pServer = NULL;
BLEAdvertising* pAdvertising = NULL; // Adicionado para gerenciar o advertising

void setup() {
  Serial.begin(115200);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_MOSFET_1, OUTPUT);
  digitalWrite(PINO_MOSFET_1, LOW);
  pinMode(PINO_MOSFET_2, OUTPUT);
  digitalWrite(PINO_MOSFET_2, LOW);
  
  // Inicialização do Bluetooth Low Energy (BLE)
  // O nome do dispositivo ("HABEMUS_S3_ROCKET") será visível para outros dispositivos BLE
  BLEDevice::init("HABEMUS_S3_ROCKET");

  // Criação do servidor BLE
  // O servidor BLE é o ponto central para interagir com dispositivos clientes (ex: seu celular).
  pServer = BLEDevice::createServer();
  // Atribui a classe de callbacks (MyServerCallbacks) ao servidor para lidar com eventos de conexão/desconexão.
  pServer->setCallbacks(new MyServerCallbacks());

  // Criação do serviço BLE
  // Um serviço agrupa características relacionadas. Aqui, temos um serviço para os dados do foguete.
  // O SERVICE_UUID é o identificador único para este serviço.
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Criação das características BLE
  // Cada característica representa um tipo específico de dado que pode ser lido ou notificado.
  // PROPRIEDADES:
  //   - PROPERTY_READ: Permite que um cliente leia o valor da característica.
  //   - PROPERTY_NOTIFY: Permite que o servidor envie notificações automáticas para clientes inscritos quando o valor da característica muda.

  // Característica para Altitude
  pAltitudeCharacteristic = pService->createCharacteristic(
                                         ALTITUDE_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  // Adiciona um descritor BLE2902, que é padrão para características com propriedade NOTIFY.
  // Ele permite que o cliente habilite/desabilite as notificações.
  pAltitudeCharacteristic->addDescriptor(new BLE2902()); 

  // Característica para Aceleração/Giroscópio
  pAcelGyroCharacteristic = pService->createCharacteristic(
                                        ACCEL_CHARACTERISTIC_UUID,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_NOTIFY
                                      );
  pAcelGyroCharacteristic->addDescriptor(new BLE2902()); 

  // Característica para Status do Paraquedas
  pParachuteCharacteristic = pService->createCharacteristic(
                                            PARACHUTE_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_NOTIFY
                                          );
  pParachuteCharacteristic->addDescriptor(new BLE2902()); 

  // Inicia o serviço BLE
  // Após criar o serviço e suas características, ele precisa ser iniciado para estar disponível.
  pService->start();

  // Configura o advertising do BLE (Anúncio Bluetooth)
  // O advertising torna o dispositivo visível para outros dispositivos BLE na área para que possam se conectar.
  // Obtém a instância do objeto de advertising do dispositivo BLE.
  pAdvertising = BLEDevice::getAdvertising();
  // Adiciona o UUID do serviço principal ao advertising, permitindo que clientes descubram o serviço.
  pAdvertising->addServiceUUID(SERVICE_UUID);
  // Habilita a resposta de varredura, o que permite que o dispositivo envie informações adicionais quando um scanner o detecta.
  pAdvertising->setScanResponse(true);
  // Define um pequeno intervalo de conexão preferido (min/max) para economizar energia, mas permite conexões rápidas.
  // 0x06 (7.5ms) e 0x12 (15ms) são valores comuns para conexões de baixa latência.
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  // Inicia o processo de advertising, tornando o dispositivo detectável.
  BLEDevice::startAdvertising();
  Serial.println("Aguardando clientes BLE...");

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
      default: // Sensores (0.0f) e (NaN) ou (NaN) e (NaN)
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

    delay(100); 
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

  delay(100);
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

void atualizarBuffer(float leitura) { // Atualizar array buffer_leituras[].
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

// Printa os dados de voo no Serial e no cartão SD.
// Recebe o tempo decorrido, altitude e eventos dos sensores de aceleração e giroscópio.
void printDados(uint16_t tempoDecorrido, float altitude, sensors_event_t a1, sensors_event_t a2, sensors_event_t g1, sensors_event_t g2) {
  Serial.print(F("Altitude = "));
  Serial.print(altitude);
  Serial.println(F(" m"));

  float media_ac_x = (a1.acceleration.x + a2.acceleration.x)/2;
  float media_ac_y = (a1.acceleration.y + a2.acceleration.y)/2;
  float media_ac_z = (a1.acceleration.z + a2.acceleration.z)/2;
  float media_gyro_x = (g1.gyro.x + g2.gyro.x)/2;
  float media_gyro_y = (g1.gyro.y + g2.gyro.y)/2;
  float media_gyro_z = (g1.gyro.z + g2.gyro.z)/2;

  //Print das acelerações na memória Flash 
  Serial.print(F("Acceleration X: ")); Serial.print(media_ac_x); Serial.print(F(" m/s^2"));
  Serial.print(F("; Y: ")); Serial.print(media_ac_y); Serial.print(F(" m/s^2"));
  Serial.print(F("; Z: ")); Serial.print(media_ac_z); Serial.println(F(" m/s^2."));

  //Print das velocidades angulares na memória Flash 
  Serial.print(F("Rotation X: ")); Serial.print(media_gyro_x); Serial.print(F(" rad/s"));
  Serial.print(F("; Y: ")); Serial.print(media_gyro_y); Serial.print(F(" rad/s"));
  Serial.print(F("; Z: ")); Serial.print(media_gyro_z); Serial.println(F(" rad/s."));

  //Print no cartão SD
  if (dataFile) { // Se o arquivo foi aberto com sucesso:

    dataFile.println("Tempo: " + String(tempoDecorrido) + " s."); // Registra a altitude no arquivo.

    dataFile.println("Altitude: " + String(altitude) + " m."); // Registra a altitude no arquivo.
    
    //Print das acelerações na memória Flash 
    dataFile.print("Acceleration X: "+ String(media_ac_x) + " m/s^2");
    dataFile.print("; Y: " + String(media_ac_y) + " m/s^2");
    dataFile.println("; Z: " + String(media_ac_z) + " m/s^2");

    //Print das velocidades angulares na memória Flash 
    dataFile.print("Rotation X: " + String(media_gyro_x) + " rad/s");
    dataFile.print("; Y: " + String(media_gyro_y) + " rad/s");
    dataFile.print("; Z: " + String(media_gyro_z) + " rad/s");
    dataFile.println(); // Nova linha para a próxima entrada de dados.
  }

  // Envia os dados formatados via Bluetooth Low Energy (BLE)
  enviarTabelaBluetooth(tempoDecorrido, altitude, media_ac_x, media_ac_y, media_ac_z, 
    media_gyro_x, media_gyro_y, media_gyro_z, paraquedasAcionado);

}

// Esta função envia os dados de voo via Bluetooth Low Energy (BLE).
void enviarTabelaBluetooth(uint16_t tempoDecorrido, float altitude, float acel_x, float acel_y, float acel_z,
                            float gyro_x, float gyro_y, float gyro_z, bool paraquedasAcionado) {
  // Apenas envia os dados se houver um dispositivo BLE conectado.
  if (deviceConnected) {
    // Formata os dados de altitude para envio via BLE como uma string e os envia via notificação.
    // Inclui o tempo decorrido.
    String altitudeData = "T: " + String(tempoDecorrido) + "s | Alt: " + String(altitude, 2) + "m";
    pAltitudeCharacteristic->setValue(altitudeData.c_str());
    pAltitudeCharacteristic->notify(); // Envia a notificação para o cliente (dispositivo conectado)

    // Formata os dados de aceleração e giroscópio para envio via BLE.
    String acelGyroDados = "Acel: X: " + String(acel_x, 2) + ", Y: " + String(acel_y, 2) + ", Z: " + String(acel_z, 2) +
                         " | Giro: X: " + String(gyro_x, 2) + ", Y: " + String(gyro_y, 2) + ", Z: " + String(gyro_z, 2);
    pAcelGyroCharacteristic->setValue(acelGyroDados.c_str());
    pAcelGyroCharacteristic->notify(); 

    // Formata o status do paraquedas para envio via BLE.
    // Utiliza 'paraquedasAcionado' para verificar o status e formatar a string.
    String parachuteStatusDados = "Paraquedas: " + String(paraquedasAcionado == 0 ? "Fechado" : "Aberto");
    pParachuteCharacteristic->setValue(parachuteStatusDados.c_str());
    pParachuteCharacteristic->notify(); 

    delay(50); // Pequeno atraso para evitar inundações de notificações BLE e sobrecarga do dispositivo.
  }
}

// Esta função escaneia o barramento I2C para detectar dispositivos conectados.
// Ela imprime os endereços dos dispositivos encontrados e finaliza a missão se nenhum dispositivo for detectado.
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

// Esta função verifica se os sensores estão funcionando corretamente nos endereços especificados.
// Se algum sensor falhar, a missão é finalizada com uma mensagem de erro.
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

void acionarBuzzer(int timeOn, int timeOff) { // Aciona o buzzer por timeOn milissegundos e desliga por timeOff milissegundos.
  digitalWrite(PINO_BUZZER, HIGH); 
  delay(timeOn);
  digitalWrite(PINO_BUZZER, LOW);  
  delay(timeOff); 
} 