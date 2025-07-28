#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <BLEDevice.h> // Esse e os headers abaixo incluem as bibliotecas do Bluetooth Low Energy
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Status dos sensores. 
enum SensorStatus {
  SENSOR_OK,
  SENSOR_FALHA_AMBOS_NAN,
  SENSOR_FALHA_ZERADO_AMBOS_OU_UM_NAN,
  SENSOR_FALHA_CONSECUTIVA_ZERADA,
  SENSOR_FALHA_UM_SENSOR
};

extern File dataFile; // Cria um objeto para o arquivo de dados no cartão SD.
extern Adafruit_BMP280 bmp1; // Cria instância do sensor BMP280 no endereço respectivo (no próprio construtor).
extern Adafruit_BMP280 bmp2;
extern Adafruit_MPU6050 mpu1; // Cria instância, mas não aceita o endereço no construtor (só no .begin()).
extern Adafruit_MPU6050 mpu2;

/* Parâmetros e buffer para a média móvel */
extern const uint8_t JANELA_LEITURAS;
extern float buffer_leituras[];
extern uint8_t indice_buffer; // já serve como contador de leituras
extern bool paraquedasAcionado;
extern uint8_t contador_leituras_zero;

/* Configurações do Bluetooth Low Energy */
// É crucial usar UUIDs únicos para o serviço e características BLE.
// Você pode gerar UUIDs aleatórios em https://www.uuidgenerator.net/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // UUID do Serviço Principal.

// Características para cada tipo de dado a ser transmitido.
#define TELEMETRY_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8" // UUID para a característica de telemetria completa.

// Definições das características BLE
BLECharacteristic* pFullTelemetryCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;

BLEServer* pServer = NULL;
BLEAdvertising* pAdvertising = NULL; // Adicionado para gerenciar o advertising

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

void verificarSensores(uint8_t ADDRESS_1, uint8_t ADDRESS_2, uint8_t ADDRESS_3, uint8_t  ADDRESS_4); // Verifica se os sensores estão funcionando corretamente nos seus endereços.
SensorStatus validarSensores(float altitude_atual_1, float altitude_atual_2); // Valida as leituras dos sensores.
void printDados(uint16_t tempoDecorrido, float altitude, sensors_event_t a1, sensors_event_t a2, sensors_event_t g1, sensors_event_t g2); // Imprime os dados no Serial e no cartão SD. 
void enviarTabelaBluetooth(uint16_t tempoDecorrido, float altitude, float acel_x, float acel_y, float acel_z, 
                           float gyro_x, float gyro_y, float gyro_z, bool paraquedasAcionado); // Envia os dados para o Bluetooth.
void atualizarBuffer(float leitura); // Atualiza o buffer de leituras com a nova leitura de altitude.
void inicializarBLE(const char* deviceName); // Inicializa o Bluetooth Low Energy (BLE) e configura o servidor e as características.
void escanearI2C(void); // Escaneia o barramento I2C para detectar dispositivos.
extern void finalizarMissao(const __FlashStringHelper *razao); // Finaliza a missão com uma razão específica.

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
    // Melhoria: Usar char[] e snprintf para otimização de memória
    char allDataBuffer[256]; // Aumente o tamanho conforme necessário para todos os dados

    // Formata todos os dados em uma única string
    snprintf(allDataBuffer, sizeof(allDataBuffer),
             "T:%ds | Alt:%.2fm | Acel:X:%.2f,Y:%.2f,Z:%.2f m/s^2 | Giro:X:%.2f,Y:%.2f,Z:%.2f rad/s | P: %s",
             tempoDecorrido,
             altitude,
             acel_x, acel_y, acel_z,
             gyro_x, gyro_y, gyro_z,
             (paraquedasAcionado ? "Aberto" : "Fechado"));

    // Envia os dados formatados para a característica de telemetria completa.
    // Isso permite que o cliente BLE receba uma notificação com todos os dados de uma vez.
    // O buffer allDataBuffer contém todos os dados formatados.
    pFullTelemetryCharacteristic->setValue(allDataBuffer);
    pFullTelemetryCharacteristic->notify();

    delay(50); // Pequeno atraso para evitar inundações de notificações BLE e sobrecarga do dispositivo.
  }
}

void inicializarBLE(const char* deviceName) { // Inicializa o Bluetooth Low Energy (BLE) e configura o servidor e as características.
  // Criação do servidor BLE
  // O nome do dispositivo ("HABEMUS_S3_ROCKET") será visível para outros dispositivos BLE
  BLEDevice::init(deviceName);

  // Define o tamanho máximo da MTU (Maximum Transmission Unit) para 517 bytes (MAX) ou o que for adequado.
  // Isso deve ser feito ANTES de criar o servidor BLE
  BLEDevice::setMTU(256); // Aumenta o tamanho da MTU para permitir que pacotes maiores sejam transmitidos, se necessário.


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

  // Característica para Aceleração/Giroscópio
  pFullTelemetryCharacteristic = pService->createCharacteristic(
                                        TELEMETRY_CHARACTERISTIC_UUID,
                                        BLECharacteristic::PROPERTY_READ |
                                        BLECharacteristic::PROPERTY_NOTIFY
                                      );
  pFullTelemetryCharacteristic->addDescriptor(new BLE2902()); // Adiciona um descritor para permitir notificações.

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
}

// Esta função escaneia o barramento I2C para detectar dispositivos conectados.
// Ela imprime os endereços dos dispositivos encontrados e finaliza a missão se nenhum dispositivo for detectado.
//Talvez essa função possa ser removida, pois não é usada no código principal, mas é útil para depuração.
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
  if (!bmp1.begin(ADDRESS_1)) { // Colocar o endereço do BMP280 é redundante, mas é uma boa prática.
    Serial.println(F("Não foi possível encontrar um sensor BMP280 válido 1, verifique a fiação!"));
    escanearI2C(); // Escaneia o barramento I2C para verificar se o sensor está conectado.
    finalizarMissao(F("FALHA SENSOR BMP1"));
  }
  if (!bmp2.begin(ADDRESS_2)) { // Colocar o endereço do BMP280 é redundante, mas é uma boa prática.
    Serial.println(F("Não foi possível encontrar um sensor BMP280 válido 2, verifique a fiação!"));
    escanearI2C(); // Escaneia o barramento I2C para verificar se o sensor está conectado.
    finalizarMissao(F("FALHA SENSOR BMP2"));
  }

  //Tentando inicializar os MPUs
  if (!mpu1.begin(ADDRESS_3)) {
    Serial.println(F("Não foi possível encontrar um sensor MPU6050 válido 1, verifique a fiação!"));
    escanearI2C(); // Escaneia o barramento I2C para verificar se o sensor está conectado.
    finalizarMissao(F("FALHA SENSOR MPU1"));
  }
  if (!mpu2.begin(ADDRESS_4)) {
    Serial.println(F("Não foi possível encontrar um sensor MPU6050 válido 2, verifique a fiação!"));
    escanearI2C(); // Escaneia o barramento I2C para verificar se o sensor está conectado.
    finalizarMissao(F("FALHA SENSOR MPU2"));
  }

  escanearI2C(); // Escaneia o barramento I2C para verificar se os sensores estão conectados.
  Serial.println(F("Sensores BMP e MPU inicializados e funcionando."));
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

#endif // FUNCTIONS_H