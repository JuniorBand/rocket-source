#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <BLEDevice.h> // Esse e os headers abaixo incluem as bibliotecas do Bluetooth Low Energy
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


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

void enviarTabelaBluetooth(uint16_t tempoDecorrido, float altitude, float acel_x, float acel_y, float acel_z, 
                            float gyro_x, float gyro_y, float gyro_z, bool paraquedasAcionado); // Envia os dados para o Bluetooth.
// void atualizarBuffer(float leitura); // Atualiza o buffer de leituras com a nova leitura de altitude.
 void inicializarBLE(const char* deviceName); // Inicializa o Bluetooth Low Energy (BLE) e configura o servidor e as características.

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

#endif // FUNCTIONS_H