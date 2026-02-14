#include <Functions.h> // Inclui o cabeçalho de funções personalizadas.

/* Definições de pinos/endereços */
const uint8_t PINO_BUZZER = 12;
const uint8_t PINO_CHIP = 9;
#define META_APOGEU 100.0f // Meta de apogeu desejada em metros (ajustável).


/* Limites e configurações de voo */
const float LIMITE_SALTO_ANOMALO = 75.0f; // Limite anormal de salto entre leituras seguidas.
const float ALTITUDE_DE_LANCAMENTO = 30.0f; // Altitude para considerar o foguete lançado.
const float QUEDA = 3.0f; // Queda de segurança para o acionamento do para-quedas.


// Variáveis globais para controle do estado da missão
float apogeu = 0.0f;
bool paraquedasAcionado = false;
bool mosfetsAcionados = false;
float leitura_inicial = 0.0f;
const uint8_t JANELA_LEITURAS = 5;
float buffer_leituras[JANELA_LEITURAS];
uint8_t indice_buffer = 0; // já serve como contador de leituras
uint8_t contador_leituras_zero = 0;
unsigned long tempoDecorrido = 0; 


/* Protótipos das funções */

float calcularMedia(float buffer_leituras[]); // Calcula a média das leituras de altitude.

void setup() {
  Serial.begin(9600);
 
  // Inicialização do Bluetooth Low Energy (BLE)
  // O nome do dispositivo ("HABEMUS_S3_ROCKET") será visível para outros dispositivos BLE
  inicializarBLE("HABEMUS_S3_ROCKET"); // Inicializa o BLE e configura o servidor e as características.
  

  for(int i = 0; i < JANELA_LEITURAS; i++) {
    buffer_leituras[i] = (float) (300 + i*15); // Inicializa o buffer de leituras com zeros.
  }
  Serial.println(F("Iniciando fase de calibracao de altitude..."));

}

void loop() {
    
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

  /* --- Fase de Operação Normal (Após a Calibração Inicial) --- */  
  float altitude_relativa = calcularMedia(buffer_leituras);

  if(millis() - tempoDecorrido >= 3000){
    Serial.println(F("HEY do loop!")); 
    tempoDecorrido = millis();
    enviarTabelaBluetooth(tempoDecorrido, altitude_relativa, 0.5f, 30.0f, 0.7f, 3.0f, 6.0f, 9.0f, paraquedasAcionado);
  } // Debug para verificar se o loop está rodando.
  
  
  delay(10);
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
