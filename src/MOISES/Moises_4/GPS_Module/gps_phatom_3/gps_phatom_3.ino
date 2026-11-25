/*
  ******************************************************************************
  * @file  gps_phatom_3.h
  * @brief Source file do minidriver para o protocolo UBX utilizado no GPS Phantom 3
  ******************************************************************************
*/

/*
 * OBS: O código usa bastante o utils.h.
 *
 * OBS: Altere os intervalos de tempo atual - millis() caso prefira mudar a velocidade de recebimento das informações.
 *
 * Importante: Há uso extenso de prints, inclusive coloridos, que não devem servir para transmitir via LoRa e podem gerar gargalo no voo,
 * então, deve-se integrar a lib de transmissão LoRa com lógica mais enxuta e direta de mensagens. Mas ainda assim, são úteis para
 * testes em solo, visto que os prints coloridos funcionam bem em terminais (como explicado em gps_phantom_3.h), e podem ser adaptados para 
 * gravação local no SD Card do sistema de geolocalização.
 *
 * Atenção: Para desativar prints do arquivo binário e economizar memória/processamento no upload, avise o compilador e comente o MODO_VOO em utils.h.
 * Isso funciona bem se evitar usar Serial.print e afins diretamente, use as macros sem cor se for o caso: printReset ou printDebug e seus afins (ln/f).
 *
 * OBS: Fique atento aos messages(...)/#warnings no output, eles ajudam a entender a configuração que está sendo utilizada, não são #errors.
*/

#include <HardwareSerial.h>
#include <math.h>
#include <time.h>  // For gmtime
#include "gps_phatom_3.h"
#include "utils.h" // Esse código depende da lógica do utils.h

#define RXD2 16
#define TXD2 17
HardwareSerial SerialGPS(2);


// Variáveis de controle de exibição e altitude
float altitudeInicial = -999.0f;
float altitudeMaximaAtingida = 0.0f;
ulong lastMsg = 0UL;
ulong tempo_atual = 0UL; // Usar como freio pra não floodar o terminal
EstadoDoSistema   estado_atual = EstadoDoSistema::CALIBRACAO;
EstadoDoSistema estado_anterior = EstadoDoSistema::CALIBRACAO;
u8 quant_nav = 0x00;
ulong tempoInicioEstabilizacao = 0UL;
bool confirmandoPouso = false;
bool decolou = false;
float altRelativa = 0.0f;

// Protótipos das f
bool verificarEstado(void);
inline void erroCritico(cstr razao_erro);


void setup() {
  #ifndef MODO_VOO
    Serial.begin(115200);
    delay(200);
    printlnDebug(T("\n>>>>>> Em MODO_SOLO <<<<<<"));
    // Velocidade de comunicação do módulo Phantom 3
    SerialGPS.begin(57600, SERIAL_8N1, RXD2, TXD2);
    printlnBlue(T("\n================================================="));
    printlnBlue(T(">>>>>> SISTEMA DE MONITORAMENTO GPS <<<<<<"));
    printlnBlue(T("================================================="));
    delay(1000);
  #endif
  
  gpsSetup(&quant_nav);

  printlnDebug();

  switch (quant_nav) {
    case 0x11:
      printlnGreen(T("NAV-PVT e NAV-SOL funcionando!!!"));
      break;
    case 0x01:
      printlnYellow(T("Somente NAV-PVT funcionando!!!"));
      break;
    case 0x10:
      printlnYellow(T("Somente NAV-SOL funcionando!!!"));
      break;
    case 0x00:
      printlnRed(T("Falha no envio dos comandos para o GPS!!!"));
      estado_atual = EstadoDoSistema::ERRO_CALIBRACAO;
      break;
    default: break;
  }

  tempo_atual = millis();
}

void loop() {


  if(!verificarEstado()){ return; }

  static u8 state = 0;
  static u8 msgClass = 0, msgId = 0;
  static u16 msgLen = 0, payloadCount = 0;
  static u8 payload[100]; // 100 é o suficiente aqui.

  if (SerialGPS.available()) { // Sem uso de while()
    volatile u8 c = SerialGPS.read();

    switch (state) {
      case 0: if (c == 0xB5) state = 1; break;
      case 1: if (c == 0x62) state = 2; else state = 0; break;
      case 2: msgClass = c; state = 3; break;
      case 3: msgId = c; state = 4; break;
      case 4: msgLen = c; state = 5; break;
      case 5: msgLen |= (c << 8); payloadCount = 0; state = 6; break;
      case 6:
        if (payloadCount < sizeof(payload)){ payload[payloadCount++] = c; }
        if (payloadCount >= msgLen){ state = 7; }
        break;
      case 7:
        processGPS(msgClass, msgId, payload, msgLen);
        state = 0;
        break;
    }
  }

  // Alerta de perda de comunicação
  if (millis() - lastMsg > 5000 && lastMsg != 0 && !SerialGPS.available()) {
    if (estado_atual != EstadoDoSistema::ERRO_CRITICO) {
        estado_anterior = estado_atual; // Grava o que estava fazendo antes de cair
        estado_atual = EstadoDoSistema::ERRO_CRITICO;
    }
    lastMsg = millis();
  }

}

bool verificarEstado(void){
  bool estavel = false;
  
  // A MÁQUINA DE ESTADOS RODA LIVRE (Na velocidade do CPU)
  switch (estado_atual) {
      case EstadoDoSistema::ESTAVEL:
      case EstadoDoSistema::EM_VOO:
        estavel = true;
        break;
        
      case EstadoDoSistema::POUSADO:
        // Continua processando os dados no chão
        estavel = true; 
        if (millis() - tempo_atual >= 5000) { // Printa a cada 5s para não floodar
            printlnLYellow(T("Missão Concluída. Aguardando Resgate..."));
            tempo_atual = millis();
        }
        break;
        
      case EstadoDoSistema::CALIBRACAO:
        estado_atual = EstadoDoSistema::ESTAVEL;
        estavel = true;
        break;
        
      case EstadoDoSistema::ERRO_CALIBRACAO:
        if (millis() - tempo_atual >= 1000) {
            erroCritico(T("CALIBRAÇÃO"));
            tempo_atual = millis();
        }
        break;
        
      case EstadoDoSistema::ERRO_CRITICO:
        if (millis() - tempo_atual >= 1000) {
            erroCritico(T("CONEXÃO"));
            tempo_atual = millis();
        }
        // Tentativa inteligente de reconexão
        if (SerialGPS.available() && millis() - lastMsg < 5000) {
            estado_atual = estado_anterior; // Retorna exatamente para a fase em que estava
            lastMsg = millis();
        }
        break;
        
      default: 
        break;
  }
  
  return estavel;
}

inline void erroCritico(cstr razao_erro) {
  printlnLRed(T("\n================================================================="));
  printflnLRed(T(">>>>>> ERRO CRITICO! %s. SISTEMA PARALISADO. <<<<<<"), razao_erro);
  printlnLRed(T("================================================================="));
  return;
}
   