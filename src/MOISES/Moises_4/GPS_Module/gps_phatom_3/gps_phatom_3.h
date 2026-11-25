/*
  ******************************************************************************
  * @file   gps_phatom_3.h
  * @brief  Header file do minidriver para o protocolo UBX utilizado no GPS Phantom 3 com Arduino.h
  ******************************************************************************
*/
/*
  * Sim: estou definindo tudo no header, já que o projeto é pequeno.
  * Caso em MODO_SOLO: Para uma melhor experiência use o terminal (shell/cmd/PowerShell), Tera Term ou Putty para ver as cores ANSI.
  * Link para a documentação: https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf.
*/

#ifndef GPS_TODOS_OS_DADOS_
#define GPS_TODOS_OS_DADOS_

#pragma message("gps_phatom_3.h visa o uso do Arduino.h")

#include <HardwareSerial.h>
#include <math.h>
#include <time.h>  // For gmtime
#include "utils.h" // Esse código depende da lógica do utils.h

// UBX é Little-Endian
// Comandos do Message struture do NAV-PVT e NAV-SOL
constexpr u8 sync_1 = 0xB5;
constexpr u8 sync_2 = 0x62;
constexpr u8 cfg_class = 0x01;
constexpr u8 msg_id = 0x06;
constexpr u8 nav_class = 0x01;
constexpr u8 pvt_id = 0x07;
constexpr u8 sol_id = 0x06;

/*
 * A estrutura é tal que: 
 * {sync_1, sync_2, class, id, byte_len_LSB, byte_len_MSB, payload, times_per_cycle, ck_a, ck_b}
 * O payload varia de tamanho (byte_len) de acordo com o comando.
 * Para mais informações veja a seção 32.2 UBX Frame Structure do documento.
*/


// Constantes WGS84 para conversão ECEF (usadas no backup NAV-SOL)
constexpr float A = 6378137.0f;
constexpr float B = 6356752.3142f;
constexpr float E2 = ((1 - (B * B)) / (A * A));
constexpr long GPS_EPOCH_SECONDS = 315964800UL;

// --- No gps_phatom_3.h ---

enum class EstadoDoSistema {
    CALIBRACAO, 
    ESTAVEL,    
    EM_VOO,     // Adicionado para a trava de decolagem
    POUSADO,    
    ERRO_CALIBRACAO,
    ERRO_CRITICO 
};


// Variáveis de controle de pouso
extern ulong lastMsg;
extern HardwareSerial SerialGPS;
extern float altitudeInicial;
extern float altitudeMaximaAtingida;
extern EstadoDoSistema estado_atual;
extern ulong tempoInicioEstabilizacao;
extern bool confirmandoPouso;
extern bool decolou;
extern float altRelativa;


u8 gpsSetup(u8* quant_nav);
void sendUBX(u8 *msg, u8 len);
void processGPS(u8 mClass, u8 mId, u8 *ptr, u16 len);
bool ackConfirmation(u8 msg_cls, u8 msg_id);
inline void verificarAltitude(double altGeral, float velV, double lat, double lon);

u8 gpsSetup(u8* quant_nav){
  // Ativar NAV-PVT (Mensagem completa: Tempo, Posição, Velocidade)
  u8 enable_nav_pvt[] = {sync_1, sync_2, cfg_class, msg_id, 0x03, 0x00, nav_class, pvt_id, 0x01};
  sendUBX(enable_nav_pvt, sizeof(enable_nav_pvt));
  bool confirm_pvt = ackConfirmation(nav_class, pvt_id);

  // Ativar NAV-SOL (Mensagem de suporte para redundância)
  u8 enable_nav_sol[] = {sync_1, sync_2, cfg_class, msg_id, 0x03, 0x00, nav_class, sol_id, 0x01};
  sendUBX(enable_nav_sol, sizeof(enable_nav_sol));
  bool confirm_sol = ackConfirmation(nav_class, sol_id);


  if (confirm_pvt && confirm_sol){ return 0x11; } 
  else if (confirm_pvt){ return 0x01; }
  else if (confirm_sol){ return 0x10; }
  
  return 0x00;
}

void sendUBX(u8 *msg, u8 len) {

  u8 ck_a = 0, ck_b = 0;
  for (int i = 2; i < len; i++) { ck_a += msg[i]; ck_b += ck_a; }
  SerialGPS.write(msg, len); SerialGPS.write(ck_a); SerialGPS.write(ck_b);


}

// Confirma o recebimento do comando enviando o ACK-ACK 
bool ackConfirmation(u8 msg_cls, u8 msg_id) {
  u8 state = 0;
  u8 msgClass = 0, msgId = 0;
  u16 msgLen = 0, payloadCount = 0;
  u8 payload[2];
  bool confirm = false;
  ulong time_ack = millis();

  // O loop roda por no máximo 5 segundos esperando a resposta
  while (millis() - time_ack < 5000) {
    
    // SÓ LÊ se houver dado físico no cabo
    if (SerialGPS.available()) {
      u8 c = SerialGPS.read();

      switch (state) {
        case 0: if (c == 0xB5) state = 1; break;
        case 1: if (c == 0x62) state = 2; else state = 0; break;
        case 2: msgClass = c; state = 3; break;
        case 3: msgId = c; state = 4; break;
        case 4: msgLen = c; state = 5; break;
        case 5: msgLen |= (c << 8); payloadCount = 0; state = 6; break;
        case 6:
          if (payloadCount < sizeof(payload)) { payload[payloadCount] = c; }
          payloadCount++; // Incrementa SEMPRE
          if (payloadCount >= msgLen) {
            // Verifica se a mensagem recebida é um pacote ACK-ACK (0x05, 0x01)
            if (msgClass == 0x05 && msgId == 0x01) {
              // O payload do ACK contém a Classe e o ID que foram aceitos
              if (payload[0] == msg_cls && payload[1] == msg_id) {
                confirm = true;
              }
            }
            state = 0; // Reseta para o próximo pacote caso não seja o ACK
          }
          break;
      }
    }
    if (confirm) break; // Sai do loop imediatamente se confirmou
  }
  return confirm;
}

void processGPS(u8 mClass, u8 mId, u8 *ptr, u16 len) {

  if (mClass != 0x01) return; // nav_class
  double lat = 0.0f, lon = 0.0f, altGeral = 0.0f;
  float velH = 0.0f, velV = 0.0f;
  int ano = 0, mes = 0, dia = 0, hora = 0, min = 0, seg = 0, numSV = 0;
  bool fixValido = false;

  // --- EXTRAÇÃO VIA NAV-PVT (Mensagem mais completa) ---
  if (mId == pvt_id && len >= 92) {
    ano = TRANSF_16_BIT(ptr, 4); mes = ptr[6]; dia = ptr[7];
    hora = ptr[8]; min = ptr[9]; seg = ptr[10];
    numSV = ptr[23];

    // CONVERSÃO SEGURA PARA BRASÍLIA (UTC-3)
    // Usamos a struct tm e o mktime para lidar automaticamente com viradas de dias/meses/anos bissextos
    struct tm t_brt = {0};
    t_brt.tm_year = ano - 1900;
    t_brt.tm_mon = mes - 1;
    t_brt.tm_mday = dia;
    t_brt.tm_hour = hora - 3; // Subtrai 3 horas do UTC (Brasília)
    t_brt.tm_min = min;
    t_brt.tm_sec = seg;
    
    mktime(&t_brt); // O C++ recalcula e normaliza a data inteira instantaneamente
    ano = t_brt.tm_year + 1900;
    mes = t_brt.tm_mon + 1;
    dia = t_brt.tm_mday;
    hora = t_brt.tm_hour;

    lon = static_cast<double>((TRANSF_32_BIT(ptr, 24)) / 10000000.0);
    lat = static_cast<double>((TRANSF_32_BIT(ptr, 28)) / 10000000.0);
    altGeral = static_cast<double>((TRANSF_32_BIT(ptr, 36)) / 1000.0);

    i32 gSpeed = TRANSF_32_BIT(ptr, 60);
    velH = static_cast<float>((gSpeed / 1000.0) * 3.6);
    
    i32 vSpeed = TRANSF_32_BIT(ptr, 64);
    velV = static_cast<float>((vSpeed / 1000.0) * -1.0);
    
    fixValido = static_cast<bool>(ptr[20] >= 3);

    // --- PRINTS DA NAV-PVT ---
    printlnLBlue(T("\r\n====================================================================="));
    printlnLBlue(T(">>>>>> DADOS RECEBIDOS: NAV-PVT (Posição, Velocidade e Tempo) <<<<<<"));
    printlnLBlue(T("====================================================================="));

    printlnLYellow(T("#### Tempo (Brasília - BRT)"));
    printflnWhite(T("- Data: %04d-%02d-%02d"), ano, mes, dia);
    printflnWhite(T("- Hora: %02d:%02d:%02d"), hora, min, seg);

    printlnLYellow(T("\n#### Posição Geodésica"));
    printflnWhite(T("- Latitude:  %.7f graus"), lat);
    printflnWhite(T("- Longitude: %.7f graus"), lon);
    printflnWhite(T("- Altitude:  %.2f m"), altGeral);

    printlnLYellow(T("\n#### Cinemática"));
    printflnWhite(T("- Velocidade Horizontal: %.2f km/h"), velH);
    printflnWhite(T("- Velocidade Vertical:   %.2f m/s"), velV);

    printlnLYellow(T("\n#### Status de Conexão"));
    printWhite(T("- Satélites Rastreados: "));
    if (numSV >= 6) { printflnLGreen(T("%d"), numSV); } 
    else { printflnLRed(T("%d"), numSV); }

    printWhite(T("- Fix Valido (>= 3): "));
    if (fixValido) { printflnLGreen(T("SIM (Sinal de Navegação Estável)")); } 
    else { printflnLRed(T("NÃO (Buscando sinal...)")); }
    
    lastMsg = millis();
  }
  
  // --- EXTRAÇÃO VIA NAV-SOL (Backup/ECEF) ---
  else if (mId == sol_id && len >= 52) {

    numSV = ptr[47];
    i16 week = TRANSF_16_BIT(ptr, 8);
    u32 iTOW = TRANSF_U32_BIT(ptr, 0);
    i32 fTOW = TRANSF_32_BIT(ptr, 4);
    
    // CONVERSÃO SEGURA PARA BRASÍLIA (UTC-3)
    long gpsSec = GPS_EPOCH_SECONDS + (week * 604800UL) + (iTOW / 1000UL);
    time_t t = gpsSec - 10800UL; // <-- Subtração exata de 10800 segundos (3 horas) antes de extrair a data
    struct tm *tm = gmtime(&t);
    ano = tm->tm_year + 1900; mes = tm->tm_mon + 1; dia = tm->tm_mday;
    hora = tm->tm_hour; min = tm->tm_min; seg = tm->tm_sec;

    i32 ecefX = TRANSF_32_BIT(ptr, 12);
    i32 ecefY = TRANSF_32_BIT(ptr, 16);
    i32 ecefZ = TRANSF_32_BIT(ptr, 20);
    double X = static_cast<double>(ecefX / 100.0);
    double Y = static_cast<double>(ecefY / 100.0);
    double Z = static_cast<double>(ecefZ / 100.0);
    double p = sqrt(X*X + Y*Y);
    double theta = atan2(Z * A, p * B);
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);
    double sin3 = sinTheta * sinTheta * sinTheta;
    double cos3 = cosTheta * cosTheta * cosTheta;
    lat = atan2(Z + E2 * B * sin3, p - E2 * A * cos3) * 180.0 / M_PI;
    lon = atan2(Y, X) * 180.0 / M_PI;
    double sinLat = sin(lat * M_PI / 180.0);
    double N = A / sqrt(1 - E2 * sinLat * sinLat);
    altGeral = p / cos(lat * M_PI / 180.0) - N;
    u8 gpsFix = ptr[10];
    fixValido = static_cast<bool>(gpsFix >= 3);

    // --- PRINTS DA NAV-SOL ---
    printlnLBlue(T("\r\n====================================================================="));
    printlnLBlue(T(">>>>>> DADOS RECEBIDOS: NAV-SOL (Solução e ECEF Raw) <<<<<<"));
    printlnLBlue(T("====================================================================="));

    printlnLYellow(T("#### iTOW (Tempo da semana em milissegundos)"));
    printflnWhite(T("- Raw: %u ms"), iTOW);
    printlnLCyan(T("- Conversões:"));
    printflnWhite(T("  - Segundos: %u s"), iTOW / 1000);
    printflnWhite(T("  - Minutos: %u min"), (iTOW / 1000) / 60);
    printflnWhite(T("  - Horas: %.2f h"), (iTOW / 1000.0) / 3600.0);
    printflnWhite(T("  - Dias: %.2f dias"), (iTOW / 1000.0) / 86400.0);

    printlnLYellow(T("\n#### fTOW (Fração de nanosegundos do iTOW)"));
    printflnWhite(T("- Raw: %d ns"), fTOW);
    printlnLCyan(T("- Conversões:"));
    printflnWhite(T("  - Milissegundos: %.6f ms"), fTOW / 1e6);
    printflnWhite(T("  - Segundos: %.9f s"), fTOW / 1e9);

    printlnLYellow(T("\n#### week (Semana GPS)"));
    printflnWhite(T("- Raw: %d semanas"), week);
    printlnLCyan(T("- Conversões:"));
    printflnWhite(T("  - Dias: %d dias"), week * 7);
    printflnWhite(T("  - Data atual convertida: %04d-%02d-%02d"), ano, mes, dia);

    printlnLYellow(T("\n#### gpsFix (Tipo de fix GPS)"));
    printflnWhite(T("- Raw: %u"), gpsFix);
    printlnLCyan(T("- Conversões:"));
    const char* fixDesc[] = {"No Fix", "Dead Reckoning", "2D Fix", "3D Fix", "GPS + DR", "Time Only"};
    printflnWhite(T("  - Descrição: %s"), (gpsFix <= 5) ? fixDesc[gpsFix] : "Unknown");

    u8 flags = ptr[11];
    printlnLYellow(T("\n#### flags (Flags de status)"));
    printflnWhite(T("- Raw: %08b (binário)"), flags);
    printLCyan(T("- Bits simples ativos: "));
    if (flags & 0x01) printWhite(T("GPS_Fix_OK "));
    if (flags & 0x02) printWhite(T("Diff_Soln "));
    if (flags & 0x04) printWhite(T("WKNSET "));
    if (flags & 0x08) printWhite(T("TOWSET "));
    printlnReset(T("")); 

    printlnLYellow(T("\n#### Coordenadas ECEF (X, Y, Z)"));
    printflnWhite(T("- ecefX Raw: %d cm (%.2f m)"), ecefX, ecefX / 100.0);
    printflnWhite(T("- ecefY Raw: %d cm (%.2f m)"), ecefY, ecefY / 100.0);
    printflnWhite(T("- ecefZ Raw: %d cm (%.2f m)"), ecefZ, ecefZ / 100.0);

    printlnLYellow(T("\n#### ECEF Convertido para Geodésico"));
    printflnWhite(T("- Latitude:  %.7f graus"), lat);
    printflnWhite(T("- Longitude: %.7f graus"), lon);
    printflnWhite(T("- Altitude Elipsoidal: %.2f m"), altGeral);

    u32 pAcc = TRANSF_U32_BIT(ptr, 24);
    printlnLYellow(T("\n#### pAcc (Precisão 3D estimada)"));
    printflnWhite(T("- Raw: %u cm (%.2f m)"), pAcc, pAcc / 100.0);

    i32 ecefVX = TRANSF_32_BIT(ptr, 28);
    i32 ecefVY = TRANSF_32_BIT(ptr, 32);
    i32 ecefVZ = TRANSF_32_BIT(ptr, 36);
    printlnLYellow(T("\n#### Velocidades Vetoriais ECEF"));
    printflnWhite(T("- ecefVX: %d cm/s (%.2f m/s)"), ecefVX, ecefVX / 100.0);
    printflnWhite(T("- ecefVY: %d cm/s (%.2f m/s)"), ecefVY, ecefVY / 100.0);
    printflnWhite(T("- ecefVZ: %d cm/s (%.2f m/s)"), ecefVZ, ecefVZ / 100.0);

    u32 sAcc = TRANSF_U32_BIT(ptr, 40);
    printlnLYellow(T("\n#### sAcc (Precisão da velocidade)"));
    printflnWhite(T("- Raw: %u cm/s (%.2f m/s)"), sAcc, sAcc / 100.0);

    u16 pDOP = TRANSF_U16_BIT(ptr, 44);
    printlnLYellow(T("\n#### pDOP (Diluição de precisão)"));
    printflnWhite(T("- Raw: %u (Valor real: %.2f)"), pDOP, pDOP / 100.0);

    lastMsg = millis();
  }

  // --- TRAVA DE FIX E CHECAGEM DE ALTITUDE ---
  if (fixValido) {
    if (altitudeInicial == -999.0 && numSV >= 6) {
        altitudeInicial = altGeral;
    }
    altRelativa = (altitudeInicial == -999.0) ? 0.0 : (altGeral - altitudeInicial);
    lastMsg = millis();
    verificarAltitude(altGeral, velV, lat, lon); // Comentar se não quiser detecção de decolagem/pouso.
  }
}

// Comentar se não quiser detecção de decolagem/pouso.
inline void verificarAltitude(double altGeral, float velV, double lat, double lon){
  // 1. Cálculo da Altitude Relativa (AGL)
  altRelativa = (altitudeInicial == -999.0) ? 0.0 : (altGeral - altitudeInicial);

  // 2. GATILHO DE DECOLAGEM (LIFTOFF)
  // Se subir mais de 10m com velocidade vertical > 5m/s
  if (!decolou && velV > 5.0 && altRelativa > 10.0) {
      decolou = true;
      estado_atual = EstadoDoSistema::EM_VOO;
      printflnLRed(T("\n>>>>>> LIFTOFF DETECTADO! ALT: %.2fm <<<<<<"), altRelativa);
  }

  // 3. REGISTO DE APOGEU (Ponto mais alto)
  if (decolou && altRelativa > altitudeMaximaAtingida) {
      altitudeMaximaAtingida = altRelativa;
  }

  // 4. LÓGICA DE DETECÇÃO DE POUSO

  // Critérios: Vel. Vertical baixa E estar abaixo de 20% da altura máxima
  // Isso evita detecção falsa no topo da trajetória (apogeu)
  bool crit_pouso = fabs(velV) < 0.5 && altRelativa < (altitudeMaximaAtingida * 0.2);

  if (decolou && crit_pouso) {
    if (!confirmandoPouso) {
        tempoInicioEstabilizacao = millis();
        confirmandoPouso = true;
    }
    
    // Precisa estar parado por 5 segundos para confirmar
    if (confirmandoPouso && (millis() - tempoInicioEstabilizacao > 5000)) {
        estado_atual = EstadoDoSistema::POUSADO;
        decolou = false; // Missão cumprida
        printlnLGreen(T("\n================================================================="));
        printflnLGreen(T("\n>>>>>> POUSO CONFIRMADO EM: Lat %.7f, Lon %.7f <<<<<<"), lat, lon);
        printlnLGreen(T("================================================================="));
    }
    } else {
        confirmandoPouso = false; // Reset se o foguetão se mexer
    }
}


#endif