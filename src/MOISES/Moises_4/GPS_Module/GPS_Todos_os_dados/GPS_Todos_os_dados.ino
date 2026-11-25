#include <HardwareSerial.h>
#include <math.h>
#include <time.h>  // For gmtime

#define RXD2 16
#define TXD2 17
HardwareSerial SerialGPS(2);
// Constantes WGS84 para conversão ECEF (usadas no backup NAV-SOL)
#define A 6378137.0
#define B 6356752.3142
#define E2 (1 - (B * B) / (A * A))
#define GPS_EPOCH_SECONDS 315964800UL
// Variáveis de controle de exibição e altitude
float altitudeInicial = -999.0;
unsigned long lastMsg = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  // Velocidade de comunicação do módulo Phantom 3
  SerialGPS.begin(57600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("\n==========================================================================================");
  Serial.println(" SISTEMA DE MONITORAMENTO GPS ");
  Serial.println("==========================================================================================");
  delay(1000);
  // Ativar NAV-PVT (Mensagem completa: Tempo, Posição, Velocidade)
  uint8_t enable_nav_pvt[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01};
  sendUBX(enable_nav_pvt, sizeof(enable_nav_pvt));
  // Ativar NAV-SOL (Mensagem de suporte para redundância)
  uint8_t enable_nav_sol[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01};
  sendUBX(enable_nav_sol, sizeof(enable_nav_sol));
}

void loop() {
  static uint8_t state = 0;
  static uint8_t msgClass = 0, msgId = 0;
  static uint16_t msgLen = 0, payloadCount = 0;
  static uint8_t payload[200];
  while (SerialGPS.available()) {
    uint8_t c = SerialGPS.read();
    switch (state) {
      case 0: if (c == 0xB5) state = 1; break;
      case 1: if (c == 0x62) state = 2; else state = 0; break;
      case 2: msgClass = c; state = 3; break;
      case 3: msgId = c; state = 4; break;
      case 4: msgLen = c; state = 5; break;
      case 5: msgLen |= (c << 8); payloadCount = 0; state = 6; break;
      case 6:
        if (payloadCount < sizeof(payload)) payload[payloadCount++] = c;
        if (payloadCount >= msgLen) state = 7;
        break;
      case 7:
        processGPS(msgClass, msgId, payload, msgLen);
        state = 0;
        break;
    }
  }
  // Alerta de perda de comunicação
  if (millis() - lastMsg > 5000 && lastMsg != 0) {
    Serial.println("\n>>> ALERTA: Sinal GPS perdido! <<<");
    lastMsg = millis();
  }
}

void sendUBX(uint8_t *msg, uint8_t len) {
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < len; i++) { ck_a += msg[i]; ck_b += ck_a; }
  SerialGPS.write(msg, len); SerialGPS.write(ck_a); SerialGPS.write(ck_b);
}

void processGPS(uint8_t mClass, uint8_t mId, uint8_t *ptr, uint16_t len) {
  if (mClass != 0x01) return;
  double lat = 0, lon = 0, altGeral = 0;
  float velH = 0, velV = 0;
  int ano = 0, mes = 0, dia = 0, hora = 0, min = 0, seg = 0, numSV = 0;
  bool fixValido = false;
  // --- EXTRAÇÃO VIA NAV-PVT (Mensagem mais completa) ---
  if (mId == 0x07 && len >= 92) {
    ano = ptr[4] | (ptr[5] << 8); mes = ptr[6]; dia = ptr[7];
    hora = ptr[8]; min = ptr[9]; seg = ptr[10];
    numSV = ptr[23];
    lon = ((int32_t)ptr[27] << 24 | (int32_t)ptr[26] << 16 | (int32_t)ptr[25] << 8 | ptr[24]) / 10000000.0;
    lat = ((int32_t)ptr[31] << 24 | (int32_t)ptr[30] << 16 | (int32_t)ptr[29] << 8 | ptr[28]) / 10000000.0;
    altGeral = ((int32_t)ptr[39] << 24 | (int32_t)ptr[38] << 16 | (int32_t)ptr[37] << 8 | ptr[36]) / 1000.0;
    // Velocidade Horizontal: mm/s para km/h
    int32_t gSpeed = ((int32_t)ptr[63] << 24 | (int32_t)ptr[62] << 16 | (int32_t)ptr[61] << 8 | ptr[60]);
    velH = (gSpeed / 1000.0) * 3.6;
    // Velocidade Vertical: mm/s para m/s (Invertido para subida ser positivo)
    int32_t vSpeed = ((int32_t)ptr[67] << 24 | (int32_t)ptr[66] << 16 | (int32_t)ptr[65] << 8 | ptr[64]);
    velV = (vSpeed / 1000.0) * -1.0;
    fixValido = (ptr[20] >= 3);
  }
  // --- EXTRAÇÃO VIA NAV-SOL (Backup/ECEF) ---
  else if (mId == 0x06 && len >= 52) {
    numSV = ptr[47];
    int16_t week = ptr[8] | (ptr[9] << 8);
    uint32_t iTOW = (uint32_t)ptr[0] | (uint32_t)ptr[1] << 8 | (uint32_t)ptr[2] << 16 | (uint32_t)ptr[3] << 24;
    int32_t fTOW = (int32_t)ptr[4] | (int32_t)ptr[5] << 8 | (int32_t)ptr[6] << 16 | (int32_t)ptr[7] << 24;
    unsigned long gpsSec = GPS_EPOCH_SECONDS + (week * 604800UL) + (iTOW / 1000UL);
    time_t t = gpsSec; struct tm *tm = gmtime(&t);
    ano = tm->tm_year + 1900; mes = tm->tm_mon + 1; dia = tm->tm_mday;
    hora = tm->tm_hour; min = tm->tm_min; seg = tm->tm_sec;
    // Correct byte offsets for NAV-SOL:
    // iTOW: 0-3, fTOW: 4-7, week: 8-9, gpsFix: 10, flags: 11, ecefX: 12-15, ecefY: 16-19, ecefZ: 20-23, pAcc: 24-27, ecefVX: 28-31, ecefVY: 32-35, ecefVZ: 36-39, sAcc: 40-43, pDOP: 44-45, reserved1: 46, numSV: 47, reserved2: 48-51
    int32_t ecefX = ((int32_t)ptr[15] << 24 | (int32_t)ptr[14] << 16 | (int32_t)ptr[13] << 8 | ptr[12]);
    int32_t ecefY = ((int32_t)ptr[19] << 24 | (int32_t)ptr[18] << 16 | (int32_t)ptr[17] << 8 | ptr[16]);
    int32_t ecefZ = ((int32_t)ptr[23] << 24 | (int32_t)ptr[22] << 16 | (int32_t)ptr[21] << 8 | ptr[20]);
    double X = ecefX / 100.0;
    double Y = ecefY / 100.0;
    double Z = ecefZ / 100.0;
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
    uint8_t gpsFix = ptr[10];
    fixValido = (gpsFix >= 3);

    // Now print the detailed NAV-SOL format
    Serial.println("### Organização Completa dos Dados da NAV-SOL com Raw e Conversões");

    // iTOW
    Serial.println("#### iTOW (Tempo da semana em milissegundos)");
    Serial.printf("- Raw: %u ms\n", iTOW);
    Serial.println("- Conversões:");
    Serial.printf("  - Segundos: %u s\n", iTOW / 1000);
    Serial.printf("  - Minutos: %u min\n", (iTOW / 1000) / 60);
    Serial.printf("  - Horas: %.2f h\n", (iTOW / 1000.0) / 3600.0);
    Serial.printf("  - Dias: %.2f dias\n", (iTOW / 1000.0) / 86400.0);
    Serial.println("- Descrição: Tempo desde o início da semana GPS (domingo 00:00), usado para calcular hora.");

    // fTOW
    Serial.println("#### fTOW (Fração de nanosegundos do iTOW)");
    Serial.printf("- Raw: %d ns\n", fTOW);
    Serial.println("- Conversões:");
    Serial.printf("  - Milissegundos: %.6f ms\n", fTOW / 1e6);
    Serial.printf("  - Segundos: %.9f s\n", fTOW / 1e9);
    Serial.printf("  - Absoluto (sem sinal): %u ns\n", abs(fTOW));
    Serial.println("- Descrição: Ajuste fino do tempo para precisão extra (geralmente correção).");

    // week
    Serial.println("#### week (Semana GPS)");
    Serial.printf("- Raw: %d semanas\n", week);
    Serial.println("- Conversões:");
    Serial.printf("  - Dias: %d dias\n", week * 7);
    Serial.printf("  - Anos: %.2f anos\n", week / 52.1429);
    Serial.printf("  - Data base (desde 1980): %04d-%02d-%02d\n", ano, mes, dia);
    Serial.println("- Descrição: Semanas desde 06/01/1980, usado para data completa.");

    // gpsFix
    Serial.println("#### gpsFix (Tipo de fix GPS)");
    Serial.printf("- Raw: %u\n", gpsFix);
    Serial.println("- Conversões:");
    const char* fixDesc[] = {"No Fix", "Dead Reckoning", "2D Fix", "3D Fix", "GPS + DR", "Time Only"};
    Serial.printf("  - Descrição: %s\n", (gpsFix <= 5) ? fixDesc[gpsFix] : "Unknown");
    Serial.printf("  - Binário: %08b\n", gpsFix);
    Serial.printf("  - Hex: 0x%02X\n", gpsFix);
    Serial.println("- Descrição: Status do sinal (3 = bom para uso).");

    // flags
    uint8_t flags = ptr[11];
    Serial.println("#### flags (Flags de status)");
    Serial.printf("- Raw: %08b (binário)\n", flags);
    Serial.println("- Conversões:");
    Serial.printf("  - Decimal: %u\n", flags);
    Serial.printf("  - Hex: 0x%02X\n", flags);
    Serial.print("  - Bits simples: ");
    if (flags & 0x01) Serial.print("GPS Fix OK, ");
    if (flags & 0x02) Serial.print("Diff Soln, ");
    if (flags & 0x04) Serial.print("WKNSET, ");
    if (flags & 0x08) Serial.print("TOWSET, ");
    Serial.println();
    Serial.println("- Descrição: Indicadores de qualidade dos dados (bits mostram se confiável).");

    // ecefX, Y, Z
    Serial.println("#### ecefX (Coordenada X ECEF)");
    Serial.printf("- Raw: %d cm\n", ecefX);
    Serial.println("- Conversões:");
    Serial.printf("  - Metros: %.2f m\n", ecefX / 100.0);
    Serial.printf("  - Quilômetros: %.2f km\n", ecefX / 100000.0);
    Serial.println("- Descrição: Posição X no sistema global (usado para Lat/Lon).");

    Serial.println("#### ecefY (Coordenada Y ECEF)");
    Serial.printf("- Raw: %d cm\n", ecefY);
    Serial.println("- Conversões:");
    Serial.printf("  - Metros: %.2f m\n", ecefY / 100.0);
    Serial.printf("  - Quilômetros: %.2f km\n", ecefY / 100000.0);
    Serial.println("- Descrição: Posição Y (usado para Lon).");

    Serial.println("#### ecefZ (Coordenada Z ECEF)");
    Serial.printf("- Raw: %d cm\n", ecefZ);
    Serial.println("- Conversões:");
    Serial.printf("  - Metros: %.2f m\n", ecefZ / 100.0);
    Serial.printf("  - Quilômetros: %.2f km\n", ecefZ / 100000.0);
    Serial.println("- Descrição: Posição Z (usado para Altitude).");

    // ECEF Convertido
    Serial.println("#### ECEF Convertido (Lat, Lon, Altitude) — Grupo derivado de X/Y/Z");
    Serial.println("- Raw: (Não é raw único, mas derivado)");
    Serial.println("- Conversões:");
    Serial.printf("  - Latitude: %.7f graus\n", lat);
    Serial.printf("  - Longitude: %.7f graus\n", lon);
    Serial.printf("  - Altitude elipsoidal: %.2f m (acima do elipsoide)\n", altGeral);
    Serial.printf("  - Altitude MSL (em relação ao solo/nível do mar): %.2f m (estimada com geóide)\n", altGeral - 30.0);  // Aprox geoid para BR
    Serial.println("- Descrição: Posição geográfica completa (conversão de ECEF para uso prático).");

    // pAcc
    uint32_t pAcc = ((uint32_t)ptr[27] << 24 | (uint32_t)ptr[26] << 16 | (uint32_t)ptr[25] << 8 | ptr[24]);
    Serial.println("#### pAcc (Precisão 3D estimada)");
    Serial.printf("- Raw: %u cm\n", pAcc);
    Serial.println("- Conversões:");
    Serial.printf("  - Metros: %.2f m\n", pAcc / 100.0);
    Serial.printf("  - Quilômetros: %.5f km\n", pAcc / 100000.0);
    Serial.println("- Descrição: Possível erro da posição (menor número = mais preciso).");

    // ecefVX, VY, VZ
    int32_t ecefVX = ((int32_t)ptr[31] << 24 | (int32_t)ptr[30] << 16 | (int32_t)ptr[29] << 8 | ptr[28]);
    Serial.println("#### ecefVX (Velocidade X ECEF)");
    Serial.printf("- Raw: %d cm/s\n", ecefVX);
    Serial.println("- Conversões:");
    Serial.printf("  - Metros/s: %.2f m/s\n", ecefVX / 100.0);
    Serial.printf("  - Km/h: %.3f km/h\n", (ecefVX / 100.0) * 3.6);
    Serial.println("- Descrição: Velocidade na direção X (negativa = sentido oposto).");

    int32_t ecefVY = ((int32_t)ptr[35] << 24 | (int32_t)ptr[34] << 16 | (int32_t)ptr[33] << 8 | ptr[32]);
    Serial.println("#### ecefVY (Velocidade Y ECEF)");
    Serial.printf("- Raw: %d cm/s\n", ecefVY);
    Serial.println("- Conversões:");
    Serial.printf("  - Metros/s: %.2f m/s\n", ecefVY / 100.0);
    Serial.printf("  - Km/h: %.3f km/h\n", (ecefVY / 100.0) * 3.6);
    Serial.println("- Descrição: Velocidade na direção Y.");

    int32_t ecefVZ = ((int32_t)ptr[39] << 24 | (int32_t)ptr[38] << 16 | (int32_t)ptr[37] << 8 | ptr[36]);
    Serial.println("#### ecefVZ (Velocidade Z ECEF)");
    Serial.printf("- Raw: %d cm/s\n", ecefVZ);
    Serial.println("- Conversões:");
    Serial.printf("  - Metros/s: %.2f m/s\n", ecefVZ / 100.0);
    Serial.printf("  - Km/h: %.3f km/h\n", (ecefVZ / 100.0) * 3.6);
    Serial.println("- Descrição: Velocidade na direção Z (positiva = subindo).");

    // sAcc
    uint32_t sAcc = ((uint32_t)ptr[43] << 24 | (uint32_t)ptr[42] << 16 | (uint32_t)ptr[41] << 8 | ptr[40]);
    Serial.println("#### sAcc (Precisão da velocidade)");
    Serial.printf("- Raw: %u cm/s\n", sAcc);
    Serial.println("- Conversões:");
    Serial.printf("  - Metros/s: %.2f m/s\n", sAcc / 100.0);
    Serial.printf("  - Km/h: %.2f km/h\n", (sAcc / 100.0) * 3.6);
    Serial.println("- Descrição:Possível erro de velocidade.");

    // pDOP
    uint16_t pDOP = ptr[44] | (ptr[45] << 8);
    Serial.println("#### pDOP (Diluição de precisão)");
    Serial.printf("- Raw: %u (escala 0.01)\n", pDOP);
    Serial.println("- Conversões:");
    Serial.printf("  - Valor real: %.2f\n", pDOP / 100.0);
    Serial.println("- Descrição: Qualidade dos satélites (2.45 = aceitável).");

    // reserved1
    uint8_t reserved1 = ptr[46];
    Serial.println("#### reserved1 (Reservado)");
    Serial.printf("- Raw: %u\n", reserved1);
    Serial.println("- Conversões:");
    Serial.printf("  - Binário: %08b\n", reserved1);
    Serial.printf("  - Hex: 0x%02X\n", reserved1);
    Serial.println("- Descrição: Interno, ignore.");

    // numSV
    Serial.println("#### numSV (Número de satélites)");
    Serial.printf("- Raw: %u\n", numSV);
    Serial.println("- Conversões:");
    Serial.printf("  - Descrição: %u sats (boa quantidade)\n", numSV);
    Serial.println("- Descrição: Sats usados para cálculo.");

    // reserved2
    uint32_t reserved2 = ((uint32_t)ptr[51] << 24 | (uint32_t)ptr[50] << 16 | (uint32_t)ptr[49] << 8 | ptr[48]);
    Serial.println("#### reserved2 (Reservado)");
    Serial.printf("- Raw: %u\n", reserved2);
    Serial.println("- Conversões:");
    Serial.printf("  - Binário: %032b\n", reserved2);
    Serial.printf("  - Hex: 0x%08X\n", reserved2);
    Serial.println("- Descrição: Interno, ignore.");

    lastMsg = millis();
  }
  if (fixValido) {
    // Ajuste Brasília (UTC-3)
    int horaBR = hora - 3;
    if (horaBR < 0) { horaBR += 24; dia--; }
    // Trava altitude do solo no primeiro fix estável
    if (altitudeInicial == -999.0 && numSV >= 6) altitudeInicial = altGeral;
    float altRelativa = (altitudeInicial == -999.0) ? 0.0 : (altGeral - altitudeInicial);
    lastMsg = millis();
  }
}