/*
  TESTE ISOLADO: GPS de drone (u-blox M8 / Phantom 3) -> RP2040 Zero
  Arduino-Pico (Earle Philhower)

  GPS TX -> RP2040 GP1 (Serial1 RX / UART0)
  GPS RX -> RP2040 GP0 (Serial1 TX / UART0; usado para habilitar UBX)
  GPS GND -> RP2040 GND
  GPS VCC -> alimentacao adequada ao MODELO exato do modulo

  Monitor Serial USB: 115200
  GPS UART: 57600 8N1

  Sem LoRa, sem STM32, sem MATLAB. Nao modifica o firmware V3.7 salvo.
  O comando CFG-MSG habilita mensagens na porta atual, sem salvar configuracao permanente.
*/

#include <Arduino.h>
#include <math.h>
#include <cmath>
#include <time.h>

// ============================================================
// ALTERE SOMENTE ESTA LINHA PARA MUDAR A TAXA DO GPS:
// NEO-M8N com GPS+GLONASS: 1, 2, 3, 4 ou 5 Hz.
// 4 Hz = 250 ms entre novas posicoes (alvo do teste).
// ============================================================
#define GPS_FREQUENCIA_HZ 5

#if GPS_FREQUENCIA_HZ < 1 || GPS_FREQUENCIA_HZ > 5
  #error "GPS_FREQUENCIA_HZ deve ficar entre 1 e 5 para GPS+GLONASS no NEO-M8N"
#endif

static const uint16_t GPS_PERIODO_MS = (uint16_t)(1000U / GPS_FREQUENCIA_HZ);
static const uint32_t GPS_BAUD = 57600;
static const uint8_t GPS_PIN_RX = 1;
static const uint8_t GPS_PIN_TX = 0;
static const int FUSO_BR_H = -3;
// GPS e UTC diferem por segundos intercalares. Valor de referencia em 2026.
// Para horarios futuros, confirmar se ocorreu mudanca de leap seconds.
static const int GPS_MENOS_UTC_SEG = 18;

// Altitude RELATIVA: media das primeiras leituras com fix valido.
// Durante a calibracao, manter o GPS parado no ponto que sera o zero.
static const uint8_t AMOSTRAS_BASE_ALTITUDE = 10;

struct BaseAltitude {
  double soma = 0.0;
  double altitudeInicial = 0.0;
  uint8_t amostras = 0;
  bool pronta = false;
};

// NAV-PVT usa altitude MSL, NAV-SOL usa altitude elipsoidal.
// Referencias separadas impedem saltos ao trocar de tipo de mensagem.
static BaseAltitude basePVT;
static BaseAltitude baseSOL;

static void imprimirAltitudeRelativa(BaseAltitude &base, double altitudeAbsoluta,
                                    bool fixValido) {
  bool acabouDeCalibrar = false;
  if (!base.pronta && fixValido && isfinite(altitudeAbsoluta)) {
    base.soma += altitudeAbsoluta;
    ++base.amostras;
    if (base.amostras >= AMOSTRAS_BASE_ALTITUDE) {
      base.altitudeInicial = base.soma / (double)base.amostras;
      base.pronta = true;
      acabouDeCalibrar = true;
      Serial.printf("[GPS] Referencia de altitude definida: %.2f m (media de %u leituras).\n",
                    base.altitudeInicial, (unsigned)base.amostras);
    }
  }

  if (!base.pronta) {
    Serial.printf("ALTITUDE_GPS: 0.00 m (CALIBRANDO BASE: %u/%u leituras validas)\n",
                  (unsigned)base.amostras, (unsigned)AMOSTRAS_BASE_ALTITUDE);
    if (!fixValido)
      Serial.println("[GPS] Aguardando fix valido para definir a altitude inicial.");
    return;
  }

  if (!fixValido || !isfinite(altitudeAbsoluta)) {
    Serial.println("ALTITUDE_GPS: INDISPONIVEL (sem fix; zero inicial preservado)");
    return;
  }

  const double relativa = acabouDeCalibrar ? 0.0 : (altitudeAbsoluta - base.altitudeInicial);
  Serial.printf("ALTITUDE_GPS: %+.2f m (relativa ao ponto inicial)\n", relativa);
}

static const double WGS_A = 6378137.0;
static const double WGS_B = 6356752.314245;
static const double WGS_E2 = (WGS_A * WGS_A - WGS_B * WGS_B) / (WGS_A * WGS_A);
static const double WGS_EP2 = (WGS_A * WGS_A - WGS_B * WGS_B) / (WGS_B * WGS_B);
static const double RAD2DEG = 180.0 / 3.14159265358979323846;

static uint32_t bytesGPS = 0;
static uint32_t ubxValidos = 0;
static uint32_t ubxCRCerrados = 0;
static uint32_t pvtRecebidos = 0;
static uint32_t solRecebidos = 0;
static uint32_t ultimoByteMs = 0;
static uint32_t ultimoUBXMs = 0;
static uint32_t ultimoPVTMs = 0;
static uint32_t ultimoStatusMs = 0;
// Intervalos OBSERVADOS pelo GPS, calculados pelo iTOW de cada mensagem.
// Servem para conferir a taxa real: enviar CFG-RATE nao garante que foi aceita.
static uint32_t itowAnteriorPVT = 0xFFFFFFFFUL;
static uint32_t itowAnteriorSOL = 0xFFFFFFFFUL;
static uint16_t periodoObservadoPVT = 0;
static uint16_t periodoObservadoSOL = 0;

static void atualizarPeriodoGPS(uint32_t itow, uint32_t &anterior, uint16_t &periodo) {
  if (anterior != 0xFFFFFFFFUL) {
    // iTOW reinicia ao final da semana GPS.
    const uint32_t delta = itow >= anterior ? (itow - anterior)
                      : (604800000UL - anterior + itow);
    if (delta > 0 && delta <= 5000) periodo = (uint16_t)delta;
  }
  anterior = itow;
}

// Ler inteiros little-endian SEM deslocar inteiros assinados (evita comportamento indefinido).
static uint16_t u16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static uint32_t u32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t i32(const uint8_t *p) {
  return (int32_t)u32(p);
}

static bool anoBissexto(int ano) {
  return (ano % 4 == 0 && (ano % 100 != 0 || ano % 400 == 0));
}
static int diasMes(int mes, int ano) {
  static const uint8_t dias[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (mes < 1 || mes > 12) return 30;
  if (mes == 2 && anoBissexto(ano)) return 29;
  return dias[mes - 1];
}

static void imprimirUTCmenos3(int ano, int mes, int dia, int hora, int minuto, int segundo) {
  hora += FUSO_BR_H;
  if (hora < 0) {
    hora += 24;
    dia--;
    if (dia < 1) {
      mes--;
      if (mes < 1) {
        mes = 12;
        ano--;
      }
      dia = diasMes(mes, ano);
    }
  }
  Serial.printf("Hora UTC-3: %02d/%02d/%04d %02d:%02d:%02d\n",
                dia, mes, ano, hora, minuto, segundo);
}

static void envioUBX(const uint8_t *msgSemChecksum, size_t tamanho) {
  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < tamanho; ++i) {
    ckA = (uint8_t)(ckA + msgSemChecksum[i]);
    ckB = (uint8_t)(ckB + ckA);
  }
  Serial1.write(msgSemChecksum, tamanho);
  Serial1.write(ckA);
  Serial1.write(ckB);
  Serial1.flush();
}

// Impressao compacta: so campos uteis para o sistema de telemetria.
// Velocidade vertical: positiva = subida, negativa = descida.
// Modulo da velocidade: magnitude 3D sqrt(Vn^2 + Ve^2 + Vd^2).
static void imprimirPVT(const uint8_t *p, uint16_t len) {
  if (len < 92) return;
  ++pvtRecebidos;
  ultimoPVTMs = millis();
  atualizarPeriodoGPS(u32(p), itowAnteriorPVT, periodoObservadoPVT);

  const uint8_t fixType = p[20];
  const uint8_t flags = p[21];
  const uint8_t sat = p[23];
  const bool fixValido = (fixType == 3 || fixType == 4) && ((flags & 0x01) != 0);
  const bool tempoValido = (p[11] & 0x03) == 0x03;

  // NAV-PVT: coordenadas em graus x 1e-7, altitudes em mm,
  // velocidade N/E/D em mm/s (D e positiva para BAIXO).
  const double lon = i32(p + 24) * 1e-7;
  const double lat = i32(p + 28) * 1e-7;
  const double altMSL = i32(p + 36) / 1000.0;
  const double vN = i32(p + 48) / 1000.0;
  const double vE = i32(p + 52) / 1000.0;
  const double vD = i32(p + 56) / 1000.0;
  const double vHorizontal = hypot(vN, vE);
  const double vVertical = -vD;
  const double vTotal = sqrt(vN * vN + vE * vE + vD * vD);
  const double precisao = u32(p + 40) / 1000.0; // hAcc horizontal

  Serial.println("\n========= GPS RP2040 | NAV-PVT =========");
  Serial.printf("GPS_VALIDO: %s | Fix: %u\n", fixValido ? "SIM" : "NAO", fixType);
  Serial.printf("SATELITES: %u\n", sat);
  if (tempoValido)
    imprimirUTCmenos3(u16(p + 4), p[6], p[7], p[8], p[9], p[10]);
  else
    Serial.println("DATA_HORA: aguardando sincronismo");
  Serial.printf("LATITUDE: %.7f graus\n", lat);
  Serial.printf("LONGITUDE: %.7f graus\n", lon);
  imprimirAltitudeRelativa(basePVT, altMSL, fixValido);
  Serial.printf("VELOCIDADE_HORIZONTAL: %.2f m/s | %.2f km/h\n", vHorizontal, vHorizontal * 3.6);
  Serial.printf("VELOCIDADE_VERTICAL: %.2f m/s\n", vVertical);
  Serial.printf("MODULO_VELOCIDADE_3D: %.2f m/s | %.2f km/h\n", vTotal, vTotal * 3.6);
  Serial.printf("PRECISAO_GPS: %.2f m (horizontal estimada)\n", precisao);
  if (!fixValido)
    Serial.println("ATENCAO: POSICAO E VELOCIDADES SAO ESTIMATIVAS SEM FIX VALIDO");
}

static void imprimirSOL(const uint8_t *p, uint16_t len) {
  if (len < 52) return;
  ++solRecebidos;
  atualizarPeriodoGPS(u32(p), itowAnteriorSOL, periodoObservadoSOL);
  // Prefere NAV-PVT quando ele existe, evitando imprimir duas vezes a mesma epoca.
  if (ultimoPVTMs != 0 && millis() - ultimoPVTMs < 3500) return;

  const uint32_t itow = u32(p + 0);
  const int16_t semana = (int16_t)u16(p + 8);
  const uint8_t fixType = p[10];
  const uint8_t flags = p[11];
  const uint8_t sat = p[47];
  const bool fixValido = (fixType == 3 || fixType == 4) && ((flags & 0x01) != 0);

  // NAV-SOL: ECEF XYZ em cm; velocidade ECEF Vx,Vy,Vz em cm/s.
  const double x = i32(p + 12) / 100.0;
  const double y = i32(p + 16) / 100.0;
  const double z = i32(p + 20) / 100.0;
  const double vx = i32(p + 28) / 100.0;
  const double vy = i32(p + 32) / 100.0;
  const double vz = i32(p + 36) / 100.0;
  const double precisao = u32(p + 24) / 100.0; // pAcc: 3D, em cm

  // WGS84, ECEF -> latitude, longitude, altitude elipsoidal.
  const double rho = hypot(x, y);
  if (rho < 1.0) return;
  const double theta = atan2(z * WGS_A, rho * WGS_B);
  const double st = sin(theta), ct = cos(theta);
  const double latRad = atan2(z + WGS_EP2 * WGS_B * st * st * st,
                              rho - WGS_E2 * WGS_A * ct * ct * ct);
  const double lonRad = atan2(y, x);
  const double sinLat = sin(latRad), cosLat = cos(latRad);
  const double sinLon = sin(lonRad), cosLon = cos(lonRad);
  const double N = WGS_A / sqrt(1.0 - WGS_E2 * sinLat * sinLat);
  const double altElipsoidal = rho / cosLat - N;

  // ECEF -> velocidades locais Leste, Norte e Cima (ENU).
  const double vLeste = -sinLon * vx + cosLon * vy;
  const double vNorte = -sinLat * cosLon * vx - sinLat * sinLon * vy + cosLat * vz;
  const double vVertical = cosLat * cosLon * vx + cosLat * sinLon * vy + sinLat * vz;
  const double vHorizontal = hypot(vLeste, vNorte);
  const double vTotal = sqrt(vx * vx + vy * vy + vz * vz);

  Serial.println("\n========= GPS RP2040 | NAV-SOL =========");
  Serial.printf("GPS_VALIDO: %s | Fix: %u | Flags: 0x%02X\n",
                fixValido ? "SIM" : "NAO", fixType, flags);
  Serial.printf("SATELITES: %u\n", sat);
  // A semana GPS se refere a GPS Time: UTC = GPS Time - leap seconds.
  // Em 2026 o offset e 18 s; preferir NAV-PVT quando disponivel.
  if (semana >= 2000 && semana <= 3000 && (flags & 0x0C) == 0x0C) {
    const int64_t segundosBR = 315964800LL + (int64_t)semana * 604800LL +
        itow / 1000 - GPS_MENOS_UTC_SEG + (int64_t)FUSO_BR_H * 3600LL;
    time_t t = (time_t)segundosBR;
    struct tm *br = gmtime(&t);
    if (br != nullptr) {
      Serial.printf("Hora UTC-3: %02d/%02d/%04d %02d:%02d:%02d\n",
                    br->tm_mday, br->tm_mon + 1, br->tm_year + 1900,
                    br->tm_hour, br->tm_min, br->tm_sec);
    }
  } else {
    Serial.println("DATA_HORA: aguardando semana/tempo GPS valido");
  }

  Serial.printf("LATITUDE: %.7f graus\n", latRad * RAD2DEG);
  Serial.printf("LONGITUDE: %.7f graus\n", lonRad * RAD2DEG);
  imprimirAltitudeRelativa(baseSOL, altElipsoidal, fixValido);
  Serial.printf("VELOCIDADE_HORIZONTAL: %.2f m/s | %.2f km/h\n", vHorizontal, vHorizontal * 3.6);
  Serial.printf("VELOCIDADE_VERTICAL: %.2f m/s\n", vVertical);
  Serial.printf("MODULO_VELOCIDADE_3D: %.2f m/s | %.2f km/h\n", vTotal, vTotal * 3.6);
  Serial.printf("PRECISAO_GPS: %.2f m (3D estimada)\n", precisao);
  if (!fixValido)
    Serial.println("ATENCAO: POSICAO E VELOCIDADES SAO ESTIMATIVAS SEM FIX VALIDO");
}

static void tratarPacote(uint8_t msgClasse, uint8_t msgId, const uint8_t *payload, uint16_t len) {
  if (msgClasse == 0x01 && msgId == 0x07) imprimirPVT(payload, len);
  else if (msgClasse == 0x01 && msgId == 0x06) imprimirSOL(payload, len);
  else if (msgClasse == 0x05 && len >= 2 && payload[0] == 0x06 && payload[1] == 0x01) {
    Serial.println(msgId == 0x01 ? "[GPS] CFG-MSG aceito" : "[GPS] CFG-MSG recusado");
  }
  else if (msgClasse == 0x05 && len >= 2 && payload[0] == 0x06 && payload[1] == 0x08) {
    Serial.println(msgId == 0x01 ? "[GPS] CFG-RATE ACEITO pelo receptor"
                                 : "[GPS] CFG-RATE RECUSADO pelo receptor");
  }
  else if (msgClasse == 0x06 && msgId == 0x08 && len >= 6) {
    const uint16_t periodo = u16(payload);
    const uint16_t ciclos = u16(payload + 2);
    Serial.printf("[GPS] CFG-RATE LIDO: measRate=%u ms navRate=%u -> %.2f Hz configurados\n",
                  (unsigned)periodo, (unsigned)ciclos,
                  (periodo && ciclos) ? (1000.0 / (periodo * (double)ciclos)) : 0.0);
  }
}

// Parser UBX com leitura de CK_A e CK_B, verificacao de integridade e limite de buffer.
struct ParserUBX {
  uint8_t etapa = 0;
  uint8_t classe = 0, id = 0, ckA = 0, ckB = 0;
  uint8_t payload[128];
  uint16_t tam = 0, pos = 0;

  void somar(uint8_t c) {
    ckA = (uint8_t)(ckA + c);
    ckB = (uint8_t)(ckB + ckA);
  }

  void consumir(uint8_t c) {
    switch (etapa) {
      case 0: etapa = (c == 0xB5) ? 1 : 0; break;
      case 1:
        if (c == 0x62) { etapa = 2; ckA = 0; ckB = 0; }
        else etapa = (c == 0xB5) ? 1 : 0;
        break;
      case 2: classe = c; somar(c); etapa = 3; break;
      case 3: id = c; somar(c); etapa = 4; break;
      case 4: tam = c; somar(c); etapa = 5; break;
      case 5:
        tam |= ((uint16_t)c << 8); somar(c); pos = 0;
        etapa = (tam == 0) ? 7 : 6;
        break;
      case 6:
        if (pos < sizeof(payload)) payload[pos] = c;
        somar(c);
        ++pos;
        if (pos >= tam) etapa = 7;
        break;
      case 7:
        etapa = (c == ckA) ? 8 : 9;
        break;
      case 8:
        if (c == ckB) {
          ++ubxValidos;
          ultimoUBXMs = millis();
          if (tam <= sizeof(payload)) tratarPacote(classe, id, payload, tam);
        } else ++ubxCRCerrados;
        etapa = 0;
        break;
      case 9:
        ++ubxCRCerrados;
        etapa = 0;
        break;
      default: etapa = 0; break;
    }
  }
};

static ParserUBX parser;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n================================================");
  Serial.println(" GPS PHANTOM 3 / UBLOX M8 -> RP2040 ZERO | ALTITUDE ZERO INICIAL");
  Serial.println(" UART GPS: GP1 RX, GP0 TX, 57600 baud");
  Serial.printf(" FREQUENCIA SOLICITADA AO GPS: %u Hz (%u ms)\n",
                (unsigned)GPS_FREQUENCIA_HZ, (unsigned)GPS_PERIODO_MS);
  Serial.println(" Monitor USB: 115200 baud");
  Serial.println(" PRINT: LAT LON ALT VH VV VELOCIDADE_3D DATA HORA SAT PRECISAO FIX");
  Serial.println(" ALTITUDE: media das primeiras 10 leituras COM FIX -> 0.00 m");
  Serial.println(" TESTE ISOLADO - SEM LORA / SEM STM32");
  Serial.println("================================================");

  Serial1.setRX(GPS_PIN_RX);
  Serial1.setTX(GPS_PIN_TX);
  Serial1.setFIFOSize(256);
  Serial1.begin(GPS_BAUD);
  delay(250);

  // UBX-CFG-RATE (classe 0x06 / id 0x08): altera a TAXA REAL de
  // navegacao/medicao do receptor, em vez de apenas limitar prints.
  // measRate=GPS_PERIODO_MS; navRate=1; timeRef=GPS (1).
  uint8_t configurarRate[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    (uint8_t)(GPS_PERIODO_MS & 0xFF),
    (uint8_t)(GPS_PERIODO_MS >> 8),
    0x01, 0x00, 0x01, 0x00
  };
  envioUBX(configurarRate, sizeof(configurarRate));
  Serial.printf("[GPS] Enviando CFG-RATE: %u Hz / %u ms\n",
                (unsigned)GPS_FREQUENCIA_HZ, (unsigned)GPS_PERIODO_MS);
  delay(120);

  // Mesmas mensagens UBX do codigo anterior. Taxa de mensagem=1 significa
  // um NAV-PVT/SOL por solucao, nao 1 Hz. A frequencia e dada por CFG-RATE.
  // Sao pedidos VOLATEIS; nao salvam configuracao permanente no modulo GPS.
  const uint8_t habilitarPVT[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01};
  const uint8_t habilitarSOL[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01};
  envioUBX(habilitarPVT, sizeof(habilitarPVT));
  delay(100);
  envioUBX(habilitarSOL, sizeof(habilitarSOL));
  delay(120);

  // Consulta a configuracao ATIVA, permitindo conferir o retorno do GPS.
  const uint8_t consultarRate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00};
  envioUBX(consultarRate, sizeof(consultarRate));
  Serial.println("[GPS] Solicitei NAV-PVT/NAV-SOL e consultei CFG-RATE.");
}

void loop() {
  while (Serial1.available() > 0) {
    const uint8_t c = (uint8_t)Serial1.read();
    ++bytesGPS;
    ultimoByteMs = millis();
    parser.consumir(c);
  }

  const uint32_t agora = millis();
  if (agora - ultimoStatusMs >= 5000) {
    ultimoStatusMs = agora;
    Serial.printf("\n[GPS] Bytes=%lu UBX_OK=%lu CRC_ERR=%lu PVT=%lu SOL=%lu\n",
      (unsigned long)bytesGPS, (unsigned long)ubxValidos, (unsigned long)ubxCRCerrados,
      (unsigned long)pvtRecebidos, (unsigned long)solRecebidos);
    if (periodoObservadoPVT)
      Serial.printf("[GPS] TAXA REAL NAV-PVT: %u ms / %.2f Hz\n",
                    (unsigned)periodoObservadoPVT, 1000.0 / periodoObservadoPVT);
    if (periodoObservadoSOL)
      Serial.printf("[GPS] TAXA REAL NAV-SOL: %u ms / %.2f Hz\n",
                    (unsigned)periodoObservadoSOL, 1000.0 / periodoObservadoSOL);
    if (!periodoObservadoPVT && !periodoObservadoSOL)
      Serial.println("[GPS] TAXA REAL: aguardando pelo menos 2 mensagens de navegacao.");

    if (bytesGPS == 0 || (ultimoByteMs != 0 && agora - ultimoByteMs > 5000)) {
      Serial.println("[ALERTA] Nenhum byte recente do GPS: conferir VCC/GND, TX->GP1 e baud 57600.");
    } else if (ubxValidos == 0 || (ultimoUBXMs != 0 && agora - ultimoUBXMs > 5000)) {
      Serial.println("[ALERTA] Bytes chegando, mas sem UBX recente. Verificar protocolo, baud e RX do GPS no GP0.");
    } else if (pvtRecebidos == 0 && solRecebidos == 0) {
      Serial.println("[ALERTA] UBX chega, mas NAV-PVT e NAV-SOL nao estao habilitados.");
    }
  }
}
