/*
============================================================================
CÓDIGO DO FOGUETE - VERSÃO COMPLETA E CORRIGIDA PARA ARDUINO
COM SENSOR BMP180 E DISPLAY OLED I2C

CORREÇÃO ADICIONADA:
    - Uso de dtostrf() para formatar floats para string, resolvendo o problema
      de valores não exibidos ("? M") no display OLED.

FUNCIONALIDADES PRINCIPAIS:
    - Leitura de altitude do sensor BMP180.
    - Exibição de dados (altitude, apogeu, status) em display OLED I2C.
    - Lógica de calibração inicial da altitude.
    - Detecção de lançamento e apogeu.
    - Lógica de recuperação (acionamento de MOSFETs) baseada em apogeu e descida.
    - Sinalização via buzzer e LED.
    - Detecção automática de endereços I2C de dispositivos (OLED e BMP180).
    - Tratamento de erros e travamento do sistema com feedback visual/sonoro.

CONVENÇÕES DE NOMECLATURA:
    - Funções: camelCase
    - Variáveis: snake_case
    - Constantes: SCREAMING_SNAKE_CASE

BIBLIOTECAS UTILIZADAS:
    - Wire.h: Comunicação I2C.
    - Adafruit_BMP085.h: Driver para sensor BMP180/BMP085.
    - Adafruit_GFX.h: Biblioteca gráfica base para displays Adafruit.
    - Adafruit_SSD1306.h: Driver específico para displays OLED SSD1306.
============================================================================
*/

// ==========================================================================
// INCLUSÃO DE BIBLIOTECAS
// ==========================================================================
#include <Wire.h>          // Essencial para comunicação I2C (usado por BMP180 e OLED)
#include <Adafruit_BMP085.h> // Para o sensor de pressão/temperatura/altitude BMP180
#include <Adafruit_GFX.h>  // Biblioteca gráfica base da Adafruit
#include <Adafruit_SSD1306.h> // Driver específico para displays OLED com chip SSD1306
#include <stdlib.h>        // Necessário para dtostrf()

// ==========================================================================
// DEFINIÇÕES DE PINOS (Constantes com SCREAMING_SNAKE_CASE)
// ==========================================================================
#define PINO_BUZZER 11            // Pino para o buzzer de sinalização
#define PINO_MOSFET_1 9           // Pino para o primeiro MOSFET (carga 1)
#define PINO_MOSFET_2 10          // Pino para o segundo MOSFET (carga 2)
#define LED_STATUS_PIN LED_BUILTIN // Pino do LED embutido do Arduino (geralmente pino 13)

// ==========================================================================
// LIMITES E CONFIGURAÇÕES DE VOO (Constantes com SCREAMING_SNAKE_CASE)
// ==========================================================================
const float LIMITE_SALTO_ANOMALO = 75.0;     // Variação de leitura de altitude para detectar erro no sensor
const float ALTITUDE_DE_LANCAMENTO = 0.5;    // Altitude mínima para considerar o foguete lançado (em metros)

// Constantes para a lógica de recuperação (ACIONAMENTO DO PARAQUEDAS/CARGA)
const float ALTITUDE_LIMITE_X = 100.0; // Exemplo: Considera "apogeu alto" se passar de 100 metros
const float DESCIDA_MINIMA_Y = 10.0;   // Exemplo: Descida mínima (10 metros) para acionar recuperação


// ==========================================================================
// CONFIGURAÇÕES DO DISPLAY OLED (Constantes e Objeto)
// ==========================================================================
// !!! ATENÇÃO: AJUSTE ESTES VALORES PARA O SEU DISPLAY OLED !!!
#define LARGURA_OLED 128 // Geralmente 128
#define ALTURA_OLED 64   // Pode ser 64 (comum) ou 32 (mais compacto)
#define OLED_RESET -1    // Pino de reset (use -1 para a maioria dos módulos I2C que não precisam de reset externo)

// Objeto do display OLED (inicializado sem endereço fixo, pois será detectado)
Adafruit_SSD1306 display(LARGURA_OLED, ALTURA_OLED, &Wire, OLED_RESET);
uint8_t oled_address = 0; // Variável para armazenar o endereço I2C do OLED encontrado

// ==========================================================================
// OBJETOS DE SENSORES (Globais)
// ==========================================================================
// Objeto do sensor BMP180 (a biblioteca gerencia o endereço 0x77 por padrão)
Adafruit_BMP085 bmp;
uint8_t bmp_address = 0; // Variável para armazenar o endereço I2C do BMP180 encontrado (deve ser 0x77)

// ==========================================================================
// PROTÓTIPOS DE FUNÇÕES (camelCase)
// Para que o compilador saiba que essas funções existem antes de serem definidas
// ==========================================================================
// Funções de controle do voo e sistema
void verificaAltitude(float altitude, float& apogeu_ref, bool& acionamento_iniciado_ref, bool& foguete_lancado_ref, void (*finalizar)(const __FlashStringHelper *));
void finalizarMissao(const __FlashStringHelper *razao_missao);
float lerAltitudeBmp180();

// Funções de utilidade e depuração
void travarSistema(const __FlashStringHelper *mensagem_serial, const __FlashStringHelper *mensagem_oled_linha1, const __FlashStringHelper *mensagem_oled_linha2);
void printHex(uint8_t num); // Função auxiliar para imprimir endereços hexadecimais


// ==========================================================================
// IMPLEMENTAÇÃO DA FUNÇÃO travarSistema()
// Mover esta função para antes do setup() corrige o erro de "undefined reference".
// ==========================================================================
void travarSistema(const __FlashStringHelper *mensagem_serial, const __FlashStringHelper *mensagem_oled_linha1, const __FlashStringHelper *mensagem_oled_linha2) {
  Serial.println(F("================================================="));
  Serial.print(F("ERRO CRÍTICO! "));
  Serial.println(mensagem_serial);
  Serial.println(F("Sistema parado. Verifique as conexões."));
  Serial.println(F("================================================="));

  // Tenta exibir no display se ele foi encontrado e inicializado com sucesso anteriormente
  if (oled_address != 0) { // Se o endereço do OLED foi encontrado, ele foi inicializado
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("ERRO CRITICO!"));
    display.setCursor(0, 10);
    display.print(mensagem_oled_linha1);
    display.setCursor(0, 20);
    display.print(mensagem_oled_linha2);
    display.display();
  } else {
    // Se o OLED NÃO foi encontrado/inicializado, tenta ligá-lo com um dos endereços comuns
    // APENAS para mostrar a mensagem de erro ANTES de travar.
    // Isso é uma tentativa de último recurso.
    // NOTE: Se o problema for na própria fiação SDA/SCL, mesmo isso pode falhar.
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C) || display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print(F("ERRO CRITICO!"));
        display.setCursor(0, 10);
        display.print(mensagem_oled_linha1);
        display.setCursor(0, 20);
        display.print(mensagem_oled_linha2);
        display.display();
    }
  }

  // Loop infinito com buzzer e LED piscando para indicar o erro grave
  while (true) {
    digitalWrite(PINO_BUZZER, HIGH);
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(500); // Pisca mais lentamente para indicar erro travado
    digitalWrite(PINO_BUZZER, LOW);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(500);
  }
}


// ==========================================================================
// FUNÇÃO setup() - Inicialização do Sistema
// ==========================================================================
void setup() {
  // Inicializa a comunicação serial para depuração e informações
  Serial.begin(115200);
  while (!Serial); // Aguarda a conexão serial em algumas placas (ex: ESP32, Teensy, ou se Serial Monitor não estiver aberto)

  // Configura os pinos digitais como saída
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_MOSFET_1, OUTPUT);
  pinMode(PINO_MOSFET_2, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);

  // Garante que os MOSFETs e o LED estão desligados no início
  digitalWrite(PINO_MOSFET_1, LOW);
  digitalWrite(PINO_MOSFET_2, LOW);
  digitalWrite(LED_STATUS_PIN, LOW);

  // Inicia a comunicação I2C (essencial para OLED e BMP180)
  Wire.begin();

  // ======================================================================
  // VARREDURA E IDENTIFICAÇÃO DOS DISPOSITIVOS I2C
  // ======================================================================
  Serial.println(F("\n================================================="));
  Serial.println(F(" INICIANDO VARREDURA I2C PARA DISPOSITIVOS..."));
  byte error;
  int device_count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission(); // Envia um byte de endereço para o barramento

    if (error == 0) { // Significa que um dispositivo respondeu (ACK recebido)
      Serial.print(F("Dispositivo I2C encontrado no endereco 0x"));
      printHex(address); // Função auxiliar para formatar a saída hexadecimal
      Serial.println();
      device_count++;

      // Tenta identificar qual componente é com base em endereços comuns
      if (address == 0x3C || address == 0x3D) {
        if (oled_address == 0) { // Armazena o endereço do OLED se ainda não foi encontrado
          oled_address = address;
          Serial.println(F("  -> Provavelmente o Display OLED."));
        } else {
          Serial.println(F("  -> Endereco comum de OLED, mas ja um OLED identificado."));
        }
      } else if (address == 0x77) {
        if (bmp_address == 0) { // Armazena o endereço do BMP180 se ainda não foi encontrado
          bmp_address = address;
          Serial.println(F("  -> Provavelmente o Sensor BMP180."));
        } else {
          Serial.println(F("  -> Endereco comum de BMP180, mas ja um BMP180 identificado."));
        }
      } else {
        Serial.println(F("  -> Dispositivo I2C desconhecido."));
      }
    } else if (error == 4) { // Outro tipo de erro (geralmente problema de hardware)
      Serial.print(F("Erro desconhecido (4) durante varredura no endereco 0x"));
      printHex(address);
      Serial.println();
    }
    delay(1); // Pequeno atraso para dar tempo ao barramento
  }

  Serial.print(F("Varredura I2C concluida. Total de dispositivos encontrados: "));
  Serial.println(device_count);
  Serial.println(F("================================================="));


  // ======================================================================
  // INICIALIZAÇÃO DO DISPLAY OLED
  // ======================================================================
  Serial.println(F("Iniciando display OLED..."));
  if (oled_address == 0) { // Se o scanner não encontrou o OLED
    travarSistema(F("Nenhum display OLED encontrado no barramento I2C."), F("OLED NAO ENCONTRADO"), F("Verifique conexoes."));
  }

  // Tenta inicializar o display OLED com o endereço encontrado
  // SSD1306_SWITCHCAPVCC é comum para a maioria dos módulos 0.96" e 1.3"
  if (!display.begin(SSD1306_SWITCHCAPVCC, oled_address)) {
    travarSistema(F("Falha ao inicializar o display OLED no endereco encontrado."), F("FALHA INIC. OLED"), F("Verifique modulo."));
  }

  // Configurações básicas do OLED
  display.clearDisplay();    // Limpa o buffer de memória do display
  display.setTextSize(1);    // Define o tamanho do texto (1 = menor, 2 = 2x maior, etc.)
  display.setTextColor(SSD1306_WHITE); // Define a cor do texto (pixels brancos/acesos)
  display.setCursor(0, 0);   // Define a posição inicial do cursor (coluna 0, linha 0)
  display.print(F("OLED OK!")); // Mensagem de sucesso
  display.display();         // Envia o conteúdo do buffer para o display físico
  delay(500); // Pequeno atraso para o usuário ver a mensagem


  // ======================================================================
  // MENU INICIAL (Serial e OLED)
  // ======================================================================
  Serial.println(F("\n================================================="));
  Serial.println(F(" COMPUTADOR DE VOO HABEMOS ROCKET v1.2"));
  Serial.println(F("================================================="));

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(F("Computador de Voo"));
  display.setCursor(0, 10); // Desloca para a próxima "linha" no OLED
  display.print(F("Habemos Rocket v1.2"));
  display.display();
  delay(2000); // Exibe por 2 segundos

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(F("Iniciando sensor"));
  display.display();
  Serial.println(F("Tentando inicializar BMP180..."));
  delay(1000); // Exibe por 1 segundo


  // Sinal sonoro de inicialização (buzzer)
  digitalWrite(PINO_BUZZER, HIGH);
  delay(300);
  digitalWrite(PINO_BUZZER, LOW);
  delay(300);

  // ======================================================================
  // INICIALIZAÇÃO DO SENSOR BMP180
  // ======================================================================
  // A biblioteca Adafruit_BMP085 já tenta inicializar no endereço 0x77 por padrão.
  // Não é necessário passar 'bmp_address' aqui, pois a biblioteca gerencia isso.
  if (!bmp.begin()) {
    travarSistema(F("Nao foi possivel encontrar BMP180 (0x77)."), F("ERRO SENSOR BMP"), F("Verifique conexoes."));
  }

  Serial.println(F("Sensor BMP180 encontrado com sucesso!"));
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(F("Sensor Iniciado"));
  display.setCursor(0, 10);
  display.print(F("Calibrando..."));
  display.display();
  Serial.println(F("Iniciando fase de calibracao de altitude..."));
  delay(1000); // Exibe por 1 segundo

  // Sinal sonoro para indicar o início da calibração
  digitalWrite(PINO_BUZZER, HIGH);
  delay(250);
  digitalWrite(PINO_BUZZER, LOW);
  delay(250);
}


// ==========================================================================
// FUNÇÃO loop() - Lógica Principal do Voo
// ==========================================================================
void loop() {
  // Variáveis de estado estáticas (mantêm seus valores entre chamadas do loop)
  static bool acionamento_iniciado = false;
  static float apogeu = 0; // Armazena a altitude máxima alcançada

  static float leitura_inicial_fixa = 0; // Altitude de referência após calibração
  static bool calibracao_inicial_concluida = false;

  static const int QUANTIDADE_LEITURAS_MEDIA = 5; // Número de leituras para a média móvel
  static float buffer_leituras[QUANTIDADE_LEITURAS_MEDIA] = {0}; // Buffer para média móvel
  static int indice_buffer = 0;
  static int contagem_leituras_pre_calibracao = 0;

  static float ultima_altitude_relativa = 0.0f;
  static bool primeira_medicao_pos_calibracao = true;
  static int contador_leituras_zero = 0; // Conta leituras nulas/zero consecutivas
  static bool foguete_lancado = false; // Flag para indicar se o foguete já foi lançado

  // Realiza a leitura da altitude atual do sensor BMP180
  float altitude_absoluta_atual = lerAltitudeBmp180();

  // --- TRATAMENTO DE LEITURAS ZERO/NULAS ---
  if (altitude_absoluta_atual == 0.0) { // O BMP180 pode retornar 0.0 em caso de falha de leitura
    contador_leituras_zero++;
    Serial.print(F("ALERTA: Leitura de sensor 0.0 detectada ("));
    Serial.print(contador_leituras_zero);
    Serial.println(F(")."));
    if (contador_leituras_zero >= 3) { // Se 3 leituras 0.0 consecutivas, finaliza
      finalizarMissao(F("TRES LEITURAS ZERADAS/NULAS CONSECUTIVAS"));
    }
    return; // Pula o resto do loop atual
  } else {
    contador_leituras_zero = 0; // Reseta o contador se a leitura for válida
  }

  // Verifica se a leitura do sensor retornou um valor inválido (NaN - Not a Number)
  if (isnan(altitude_absoluta_atual)) {
    Serial.println(F("ERRO: Leitura do sensor retornou NaN."));
    finalizarMissao(F("ERRO DE LEITURA DO SENSOR (NaN)"));
    return; // Pula o resto do loop atual
  }

  // Adiciona a leitura atual ao buffer da média móvel
  buffer_leituras[indice_buffer] = altitude_absoluta_atual;
  indice_buffer = (indice_buffer + 1) % QUANTIDADE_LEITURAS_MEDIA;


  // ======================================================================
  // LÓGICA DE CALIBRAÇÃO INICIAL DA ALTITUDE (antes do lançamento)
  // ======================================================================
  if (!calibracao_inicial_concluida) {
    // Exibe a leitura individual durante a calibração no Serial e OLED
    char oled_buffer[20]; // Buffer para formatar string para OLED
    
    // Usando dtostrf() para formatar a altitude
    dtostrf(altitude_absoluta_atual, 1, 2, oled_buffer); // 1 dígito mínimo, 2 casas decimais
    char calibration_msg[30];
    sprintf(calibration_msg, "Leitura %d: %sM", contagem_leituras_pre_calibracao + 1, oled_buffer);

    Serial.print(F("Calibrando... "));
    Serial.println(calibration_msg);

    display.clearDisplay(); // Limpa o buffer do OLED para o novo texto
    display.setCursor(0, 0);
    display.print(F("Calibrando..."));
    display.setCursor(0, 10);
    display.print(calibration_msg);
    display.display(); // Atualiza o OLED

    // Preenche o buffer de leituras
    if (contagem_leituras_pre_calibracao < QUANTIDADE_LEITURAS_MEDIA) {
      contagem_leituras_pre_calibracao++;
      delay(500);
    }

    // Quando tivermos leituras suficientes, calculamos a altitude de referência (zero)
    if (contagem_leituras_pre_calibracao == QUANTIDADE_LEITURAS_MEDIA) {
      float soma_buffer_calibracao = 0;
      for (int i = 0; i < QUANTIDADE_LEITURAS_MEDIA; i++) {
        soma_buffer_calibracao += buffer_leituras[i];
      }
      leitura_inicial_fixa = soma_buffer_calibracao / QUANTIDADE_LEITURAS_MEDIA;
      calibracao_inicial_concluida = true;

      Serial.println(F("\n================================================="));
      Serial.print(F("Calibracao concluida. Altitude de referencia (zero): "));
      Serial.print(leitura_inicial_fixa);
      Serial.println(F(" m."));
      Serial.println(F("AGUARDANDO LANCAMENTO..."));
      Serial.println(F("================================================="));

      // Mensagem de calibração concluída no OLED
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("Calibrado!"));
      display.setCursor(0, 10);
      
      dtostrf(leitura_inicial_fixa, 1, 2, oled_buffer);
      sprintf(calibration_msg, "Ref: %s M", oled_buffer);
      display.print(calibration_msg);
      display.display();
      
      delay(1500); // Deixa a mensagem de calibração visível

      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("AGUARDANDO..."));
      display.setCursor(0, 10);
      display.print(F("LANCA QDO >"));
      display.print(ALTITUDE_DE_LANCAMENTO); // float direto no print para este caso específico
      display.print(F("m"));
      display.display();
    }
    return; // Retorna para continuar a calibração nas próximas iterações do loop
  }

  // ======================================================================
  // LÓGICA APÓS A CALIBRAÇÃO E DURANTE O VOO
  // ======================================================================
  // Calcula a média móvel das últimas leituras
  float soma_buffer = 0;
  for (int i = 0; i < QUANTIDADE_LEITURAS_MEDIA; i++) {
    soma_buffer += buffer_leituras[i];
  }
  float nova_media_continua = soma_buffer / QUANTIDADE_LEITURAS_MEDIA;
  // Calcula a altitude relativa em relação ao ponto de calibração (zero)
  float altitude_relativa_media_movel = nova_media_continua - leitura_inicial_fixa;

  // --- DEPURAÇÃO SERIAL DA ALTITUDE ---
  Serial.print(F("Altitude = "));
  Serial.print(altitude_relativa_media_movel);
  Serial.println(F(" m"));
  
  // --- ATUALIZAÇÃO DO DISPLAY OLED COM ALTITUDE E APOGEU ---
  char oled_buffer_alt[10]; // Buffer para altitude, ajustado para dtostrf
  dtostrf(altitude_relativa_media_movel, 1, 2, oled_buffer_alt); // 1 dígito mínimo, 2 casas decimais

  char oled_buffer_apogeu[10]; // Buffer para apogeu, ajustado para dtostrf
  dtostrf(apogeu, 1, 2, oled_buffer_apogeu); // 1 dígito mínimo, 2 casas decimais

  display.clearDisplay(); // Sempre limpe o buffer antes de desenhar um novo "frame"
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Exibe a altitude atual
  display.setCursor(0, 0);
  display.print(F("Altitude:"));
  display.setTextSize(2); // Valor da altitude maior
  display.setCursor(0, 10);
  display.print(oled_buffer_alt);
  display.print(F(" M")); // Adiciona ' M' aqui
  
  // Exibe o apogeu
  display.setTextSize(1); // Volta ao tamanho normal para "Apogeu:"
  display.setCursor(0, 30);
  display.print(F("Apogeu:"));
  display.setTextSize(1); // Valor do apogeu no mesmo tamanho ou ligeiramente maior se quiser
  display.setCursor(0, 40);
  display.print(oled_buffer_apogeu);
  display.print(F(" M")); // Adiciona ' M' aqui
  
  display.display(); // ATUALIZA O DISPLAY FÍSICO COM TODO O CONTEÚDO DO BUFFER


  // ======================================================================
  // LÓGICA DE DETECÇÃO DE LANÇAMENTO
  // ======================================================================
  if (!foguete_lancado) {
    if (altitude_relativa_media_movel > ALTITUDE_DE_LANCAMENTO) {
      foguete_lancado = true;
      Serial.println(F(">>> LANCAMENTO DETECTADO! ARMANDO SISTEMA DE APOGEU. <<<"));
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(F("LANCTO DETECTADO"));
      display.setCursor(0,10);
      display.print(F("Armando Apogeu"));
      display.display();
    }
  }
  // ======================================================================
  // LÓGICA DE VOO APÓS O LANÇAMENTO
  // ======================================================================
  else {
    // Verificação de saltos anômalos de leitura (erro no sensor durante o voo)
    if (primeira_medicao_pos_calibracao) {
      ultima_altitude_relativa = altitude_relativa_media_movel;
      primeira_medicao_pos_calibracao = false;
    } else {
      float delta_leitura_bruta = abs(altitude_relativa_media_movel - ultima_altitude_relativa);
      if (delta_leitura_bruta >= LIMITE_SALTO_ANOMALO) {
        finalizarMissao(F("SALTO DE LEITURA ANOMALO"));
      }
      ultima_altitude_relativa = altitude_relativa_media_movel;
    }

    // Atualiza o apogeu (altitude máxima alcançada)
    if (altitude_relativa_media_movel > apogeu) {
      apogeu = altitude_relativa_media_movel;
      // O apogeu é atualizado no display automaticamente na próxima iteração do loop
    }

    // Verifica a altitude para acionamento do paraquedas/carga com a nova lógica
    verificaAltitude(altitude_relativa_media_movel, apogeu, acionamento_iniciado, foguete_lancado, finalizarMissao);
  }

  // Pequeno atraso para estabilizar a leitura e atualização do display
  delay(50); // Ajuste conforme a necessidade de suavidade do display e resposta do sistema
}

// ==========================================================================
// FUNÇÕES AUXILIARES
// ==========================================================================

/*
 * lerAltitudeBmp180() função (camelCase)
 * Retorna a altitude em metros lida do sensor BMP180.
 * Sinaliza uma leitura bem-sucedida com um breve pulso no LED de status.
 */
float lerAltitudeBmp180() {
  // 'extern Adafruit_BMP085 bmp;' não é estritamente necessário se 'bmp' é global,
  // mas é uma boa prática para indicar que a variável é definida em outro lugar.
  extern Adafruit_BMP085 bmp; 

  float altitude = bmp.readAltitude();

  // Aciona o LED brevemente para indicar uma leitura bem-sucedida
  digitalWrite(LED_STATUS_PIN, HIGH);
  delay(1); // Mantém o LED ligado por 1ms
  digitalWrite(LED_STATUS_PIN, LOW);

  return altitude;
}

/*
 * finalizarMissao() função (camelCase)
 * Interrompe o fluxo normal do programa, exibe a razão da finalização
 * e aciona os MOSFETs (se não for um erro crítico de inicialização de sensor).
 * Por fim, entra em um loop infinito piscando o buzzer e LED.
 */
void finalizarMissao(const __FlashStringHelper *razao_missao) {
  Serial.println(F("\n================================================="));
  Serial.print(F("FINALIZANDO MISSAO. MOTIVO: "));
  Serial.println(razao_missao);
  Serial.println(F("================================================="));

  // Tenta exibir a mensagem de finalização no display OLED, se ele estiver ativo
  if (oled_address != 0) { // Verifica se o OLED foi encontrado e inicializado
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print(F("MISSAO FINALIZADA"));
    display.setCursor(0, 10);
    display.print(razao_missao); // Exibe a razão da finalização no OLED
    display.display();
  }
  delay(2000); // Exibe por 2 segundos

  // Aciona as cargas (MOSFETs) APENAS se não for um erro de inicialização crítico
  // para evitar acionamentos acidentais no chão devido a sensor defeituoso.
  if (razao_missao != F("ERRO CRITICO NA INICIALIZACAO DO SENSOR")) {
    Serial.println(F("Acionando MOSFETs com PWM..."));
    if (oled_address != 0) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("ACIONANDO CARGAS"));
      display.display();
    }
    analogWrite(PINO_MOSFET_1, 255); // PWM máximo (acionamento total)
    analogWrite(PINO_MOSFET_2, 255);

    delay(3000); // Mantém as cargas ativadas por 3 segundos

    Serial.println(F("Desligando MOSFETs."));
    if (oled_address != 0) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print(F("Cargas Desligadas"));
      display.display();
    }
    analogWrite(PINO_MOSFET_1, 0); // Desliga os MOSFETs
    analogWrite(PINO_MOSFET_2, 0);
    delay(1000); // Pequeno delay para a mensagem ser visível
  }

  // Loop infinito com buzzer e LED piscando para indicar o fim da missão e travar o programa
  if (oled_address != 0) { // Se o OLED estiver funcionando, exibe mensagem final
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(F("Sistema Parado"));
    display.setCursor(0,10);
    display.print(F("Aguarde Remocao"));
    display.display();
  }

  while (true) { // Loop de travamento
    digitalWrite(PINO_BUZZER, HIGH);
    digitalWrite(LED_STATUS_PIN, HIGH);
    delay(300); // Buzzer e LED piscam a cada 300ms
    digitalWrite(PINO_BUZZER, LOW);
    digitalWrite(LED_STATUS_PIN, LOW);
    delay(300);
  }
}

/*
 * verificaAltitude() função (camelCase)
 * Verifica a altitude para decidir o acionamento do paraquedas/carga.
 * Implementa duas lógicas de recuperação:
 * 1. Foguete voou acima de ALTITUDE_LIMITE_X e desceu DESCIDA_MINIMA_Y.
 * 2. Foguete voou abaixo de ALTITUDE_LIMITE_X e desceu DESCIDA_MINIMA_Y,
 * após ter sido detectado o lançamento.
 */
void verificaAltitude(float altitude, float& apogeu_ref, bool& acionamento_iniciado_ref, bool& foguete_lancado_ref, void (*finalizar)(const __FlashStringHelper *)) {
  // A recuperação (acionamento da carga) só deve ser acionada uma única vez
  if (acionamento_iniciado_ref) {
    return;
  }

  // --- Lógica 1: Foguete voou alto e começou a descer ---
  // Se o apogeu foi maior que o limite alto (ex: 100m) E a altitude atual
  // caiu o suficiente em relação ao apogeu (ex: 10m de descida)
  if (apogeu_ref > ALTITUDE_LIMITE_X && altitude <= apogeu_ref - DESCIDA_MINIMA_Y) {
    acionamento_iniciado_ref = true; // Marca que o acionamento foi iniciado
    finalizar(F("RECUPERACAO: APOGEU ALTO E DESCIDA"));
    return;
  }

  // --- Lógica 2: Foguete voou baixo e começou a descer ---
  // Se o apogeu foi menor ou igual ao limite alto (ex: <= 100m) E o foguete
  // foi detectado como lançado E a altitude atual caiu o suficiente em relação ao apogeu
  if (apogeu_ref <= ALTITUDE_LIMITE_X && foguete_lancado_ref && altitude <= apogeu_ref - DESCIDA_MINIMA_Y) {
    acionamento_iniciado_ref = true; // Marca que o acionamento foi iniciado
    finalizar(F("RECUPERACAO: APOGEU BAIXO E DESCIDA"));
    return;
  }
}

/*
 * printHex() função (camelCase)
 * Função auxiliar para imprimir valores em formato hexadecimal (0xXX) no Serial Monitor.
 */
void printHex(uint8_t num) {
  String hex_string = String(num, HEX); // Converte para String hexadecimal
  hex_string.toUpperCase(); // Converte para maiúsculas (ex: 0A ao invés de 0a)
  if (hex_string.length() == 1) {
    Serial.print("0"); // Adiciona zero à esquerda se for apenas um dígito (ex: 0F ao invés de F)
  }
  Serial.print(hex_string);
}