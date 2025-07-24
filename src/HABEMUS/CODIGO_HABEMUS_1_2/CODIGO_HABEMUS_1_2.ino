/*
============================================================================
CÓDIGO DO FOGUETE - VERSÃO PARA ARDUINO COM SENSOR BMP280

MODIFICADO: O objeto Adafruit_BMP280 'bmp' foi movido de volta para o escopo global
           para ser acessível por setup() e loop(), resolvendo o erro de
           "undefined reference".
           As outras variáveis de estado permanecem locais estáticas em loop().

1. Atualização do Sensor:
   - Substituída a biblioteca Adafruit_BMP085 pela Adafruit_BMP280.
   - Alteradas as chamadas de função para corresponderem à API do BMP280.
   - Ajustada a inicialização do sensor e a leitura da altitude.
2. Formatação e Clareza:
   - Identação padronizada para melhor legibilidade.
   - Adicionadas chaves `{}` a todos os blocos `if`, `else` e loops para clareza e segurança.
   - Comentários adicionados/restaurados no meio do código para maior clareza.
3. Otimização de Velocidade:
   - Implementado o **Modo Forçado** do BMP280 para obter a maior taxa de leitura possível.
   - Removidos delays desnecessários no loop principal e na calibração para execução contínua.
   - MANTIDOS os delays propositais para o acionamento dos MOSFETs e do buzzer.
   - ADICIONADO: LED do Arduino pisca uma vez a cada leitura de altitude.
4. Correção de Erro de Compilação:
   - Alterado 'STANDBY_MS_0_5' para 'STANDBY_MS_1' para corrigir erro de compilação da biblioteca Adafruit_BMP280.
5. NOVA LÓGICA DE RECUPERAÇÃO:
   - Implementadas as regras:
     - Foguete voou acima de Xm (100m) e desceu Ym (10m), acionar a recuperação.
     - Foguete voou abaixo de Xm (100m) e desceu Ym (10m), acionar a recuperação (recuperação de falha).
6. NOVA SAÍDA DE CALIBRAÇÃO:
   - Mostra cada uma das 5 leituras de pré-calibração antes de calcular a média.
7. **NOVO MENU INICIAL:**
   - Adicionado um cabeçalho "Computador de Voo Habemos Rocket v1.2" na inicialização.
============================================================================
REMODELADO COM BASE NA ANÁLISE:
    - Convenção de Nomenclatura: Funções camelCase, Variáveis snake_case, Constantes SCREAMING_SNAKE_CASE.
    - loop(): Linhas de buffer movidas. Tratamento de leitura zero com 'return;'.
    - lerAltitudeBmp280(): Remoção de 'extern' redundante.
    - Manutenção de todas as funcionalidades anteriores (LED, lógica de recuperação, etc.).
============================================================================
*/

#include <Wire.h>
#include <Adafruit_BMP280.h> // Biblioteca correta para o BMP280

/* Definições de pinos (SCREAMING_SNAKE_CASE) */
#define PINO_BUZZER 11
#define PINO_MOSFET_1 9
#define PINO_MOSFET_2 10
#define LED_STATUS_PIN LED_BUILTIN // Pino do LED embutido do Arduino (geralmente pino 13)

/* Limites e configurações de voo (SCREAMING_SNAKE_CASE) */
const float LIMITE_SALTO_ANOMALO = 75.0;
const float ALTITUDE_DE_LANCAMENTO = 30.0;

// Constantes para a lógica de recuperação (SCREAMING_SNAKE_CASE)
const float ALTITUDE_LIMITE_X = 100.0; // Altura de corte para a lógica de recuperação (X metros)
const float DESCIDA_MINIMA_Y = 10.0;    // Descida mínima para acionar a recuperação (Y metros)


// Objeto do sensor: DECLARADO GLOBALMENTE para ser acessível em setup() e loop().
Adafruit_BMP280 bmp;

// PROTÓTIPOS DAS FUNÇÕES (camelCase)
void verificaAltitude(float altitude, float& apogeu_ref, bool& acionamento_iniciado_ref, bool& foguete_lancado_ref, void (*finalizar)(const __FlashStringHelper *));
void finalizarMissao(const __FlashStringHelper *razao_missao);
float lerAltitudeBmp280(); // Renomeada para camelCase


// setup() função (camelCase)
void setup() {
  Serial.begin(115200);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_MOSFET_1, OUTPUT);
  pinMode(PINO_MOSFET_2, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT); 
  digitalWrite(PINO_MOSFET_1, LOW);
  digitalWrite(PINO_MOSFET_2, LOW);
  digitalWrite(LED_STATUS_PIN, LOW); 

  // --- MENU INICIAL ---
  Serial.println(F("================================================="));
  Serial.println(F(" COMPUTADOR DE VOO HABEMOS ROCKET v1.2"));
  Serial.println(F("================================================="));
  delay(1000); // Pequeno delay para a mensagem ser visível antes do próximo log
  // --- FIM DO MENU INICIAL ---

  // Sinal sonoro de inicialização
  digitalWrite(PINO_BUZZER, HIGH);
  delay(300);
  digitalWrite(PINO_BUZZER, LOW);
  delay(300);
  Wire.begin(); // Inicia a comunicação I2C

  Serial.println(F("Tentando inicializar BMP280..."));
  if (!bmp.begin()) {
    Serial.println(F("ERRO CRÍTICO: Nao foi possivel encontrar um sensor BMP280 valido. Verifique as conexoes e o endereco I2C."));
    finalizarMissao(F("ERRO CRITICO NA INICIALIZACAO DO SENSOR"));
  }
  // Configurações para o BMP280 visando velocidade em vez de economia de energia/precisão de temperatura
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     // Modo Forçado para controle manual das leituras
                  Adafruit_BMP280::SAMPLING_X1,     // Oversampling de Temperatura x1 (menos relevante para altitude)
                  Adafruit_BMP280::SAMPLING_X16,    // Oversampling de Pressão x16 (boa precisão de pressão)
                  Adafruit_BMP280::FILTER_OFF,      // Filtragem IIR desligado para resposta mais rápida
                  Adafruit_BMP280::STANDBY_MS_1);   // Tempo de espera de 1 ms (irrelevante no modo forçado, mas precisa de um valor válido)

  Serial.println(F("Sensor BMP280 encontrado com sucesso!"));

  Serial.println(F("Iniciando fase de calibracao de altitude..."));

  // Sinal sonoro para indicar o início da calibração
  digitalWrite(PINO_BUZZER, HIGH);
  delay(250);
  digitalWrite(PINO_BUZZER, LOW);
  delay(250);
}


// loop() função (camelCase)
void loop() {
  // Variáveis de estado, declaradas como 'static' locais dentro de loop().
  // Elas são inicializadas apenas uma vez na primeira chamada de loop().
  static bool acionamento_iniciado = false;
  static float apogeu = 0;

  static float leitura_inicial_fixa = 0;
  static bool calibracao_inicial_concluida = false;

  static const int QUANTIDADE_LEITURAS_MEDIA = 5; // Constante local estática (SCREAMING_SNAKE_CASE)
  static float buffer_leituras[QUANTIDADE_LEITURAS_MEDIA] = {0}; // Inicializa o array com zeros
  static int indice_buffer = 0;
  static int contagem_leituras_pre_calibracao = 0;

  static float ultima_altitude_relativa = 0.0f;
  static bool primeira_medicao_pos_calibracao = true;
  static int contador_leituras_zero = 0;
  static bool foguete_lancado = false; // Estado para saber se o foguete decolou

  // Realiza a leitura da altitude atual do sensor BMP280
  float altitude_absoluta_atual = lerAltitudeBmp280(); // Renomeada

  // TRATAMENTO DE LEITURAS ZERO: Se a leitura for 0.0, incrementa o contador e retorna imediatamente.
  // Isso evita que leituras inválidas de 0.0 entrem no buffer ou afetem os cálculos.
  if (altitude_absoluta_atual == 0.0) {
    contador_leituras_zero++;
    // Se 3 leituras zeradas consecutivas, algo está errado
    if (contador_leituras_zero >= 3) {
      finalizarMissao(F("TRES LEITURAS ZERADAS/NULAS CONSECUTIVAS"));
    }
    return; // Retorna para não processar a leitura inválida
  } else {
    contador_leituras_zero = 0; // Reseta o contador se a leitura for válida
  }

  // Verifica se a leitura do sensor retornou um valor inválido (NaN)
  if (isnan(altitude_absoluta_atual)) {
    finalizarMissao(F("ERRO DE LEITURA DO SENSOR (NaN)"));
  }

  // LINHAS MOVIDAS: Adiciona a leitura atual ao buffer e avança o índice
  // Estas linhas agora ocorrem uma única vez no início do loop,
  // antes da lógica de calibração ou de voo.
  buffer_leituras[indice_buffer] = altitude_absoluta_atual;
  indice_buffer = (indice_buffer + 1) % QUANTIDADE_LEITURAS_MEDIA;


  // Lógica de calibração inicial da altitude (antes do lançamento)
  if (!calibracao_inicial_concluida) {
    // Exibe a leitura individual durante a calibração
    Serial.print(F("Calibrando... Leitura "));
    Serial.print(contagem_leituras_pre_calibracao + 1); // +1 para exibir de 1 a 5
    Serial.print(F(": "));
    Serial.print(altitude_absoluta_atual);
    Serial.println(F(" m"));

    // O buffer e indice já foram atualizados acima.
    if (contagem_leituras_pre_calibracao < QUANTIDADE_LEITURAS_MEDIA) {
      contagem_leituras_pre_calibracao++;
    }
    // Quando tivermos leituras suficientes para a média, calculamos a altitude de referência
    if (contagem_leituras_pre_calibracao == QUANTIDADE_LEITURAS_MEDIA) {
      float soma_buffer_calibracao = 0;
      for (int i = 0; i < QUANTIDADE_LEITURAS_MEDIA; i++) {
        soma_buffer_calibracao += buffer_leituras[i];
      }
      leitura_inicial_fixa = soma_buffer_calibracao / QUANTIDADE_LEITURAS_MEDIA;
      calibracao_inicial_concluida = true;
      Serial.println(F("================================================="));
      Serial.print(F("Calibracao concluida. Altitude de referencia (zero): "));
      Serial.print(leitura_inicial_fixa);
      Serial.println(F(" m."));
      Serial.println(F("AGUARDANDO LANCAMENTO..."));
      Serial.println(F("================================================="));
    }
    return; // Retorna para continuar a calibração nas próximas iterações
  }

  // --- Lógica após a calibração e durante o voo ---
  // O buffer já foi atualizado no início do loop.

  // Calcula a média móvel das últimas leituras
  float soma_buffer = 0;
  for (int i = 0; i < QUANTIDADE_LEITURAS_MEDIA; i++) {
    soma_buffer += buffer_leituras[i];
  }
  float nova_media_continua = soma_buffer / QUANTIDADE_LEITURAS_MEDIA;
  // Calcula a altitude relativa em relação ao ponto de calibração (zero)
  float altitude_relativa_media_movel = nova_media_continua - leitura_inicial_fixa;

  Serial.print(F("Altitude = "));
  Serial.print(altitude_relativa_media_movel);
  Serial.println(F(" m"));

  // Detecta o lançamento do foguete (altitude > ALTITUDE_DE_LANCAMENTO)
  if (!foguete_lancado) {
    if (altitude_relativa_media_movel > ALTITUDE_DE_LANCAMENTO) {
      foguete_lancado = true;
      Serial.println(F(">>> LANCAMENTO DETECTADO! ARMANDO SISTEMA DE APOGEU. <<<"));
    }
  }
  // Lógica de voo após o lançamento
  else {
    // Verificação de saltos anômalos de leitura (erro no sensor)
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
    }

    // Verifica a altitude para acionamento do paraquedas/carga com a nova lógica
    // NOTA: A lógica aqui permite o acionamento mesmo abaixo de ALTITUDE_DE_LANCAMENTO
    // se as condições de apogeu baixo e descida forem atendidas, conforme sua análise.
    verificaAltitude(altitude_relativa_media_movel, apogeu, acionamento_iniciado, foguete_lancado, finalizarMissao);
  }
}

/*
 ===================================================================
 lerAltitudeBmp280() função (camelCase)
 ===================================================================
 Função para ler altitude do BMP280.
 No modo forçado, precisamos solicitar uma nova medição.
 Acessa o objeto 'bmp' globalmente.
 */
float lerAltitudeBmp280() {
  // O objeto 'bmp' é global, acessível diretamente aqui.
  // A declaração 'extern Adafruit_BMP280 bmp;' é redundante quando a variável
  // está no próprio arquivo globalmente, mas não causa erro. Removida para clareza.

  if (!bmp.takeForcedMeasurement()) {
    // Se a medição falhar, podemos retornar 0.0, a lógica no loop principal
    // irá tratar isso como um erro se ocorrerem leituras zeradas consecutivas.
    return 0.0;
  }

  // Aciona o LED brevemente para indicar uma leitura bem-sucedida
  digitalWrite(LED_STATUS_PIN, HIGH); // Liga o LED
  delay(1); // Mantém o LED piscando
  digitalWrite(LED_STATUS_PIN, LOW);  // Desliga o LED

  return bmp.readAltitude();
}


// finalizarMissao() função (camelCase)
void finalizarMissao(const __FlashStringHelper *razao_missao) { // Variável renomeada para snake_case
  Serial.println(F(""));
  Serial.println(F("================================================="));
  Serial.print(F("FINALIZANDO MISSAO. MOTIVO: "));
  Serial.println(razao_missao); // Usando a variável renomeada
  Serial.println(F("================================================="));

  // Aciona os MOSFETs se a missão não foi finalizada por erro de inicialização do sensor
  if (razao_missao != F("ERRO CRITICO NA INICIALIZACAO DO SENSOR")) { // Usando a variável renomeada
    Serial.println(F("Acionando MOSFETs com PWM..."));
    analogWrite(PINO_MOSFET_1, 255);
    analogWrite(PINO_MOSFET_2, 255);

    delay(3000); // Delay intencional para manter os MOSFETs acionados por um tempo

    Serial.println(F("Desligando MOSFETs."));
    analogWrite(PINO_MOSFET_1, 0);
    analogWrite(PINO_MOSFET_2, 0);
  }

  // Loop infinito com buzzer e LED piscando para indicar o fim da missão e travar o programa
  while (true) {
    digitalWrite(PINO_BUZZER, HIGH);
    digitalWrite(LED_STATUS_PIN, HIGH); // LED também pisca no final da missão
    delay(300); // Delay intencional para o ritmo do buzzer/LED
    digitalWrite(PINO_BUZZER, LOW);
    digitalWrite(LED_STATUS_PIN, LOW);  // LED também pisca no final da missão
    delay(300); // Delay intencional para o ritmo do buzzer/LED
  }
}

/*
  verificaAltitude() função (camelCase)
  Verifica a altitude para decidir o acionamento do paraquedas/carga.
  Implementa duas lógicas de recuperação:
  1. Foguete voou acima de Xm (100m) e desceu Ym (10m).
  2. Foguete voou abaixo de Xm (100m) e desceu Ym (10m), após ter sido detectado o lançamento.
 */
void verificaAltitude(float altitude, float& apogeu_ref, bool& acionamento_iniciado_ref, bool& foguete_lancado_ref, void (*finalizar)(const __FlashStringHelper *)) {
  // A recuperação só deve ser acionada uma única vez
  if (acionamento_iniciado_ref) {
    return;
  }

  /* --- Lógica 1: Foguete voou alto e começou a descer ---
   Condição: Apogeu atingido foi maior que ALTITUDE_LIMITE_X (100m)
   E a altitude atual caiu pelo menos DESCIDA_MINIMA_Y (10m) abaixo do apogeu.*/
  if (apogeu_ref > ALTITUDE_LIMITE_X && altitude <= apogeu_ref - DESCIDA_MINIMA_Y) {
    acionamento_iniciado_ref = true; // Marca que o acionamento foi iniciado
    finalizar(F("RECUPERACAO: APOGEU ALTO (>100m) E DESCIDA")); // Chama a função para finalizar a missão
    return; // Sai da função após o acionamento
  }

  /* --- Lógica 2: Foguete voou baixo (abaixo de ALTITUDE_LIMITE_X) e começou a descer ---
   Condição: Apogeu atingido foi menor ou igual a ALTITUDE_LIMITE_X (100m)
   E o foguete já foi detectado como lançado (para evitar acionamento no chão antes de voar)
   E a altitude atual caiu pelo menos DESCIDA_MINIMA_Y (10m) abaixo do apogeu.*/
  if (apogeu_ref <= ALTITUDE_LIMITE_X && foguete_lancado_ref && altitude <= apogeu_ref - DESCIDA_MINIMA_Y) {
    acionamento_iniciado_ref = true; // Marca que o acionamento foi iniciado
    finalizar(F("RECUPERACAO: APOGEU BAIXO (<100m) E DESCIDA")); // Chama a função para finalizar a missão
    return; // Sai da função após o acionamento
  }
}