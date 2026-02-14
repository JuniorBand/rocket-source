/*
============================================================================
CODIGO DO FOGUETE - v5.2
- Data da Implementacao: 13 de Fevereiro de 2026
- Historico de Alteracoes:

    RESET DO FILTRO NO LANÇAMENTO (SOLUÇÃO "ANTI-LAG")
        - Antes: Ao detectar `altitude > 5m`, o código apenas mudava o estado para `EM_VOO`. 
            Se o foguete ficasse muito tempo na rampa, a covariância caía e o filtro ficava "lento".
        - Agora: Adicionada a chamada `filtro_ukf_altitude.reiniciar(lerAltitudeBMP280())` 
            dentro de `processarEstadoProntoParaVoo`. Isso "abre" a incerteza do filtro no 
            momento exato da decolagem para capturar a aceleração rápida.

    REFINAMENTO LÓGICO DE INICIALIZAÇÃO
        - Reforço na lógica de `sensor_operacional` para garantir que o sistema inicie 
            assumindo que o sensor está saudável, evitando mensagens falsas de recuperação 
            logo após o boot.

    COMENTÁRIOS EXPLICATIVOS NO FILTRO UKF
        - Como o filtro é complexo, os comentários são essenciais para não nos perdermos.

    ALIASES (APELIDOS)
        - Adicionei aliases para facilitar escrita e reescrita mais limpa do código.
        - Uso de typedef e aliases é muito útil, dá pra explorar mais posteriormente.

============================================================================
*/

// ================================================================
// Inclusao de Bibliotecas
// ================================================================
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <EEPROM.h>


// ================================================================
// ALIASES (APELIDOS)
// ================================================================

typedef uint16_t u16;
typedef uint8_t u8;
typedef uint32_t u32;
typedef unsigned long ulong;
typedef const __FlashStringHelper* cstr_flash;

// ================================================================
// Definicoes de Pinos e Constantes
// ================================================================
constexpr int PINO_MOSFET = 26;
constexpr int PINO_LED_STATUS = 2;
constexpr int PINO_BUZZER = 25;

// PARAMETROS DA MISSAO ATUALIZADOS
constexpr float ALTITUDE_DE_LANCAMENTO = 5.0f;
constexpr float DESCIDA_MINIMA_Y = 10.0f;
constexpr float ALTITUDE_POUSO = 3.0f;

constexpr u16 TAMANHO_EEPROM = 512;
constexpr u32 MARCADOR_EEPROM_VALIDA = 0xFEAD;
constexpr unsigned long INTERVALO_TEMPO_ERRO_SENSOR_EM_VOO = 8000;
constexpr u16 QUANTIDADE_LEITURAS_CALIBRACAO = 100;
constexpr float MAX_SALTO_DE_ALTITUDE = 700.0f;
constexpr u32 I2C_CLOCK_FAST_MODE = 400000L;
constexpr u16 INTERVALO_BPM_300_MS = 100;
constexpr u16 INTERVALO_BPM_400_MS = 75;

// ================================================================
// Estruturas e Enums
// ================================================================
struct DadosDoVoo {
    float altitude_maxima_do_voo; 
    float tempo_ate_o_apogeu; 
    float duracao_do_voo; 
    u32 numero_magico; // Valor marcador para validar se existem dados gravados
}; // Altimetro 

enum class EstadoDoSistema {
    CALIBRACAO, 
    PRONTO_PARA_VOO, 
    EM_VOO, 
    RECUPERACAO,
    POUSADO,
    ERRO_CRITICO 
}; // Máquina de Estados

// ================================================================
// Definicoes de Classes
// ================================================================
class FiltroUKF {
public:
    FiltroUKF();
    void reiniciar(float altitude_inicial) noexcept;
    float atualizar(float altitude_medida, float delta_tempo_s, float* velocidade_estimada = nullptr);
private:
    void decomposicao_cholesky(float A[2][2], float L[2][2]) noexcept; // Para cálculo da raiz quadrada
    float estado[2], 
    covariancia_do_erro[2][2], 
    ruido_do_processo[2][2], // Incerteza da Física (isso é, dos efeitos físicos: vento, atrito, etc)
    ruido_da_medida[1][1]; // Incerteza do Sensor
    float alpha, kappa, beta, // Pontos Sigma
    lambda, // Fator de escala composto
    pesos[3]; // Peso de cada ponto Sigma
}; // Unscented Kalman Filter

class ControladorDeVoo {
public:
    ControladorDeVoo(); // Definido a posteriori
    void configurar(); // Para o setup()
    void processarLoop(); // Para o loop()
private:
    void processarLeituraValida(float altitude_medida_abs, float delta_tempo_s);
    void processarFalhaDeSensor(bool falha_de_comunicacao);
    void processarEstadoProntoParaVoo(float altitude_relativa, float velocidade_estimada);
    void processarEstadoEmVoo(float altitude_relativa, float velocidade_estimada);
    void processarEstadoRecuperacao(float altitude_bruta_relativa);
    bool iniciarSensor() noexcept;
    void calibrarAltitude() noexcept;
    void acionarEjecao(cstr_flash razao_missao, float altitude_acionamento);
    void registrarPouso();
    void erroCritico(cstr_flash razao_erro) noexcept;
    inline float lerAltitudeBMP280() noexcept;
    inline void tocarPadraoBPM(u16 intervalo_ms) noexcept;
    inline void tocarBeeps(u8 quantidade, u16 duracao_ms) noexcept; 
    bool varreduraI2C(byte *endereco_encontrado) noexcept;
    
    EstadoDoSistema estado_atual;
    DadosDoVoo dados_do_voo;
    bool dados_finais_salvos;
    float leitura_inicial_fixa;
    ulong tempo_inicio_do_voo, tempo_erro_sensor, tempo_ultima_leitura_ukf;
    bool sensor_operacional;
    float ultima_altitude_valida_abs;
    int contador_anomalias_seguidas;
    ulong tempo_buzzer_anterior;
    bool estado_buzzer;
    
    Adafruit_BMP280 bmp;
    byte endereco_bmp;
    FiltroUKF filtro_ukf_altitude;
};


ControladorDeVoo controladorDeVoo;

// ================================================================
// Funcoes Padrao do Arduino
// ================================================================
void setup() {
    controladorDeVoo.configurar();
}

void loop() {
    controladorDeVoo.processarLoop();
}

// ================================================================
// Implementacoes da Classe FiltroUKF
// ================================================================
FiltroUKF::FiltroUKF() {
    // ESTADO INICIAL [x]
    estado[0] = 0.0f; // [ 0.0 ] -> posição inicial s0 
    estado[1] = 0.0f; // [ 0.0 ] -> velocidade inicial v0   

    // Modelo
    // COVARIÂNCIA INICIAL [P] - "O quanto eu confio no meu chute inicial?" 
    // Matriz Identidade -> Incerteza padrão, sem correlação entre posição e velocidade.
    // cov(X, Y) ~ incerteza do modelo ou do sensor
    // cov(X, Y) = (desvio_médio)^2, então se cov(e1, e1) = var(e1) = 1, então desvio_médio1 = 1 cm 
    covariancia_do_erro[0][0] = 1.0f; covariancia_do_erro[0][1] = 0.0f; // [ 1.0  0.0 ] cov(e1, e1) = var(e1) = 1.0 e cov(e2, e2) = var(e2) = 1.0
    covariancia_do_erro[1][0] = 0.0f; covariancia_do_erro[1][1] = 1.0f; // [ 0.0  1.0 ] cov(e1, e2) = cov(e2, e1) = 0 -> Zero correlação entre os erros
    
    // PARÂMETROS DO UKF (Unscented Transform)
    // Esses valores definem o quão longe os "Pontos Sigma" (batedores) vão ser jogados.
    alpha = 1.0f; kappa = 0.0f; beta = 2.0f; // Otimizado para distribuições Gaussianas
    lambda = alpha * alpha * (2 + kappa) - 2;
    
    // Física
    // RUÍDO DO PROCESSO [Q] - "O quanto a física do mundo real é caótica?"
    // Valores baixos = confia muito na lei da inércia. Valores altos = o foguete treme muito.
    // cov(e1, e1) = var(e1) = 0.001, então desvio_médio1 =~ 3.16 cm 
    ruido_do_processo[0][0] = 0.001f; ruido_do_processo[0][1] = 0.0f; // [ 0.001  0.0 ] cov(e1, e1) = var(e1) = 0.001 e cov(e2, e2) = var(e2) = 0.001
    ruido_do_processo[1][0] = 0.0f; ruido_do_processo[1][1] = 0.001f; // [ 0.0  0.001 ] cov(e1, e2) = cov(e2, e1) = 0 -> Zero correlação entre os erros
    
    // Sensor
    // RUÍDO DA MEDIDA [R] - "O quanto o sensor BMP280 é ruim?"
    // cov(e1, e1) = var(e1) = 0.1, então desvio_médio1 =~ 31.6 cm
    ruido_da_medida[0][0] = 0.1f; 

    // PESOS [W] - A média ponderada dos pontos sigma
    // O ponto central (média) tem um peso diferente dos pontos periféricos. (pesos < 1)
    pesos[0] = lambda / (2 + lambda);             // Peso do ponto central
    pesos[1] = pesos[2] = 1.0f / (2 * (2 + lambda)); // Peso dos pontos laterais
}


// Reseta o filtro para uma altitude conhecida (usado após a calibração no solo)
void FiltroUKF::reiniciar(float altitude_inicial) noexcept {

    estado[0] = altitude_inicial; // Ponto de chute inicial = leitura_inicial_fixa do sensor
    estado[1] = 0.0f;

    // Reseta a incerteza para o padrão (1.0)
    covariancia_do_erro[0][0] = 1.0f; covariancia_do_erro[0][1] = 0.0f;
    covariancia_do_erro[1][0] = 0.0f; covariancia_do_erro[1][1] = 1.0f;
}

float FiltroUKF::atualizar(float altitude_medida, float delta_tempo_s, float* velocidade_estimada) {
    
    // --- 1. GERAR PONTOS SIGMA (Os "Batedores") ---

    float raiz_covariancia[2][2];

    // A função preenche essa matriz com a raiz quadrada da matriz 'covariancia_do_erro'.
    decomposicao_cholesky(covariancia_do_erro, raiz_covariancia); // raiz_covariancia é o desvio médio

    float pontos_sigma[2][3]; // 3 pontos para 2 dimensões (1 média + 2 desvios)
    float fator_sqrt = sqrt(2 + lambda); // Distância estatística dos pontos
    
    // Ponto 1: Onde achamos que estamos (Média)
    // Na 1º iteração estado[0] = leitura_inicial_fixa e nas outras é corrigido pelo filtro
    // Na 1º iteração estado[1] = 0 e nas outras é corrigido pelo filtro  
    pontos_sigma[0][0] = estado[0]; pontos_sigma[1][0] = estado[1];

    // Ponto 2: Um pouco "pra cima/rápido" (Média + Desvio)
    pontos_sigma[0][1] = estado[0] + fator_sqrt * raiz_covariancia[0][0];
    pontos_sigma[1][1] = estado[1] + fator_sqrt * raiz_covariancia[1][0];
    
    // Ponto 3: Um pouco "pra baixo/lento" (Média - Desvio)
    pontos_sigma[0][2] = estado[0] - fator_sqrt * raiz_covariancia[0][0];
    pontos_sigma[1][2] = estado[1] - fator_sqrt * raiz_covariancia[1][0];
    
    // --- 2. PREDIÇÃO (Física) ---
    // Empurra os 3 pontos para o futuro usando Física Newtoniana (S = So + V*t)

    float predicao_pontos_sigma[2][3];
    for (int i = 0; i < 3; ++i) {
        // Nova Posição = Posição Antiga + Velocidade * Tempo
        predicao_pontos_sigma[0][i] = pontos_sigma[0][i] + pontos_sigma[1][i] * delta_tempo_s;
        // Nova Velocidade = Velocidade Antiga (Modelo de Velocidade Constante)
        predicao_pontos_sigma[1][i] = pontos_sigma[1][i];
    }

    // --- 3. RECONSTRUIR MÉDIA E COVARIÂNCIA PREDITAS ---
    // Calcula onde a nuvem de pontos foi parar em média
    // estado_predito[2] é uma média ponderada da evolução dos pontos sigma no tempo
    float estado_predito[2] = {0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        estado_predito[0] += pesos[i] * predicao_pontos_sigma[0][i];
        estado_predito[1] += pesos[i] * predicao_pontos_sigma[1][i];
    }

    // Covariância do novo estado predito em relação aos pontos sigma var(X) = (xi - μ)^2/N -> Populacional
    float covariancia_predita[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    for (int i = 0; i < 3; ++i) {
        float diff_estado_0 = predicao_pontos_sigma[0][i] - estado_predito[0];
        float diff_estado_1 = predicao_pontos_sigma[1][i] - estado_predito[1];
        covariancia_predita[0][0] += pesos[i] * diff_estado_0 * diff_estado_0;
        covariancia_predita[0][1] += pesos[i] * diff_estado_0 * diff_estado_1;
        covariancia_predita[1][1] += pesos[i] * diff_estado_1 * diff_estado_1;
    }
    
    // Simetriza a matriz
    covariancia_predita[1][0] = covariancia_predita[0][1];

    // Adiciona o Ruído do Processo [Q] (a incerteza natural/física do mundo que cresce com o tempo)
    covariancia_predita[0][0] += ruido_do_processo[0][0];
    covariancia_predita[1][1] += ruido_do_processo[1][1];
    
    // --- 4. ATUALIZAÇÃO (Correção com o Sensor) ---

    // O que esperamos ler no sensor? (Neste caso simples, esperamos ler a própria altitude)
    float predicao_medicao = estado_predito[0];
    float covariancia_medicao = 0.0f;
    float covariancia_cruzada[2] = {0.0f, 0.0f};
    
    for(int i = 0; i < 3; ++i) {
        float diff_medicao = predicao_pontos_sigma[0][i] - predicao_medicao; // mesma coisa da diff_estado_0 da covariancia_predita 
        covariancia_medicao += pesos[i] * diff_medicao * diff_medicao; // mesma coisa da covariancia_predita[0][0]
        // Calcula como a variação na medição se relaciona com a variação no estado real
        covariancia_cruzada[0] += pesos[i] * (predicao_pontos_sigma[0][i] - estado_predito[0]) * diff_medicao; // mesma coisa da covariancia_predita[0][0]
        covariancia_cruzada[1] += pesos[i] * (predicao_pontos_sigma[1][i] - estado_predito[1]) * diff_medicao; // perceba que aqui não é o mesmo que covariancia_predita[1][1], pois diff_medicao é da altura
    }

    // Adiciona o Ruído do Sensor [R]
    covariancia_medicao += ruido_da_medida[0][0];

    // CALCULA O GANHO DE KALMAN [K]
    // K = Covariância Cruzada / Covariância Total da Medição
    float ganho_de_kalman[2];
    float inv_cov_medicao = 1.0f / covariancia_medicao; // Divisão é lenta, fazemos 1 vez
    ganho_de_kalman[0] = covariancia_cruzada[0] * inv_cov_medicao;
    ganho_de_kalman[1] = covariancia_cruzada[1] * inv_cov_medicao;

    // APLICAR A CORREÇÃO FINAL
    // Resíduo = O que o sensor leu REALMENTE - O que eu esperava ler
    float residuo = altitude_medida - predicao_medicao; // a nova medida de altitude - a da nossa previsão

    // Novo Estado = Predição + (Ganho * "Surpresa do Sensor")
    estado[0] = estado_predito[0] + ganho_de_kalman[0] * residuo;
    estado[1] = estado_predito[1] + ganho_de_kalman[1] * residuo;

    // ATUALIZA A INCERTEZA (Covariância Final)
    // P_novo = P_predito - K * S * K_transposto
    float KSKT[2][2];
    KSKT[0][0] = ganho_de_kalman[0] * covariancia_medicao * ganho_de_kalman[0];
    KSKT[0][1] = ganho_de_kalman[0] * covariancia_medicao * ganho_de_kalman[1];
    KSKT[1][0] = ganho_de_kalman[1] * covariancia_medicao * ganho_de_kalman[0];
    KSKT[1][1] = ganho_de_kalman[1] * covariancia_medicao * ganho_de_kalman[1];

    covariancia_do_erro[0][0] = covariancia_predita[0][0] - KSKT[0][0];
    covariancia_do_erro[0][1] = covariancia_predita[0][1] - KSKT[0][1];
    covariancia_do_erro[1][0] = covariancia_predita[1][0] - KSKT[1][0];
    covariancia_do_erro[1][1] = covariancia_predita[1][1] - KSKT[1][1];

    // covariancia_do_erro vai ser reutilizada no próximo loop na decomposição de Cholesky
    // para estimar os próximos pontos sigma

    // Exporta a velocidade se o usuário pediu
    if (velocidade_estimada != nullptr) { *velocidade_estimada = estado[1]; }

    return estado[0]; // Retorna a altitude filtrada
}

// ALGEBRA LINEAR PURA
// Recebe matriz A (entrada) e matriz L (saída)
// Em C++, "float L[2][2]" é um ponteiro para o primeiro elemento. 
// A função escreve DIRETAMENTE na memória de L.
void FiltroUKF::decomposicao_cholesky(float A[2][2], float L[2][2]) noexcept {
    // L é uma matriz triangular inferior tal que L * L_transposta = A
    // É o equivalente matricial de tirar a raiz quadrada de um número.

    L[0][0] = sqrt(A[0][0]);
    L[0][1] = 0.0f; // Triangular inferior (acima da diagonal é zero)
    L[1][0] = A[1][0] / L[0][0];

    // Elemento (1,1) - Garante estabilidade numérica
    float val_interno = A[1][1] - L[1][0] * L[1][0];
    // Proteção contra raiz de negativo (se a matriz deixar de ser positiva definida por erro numérico)
    if (val_interno < 0) val_interno = 0; 
    L[1][1] = sqrt(val_interno);

}

// ================================================================
// Implementacoes da Classe ControladorDeVoo
// ================================================================
ControladorDeVoo::ControladorDeVoo() :
    estado_atual(EstadoDoSistema::CALIBRACAO),
    dados_finais_salvos(false),
    leitura_inicial_fixa(0.0f),
    tempo_inicio_do_voo(0),
    tempo_erro_sensor(0),
    tempo_ultima_leitura_ukf(0),
    tempo_buzzer_anterior(0),
    estado_buzzer(false),
    endereco_bmp(0),
    sensor_operacional(true), // MUDANÇA: era false, agora é true
    ultima_altitude_valida_abs(0.0f),
    contador_anomalias_seguidas(0)
{}

void ControladorDeVoo::configurar() {
    Serial.begin(115200);
    
    pinMode(PINO_MOSFET, OUTPUT);
    pinMode(PINO_BUZZER, OUTPUT);
    pinMode(PINO_LED_STATUS, OUTPUT);
    
    digitalWrite(PINO_MOSFET, LOW);
    digitalWrite(PINO_BUZZER, LOW);
    digitalWrite(PINO_LED_STATUS, LOW);
    
    Serial.println(F("\n================================================="));
    Serial.println(F(" COMPUTADOR DE VOO v7.5-MISSION_PARAMS_UPDATE"));
    Serial.println(F("================================================="));
    
    tocarBeeps(1, 300);
    Wire.begin(21, 22); // GPIO 21 como SDA (data) e GPIO 22 como SCL (clock)
    Wire.setClock(I2C_CLOCK_FAST_MODE);

    if (!EEPROM.begin(TAMANHO_EEPROM)) { // Reserva espaço na Flash
        erroCritico(F("Falha ao inicializar a EEPROM."));
        return;
    }
    if (!iniciarSensor()) {
        erroCritico(F("Nao foi possivel iniciar o sensor BMP280."));
        return;
    }

    calibrarAltitude();
    estado_atual = EstadoDoSistema::PRONTO_PARA_VOO;
    tempo_ultima_leitura_ukf = millis();
    // Vai pro loop()
}

void ControladorDeVoo::processarLoop() {
    if (estado_atual == EstadoDoSistema::POUSADO || estado_atual == EstadoDoSistema::ERRO_CRITICO) {
        tocarPadraoBPM(estado_atual == EstadoDoSistema::POUSADO ? INTERVALO_BPM_400_MS : INTERVALO_BPM_300_MS);
        return;
    }

    unsigned long tempo_atual = millis();
    float delta_tempo_s = static_cast<float>(tempo_atual - tempo_ultima_leitura_ukf) / 1000.0f; // Segurança de casting de novo
    tempo_ultima_leitura_ukf = tempo_atual;

    const float altitude_medida_abs = lerAltitudeBMP280();
    
    bool falha_de_comunicacao = isnan(altitude_medida_abs);
    bool falha_de_anomalia = !falha_de_comunicacao && (altitude_medida_abs > (ultima_altitude_valida_abs + MAX_SALTO_DE_ALTITUDE));

    if (falha_de_comunicacao || falha_de_anomalia) {
        processarFalhaDeSensor(falha_de_comunicacao);
    } else {
        processarLeituraValida(altitude_medida_abs, delta_tempo_s);
    }

    if (estado_atual == EstadoDoSistema::RECUPERACAO) {
        tocarPadraoBPM(INTERVALO_BPM_400_MS);
    }
}

void ControladorDeVoo::processarLeituraValida(float altitude_medida_abs, float delta_tempo_s) { 
    if (!sensor_operacional) { // Supondo que não dê erro na primeira vez (falha_de_comunicacao || falha_de_anomalia),
    // Temos que isso aqui vai ser lido se sensor_operacional for false como no construtor, 
    // Então atualizemos o construtor para ter ele como true inicialmente, até q o sensor falhe de fato
        digitalWrite(PINO_BUZZER, LOW); 
        estado_buzzer = false;
        Serial.println(F(">>> SENSOR RECUPERADO! <<<"));
    }

    sensor_operacional = true;
    contador_anomalias_seguidas = 0;
    ultima_altitude_valida_abs = altitude_medida_abs;

    float velocidade_estimada = 0;
    float altitude_relativa = 0;

    if (estado_atual == EstadoDoSistema::PRONTO_PARA_VOO || estado_atual == EstadoDoSistema::EM_VOO) {
        const float altitude_filtrada_abs = filtro_ukf_altitude.atualizar(altitude_medida_abs, delta_tempo_s, &velocidade_estimada);
        altitude_relativa = altitude_filtrada_abs - leitura_inicial_fixa;
    } else {
        altitude_relativa = altitude_medida_abs - leitura_inicial_fixa;
    }

    switch (estado_atual) {
        case EstadoDoSistema::PRONTO_PARA_VOO:
            processarEstadoProntoParaVoo(altitude_relativa, velocidade_estimada);
            break;
        case EstadoDoSistema::EM_VOO:
            if (tempo_erro_sensor != 0) {
                Serial.println(F("Sensor recuperado em voo. Timer de emergencia desativado."));
                tempo_erro_sensor = 0;
            }
            processarEstadoEmVoo(altitude_relativa, velocidade_estimada);
            break;
        case EstadoDoSistema::RECUPERACAO:
            processarEstadoRecuperacao(altitude_relativa);
            break;
        default: break;
    }
}

void ControladorDeVoo::processarFalhaDeSensor(bool falha_de_comunicacao) {
    sensor_operacional = false;
    
    if (!falha_de_comunicacao) {
        contador_anomalias_seguidas++;
        Serial.printf("ALERTA: Salto anomalo de altitude! Contagem: %d\n", contador_anomalias_seguidas);
    }

    switch (estado_atual) {
        case EstadoDoSistema::PRONTO_PARA_VOO:
            Serial.println(F("ALERTA: Sensor em falha! Verifique a conexao."));
            tocarPadraoBPM(INTERVALO_BPM_400_MS);
            break;
        case EstadoDoSistema::EM_VOO:
            if (tempo_erro_sensor == 0 && (falha_de_comunicacao || contador_anomalias_seguidas >= 4)) {
                tempo_erro_sensor = millis();
                Serial.println(F("!!! ALERTA: Falha grave do sensor em voo! Acionando timer de emergencia..."));
            }
            if (tempo_erro_sensor != 0 && (millis() - tempo_erro_sensor > INTERVALO_TEMPO_ERRO_SENSOR_EM_VOO)) {
                float ultima_altitude_relativa = ultima_altitude_valida_abs - leitura_inicial_fixa;
                acionarEjecao(F("RECUPERACAO DE EMERGENCIA: TIMEOUT SENSOR"), ultima_altitude_relativa);
            }
            break;
        default: break;
    }
}

void ControladorDeVoo::processarEstadoProntoParaVoo(float altitude_relativa, float velocidade_estimada) {
    Serial.printf("Aguardando lancamento... Altitude: %.2f m\n", altitude_relativa);

    if (altitude_relativa > ALTITUDE_DE_LANCAMENTO) {
        estado_atual = EstadoDoSistema::EM_VOO;
        tempo_inicio_do_voo = millis();
        dados_do_voo = {};
        dados_do_voo.numero_magico = MARCADOR_EEPROM_VALIDA;
        dados_do_voo.altitude_maxima_do_voo = altitude_relativa;
        dados_finais_salvos = false;

        // MUDANÇA:
        // O foguete já subiu 5 metros. O filtro pode estar "viciado" achando que está parado.
        // Vamos dizer pra ele: "Eu sei que estou em 5m, mas ABRA A INCERTEZA sobre a velocidade agora!"
        filtro_ukf_altitude.reiniciar(lerAltitudeBMP280()); 
        // --------------------------

        Serial.println(F(">>> DETECCAO DE LANCAMENTO! MODO 'VOO' ATIVADO. <<<"));
    }
}

void ControladorDeVoo::processarEstadoEmVoo(float altitude_relativa, float velocidade_estimada) {
    Serial.printf("Em Voo -> Altitude: %.2f m\n", altitude_relativa);

    if (altitude_relativa > dados_do_voo.altitude_maxima_do_voo) {
        dados_do_voo.altitude_maxima_do_voo = altitude_relativa;
        dados_do_voo.tempo_ate_o_apogeu = (millis() - tempo_inicio_do_voo) / 1000.0f;
    }
    
    if (altitude_relativa < (dados_do_voo.altitude_maxima_do_voo - DESCIDA_MINIMA_Y)) {
        acionarEjecao(F("APOGEU DETECTADO"), altitude_relativa);
    }
}

void ControladorDeVoo::processarEstadoRecuperacao(float altitude_bruta_relativa) {
    if (altitude_bruta_relativa <= ALTITUDE_POUSO) {
        registrarPouso();
    }
}

bool ControladorDeVoo::iniciarSensor() noexcept {
    byte endereco_temporario = 0;
    Serial.println(F("Iniciando varredura I2C..."));

    if (!varreduraI2C(&endereco_temporario)) {
        Serial.println(F("Nenhum BMP280 encontrado no barramento I2C."));
        return false;
    }

    endereco_bmp = endereco_temporario;

    Serial.printf(F("BMP280 encontrado no endereco 0x%X.\n"), endereco_bmp);

    if (!bmp.begin(endereco_bmp)) {
        Serial.println(F("Falha ao iniciar comunicacao com o BMP280."));
        return false;
    }

    bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X1,
                    Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X8,
                    Adafruit_BMP280::STANDBY_MS_1); // Pesquisar mais

    Serial.println(F("Sensor BMP280 configurado com sucesso!"));
    sensor_operacional = true;
    return true;
}

void ControladorDeVoo::calibrarAltitude() noexcept {
    Serial.println(F("Iniciando calibracao de altitude (versao rapida)..."));
    float soma_buffer_calibracao = 0;
    int leituras_validas = 0;

    for (int i = 0; i < QUANTIDADE_LEITURAS_CALIBRACAO; ++i) {
        float alt_abs = lerAltitudeBMP280();
        if (!isnan(alt_abs)) {
            soma_buffer_calibracao += alt_abs;
            leituras_validas++;
        }
        delay(10);
    }

    if (leituras_validas > 10) {
        leitura_inicial_fixa = soma_buffer_calibracao / static_cast<float>(leituras_validas); // static cast só por seguranç ade casting
        ultima_altitude_valida_abs = leitura_inicial_fixa;
    } else {
        erroCritico(F("Nao foi possivel ler o sensor durante a calibracao."));
        return;
    }

    filtro_ukf_altitude.reiniciar(leitura_inicial_fixa);

    Serial.println(F("================================================="));
    Serial.printf(F("Calibracao concluida. Altitude de referencia: %.2f m.\n"), leitura_inicial_fixa);
    Serial.println(F("SISTEMA PRONTO PARA VOO. AGUARDANDO LANCAMENTO..."));
    Serial.println(F("================================================="));
    tocarBeeps(2, 200);
}

void ControladorDeVoo::acionarEjecao(cstr_flash razao_missao, float altitude_acionamento) {
    if (estado_atual != EstadoDoSistema::EM_VOO) return;
    
    Serial.println(F("\n================================================="));
    Serial.printf(F("ACIONANDO EJECAO EM %.2f m. MOTIVO: %s\n"), altitude_acionamento, razao_missao);
    Serial.println(F("================================================="));
    digitalWrite(PINO_MOSFET, HIGH);
    estado_atual = EstadoDoSistema::RECUPERACAO;
}

void ControladorDeVoo::registrarPouso() {
    if (dados_finais_salvos) return;

    dados_finais_salvos = true;
    estado_atual = EstadoDoSistema::POUSADO;
    dados_do_voo.duracao_do_voo = (millis() - tempo_inicio_do_voo) / 1000.0f;

    EEPROM.put(0, dados_do_voo);
    EEPROM.commit();

    Serial.println(F("\n================================================="));
    Serial.println(F("POUSO DETECTADO. DADOS FINAIS SALVOS."));
    Serial.println(F("Relatorio Final de Voo:"));
    Serial.printf(F("- Altitude Maxima: %.2f m\n"), dados_do_voo.altitude_maxima_do_voo);
    Serial.printf(F("- Tempo ate o Apogeu: %.2f s\n"), dados_do_voo.tempo_ate_o_apogeu);
    Serial.printf(F("- Duracao Total do Voo: %.2f s\n"), dados_do_voo.duracao_do_voo);
    Serial.println(F("================================================="));
}

void ControladorDeVoo::erroCritico(cstr_flash razao_erro) noexcept {
    Serial.println(F("\n================================================="));
    Serial.printf(F("ERRO CRITICO! %s\nSISTEMA PARALISADO.\n"), razao_erro);
    Serial.println(F("================================================="));
    estado_atual = EstadoDoSistema::ERRO_CRITICO;
}

inline float ControladorDeVoo::lerAltitudeBMP280() noexcept {
    if (!bmp.takeForcedMeasurement()) { return NAN; }
    float leitura = bmp.readAltitude();
    return isnan(leitura) ? NAN : leitura;
}

inline void ControladorDeVoo::tocarPadraoBPM(u16 intervalo_ms) noexcept {
    const unsigned long tempo_atual = millis(); 
    if (tempo_atual - tempo_buzzer_anterior >= intervalo_ms) {
        estado_buzzer = !estado_buzzer;
        digitalWrite(PINO_BUZZER, estado_buzzer);
        tempo_buzzer_anterior = tempo_atual;
    }
} // Perfeito para inline


inline void ControladorDeVoo::tocarBeeps(u8 quantidade, u16 duracao_ms) noexcept {
    for (u8 i = 0; i < quantidade; ++i) {
        digitalWrite(PINO_BUZZER, HIGH);
        delay(duracao_ms);
        digitalWrite(PINO_BUZZER, LOW);
        if (i < quantidade - 1) delay(duracao_ms);
    }
} // Perfeito para inline

bool ControladorDeVoo::varreduraI2C(byte *endereco_encontrado) noexcept {
    for (byte endereco = 0x76; endereco <= 0x77; ++endereco) {
        Wire.beginTransmission(endereco);
        if (Wire.endTransmission() == 0) {
            *endereco_encontrado = endereco;
            return true;
        }
    }
    return false;
} // Varredura Padrão