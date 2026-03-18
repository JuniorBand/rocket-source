/*
  ******************************************************************************
  * @file    config_voo.c
  * @date 	 14 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Source code que configura a parte complexa do código do foguete.
  ******************************************************************************
*/

/* Obs: essa versão do config_voo.h utiliza a flash W25Q e não a nativa do STM32F4xx.
 *
 *
 *
 *
 */

#include "config_voo.h"
#include "w25q.h"
//#include "flash_stm.h" // Para uso direto da Flash do STM32.
#include "usb_com.h"
#include "main.h"
#include "ms5611.h"
#include <math.h>

#define TEMPO_MAX 80000 // Tempo máximo de voo para parar de gravar
#define DT 10.0f // 10 ms

const char* PRINT_ESTADO[] = {
	"[CALIBRACAO]", // 0
	"[PRONTO]", // 1
	"[VOO]", // 2
	"[APOGEU]", // 3
	"[RECUPERACAO]",  // 4
	"[POUSO]",  // 5
	"[ERRO CRITICO]"  // 6
};

SeguroVoo_t seguroVoo = {0};
DadosVoo_t dadosVoo = {0};
CaixaPreta_t caixaPreta = {0};
MS5611_t sensor = {0};
LKF_t filtroKalman = {0};
volatile uint8_t flagTickVoo = 0;
TIM_TypeDef *TIM_MS;
static u8 contadorFalhasSensor = 0;
static u8 flagGravacaoParada = 0;
static u8 flagFimDeVoo = 0;


#ifdef EM_VOO
	const float ALTITUDE_LANCAMENTO = 5.0f;
	const float ALTITUDE_POUSO = 3.0f;
	const float DESCIDA_MINIMA = 10.0f;
	#define META_APOGEU 300.0f // Defina a meta em metros aqui
	#define DESVIO_MIN 1.0f
#else // EM_SOLO, testes em solo
	const float ALTITUDE_LANCAMENTO = 0.0f;
	const float ALTITUDE_POUSO = 0.0f;
	const float DESCIDA_MINIMA = 2.0f;
	#define META_APOGEU 10.0f // Meta menor para testes
	#define DESVIO_MIN 0.0f
#endif


static void acionarEjecao(const char *razao_missao);
static void verificarErros(void);
static inline void registrarLogVoo(void);
static inline void resgatarFoguete(u32 intervalo, u32 delay, u32 *ultimo_beep);


#if defined(BUZZER_PORT) && defined(BUZZER_PIN)
	void beep(int duracao, int vezes) {
		for (int i = 0; i < vezes; i++) {
			writePinHigh(BUZZER_PORT, BUZZER_PIN); HAL_Delay(duracao);
			writePinLow(BUZZER_PORT, BUZZER_PIN); if (i < vezes - 1) { HAL_Delay(duracao); }
		}
	}
#else
	#error "BUZZER_PORT e BUZZER_PIN não foram definidos no main.h!"
#endif


void setupVoo(SPI_HandleTypeDef *hspi_sensor, TIM_HandleTypeDef *htim_ms){

	dadosVoo.estadoAtual = ESTADO_CALIBRACAO;
	printlnMagenta(">>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_CALIBRACAO]);

	MS5611_Init(hspi_sensor, htim_ms, &sensor);
	LKF_Construtor(&filtroKalman);
	filtroKalman.init(&filtroKalman, &(sensor.altitude), &(dadosVoo.velocidadeAtual));
	TIM_MS = htim_ms->Instance;

	CaixaPreta_t rec = lerCaixaPretaW25Q();

	if (rec.status_voo == MAGIC_NUMBER_SIRIUS) {
		// --- MODO RECUPERACAO (REINICIO NO AR) ---
		printLYellow(">>> AVISO: REINICIO EM VOO DETECTADO! RECUPERANDO DADOS... <<<");

		// 1. Restaura a pressão do chão salva na Caixa Preta
		sensor.pressao_ref = rec.pressao_solo;

		// 2. Pula a fase de "PRONTO" e vai direto para "EM VOO"
		dadosVoo.estadoAtual = ESTADO_EM_VOO;
		seguroVoo.tempo_inicio_ms = HAL_GetTick();

		beep(1000, 1); // Bip longo de alerta
		HAL_Delay(100);
		beep(50, 3);   // 3 bips curtos de pânico

	} else {
		// --- MODO NORMAL (NO CHÃO) ---
		printLCyan(">>> CALIBRANDO SENSOR DE PRESSAO NO SOLO... <<<");

		float soma_pressao = 0;
		int leituras_ok = 0;

		// Faz 15 leituras forçadas para estabilizar a referência do chão
		for (int i = 0; i < 15; i++) {
			// Como o MS5611_ReadData precisa de 3 ciclos (Pedir P -> Ler P -> Ler T)
			// Chamamos 3 vezes com 10ms de delay para simular o Timer funcionando.
			for(int j=0; j<3; j++) {
				MS5611_ReadData();
				HAL_Delay(10);
			}

			if (sensor.pressao > 400.0f) {
				soma_pressao += sensor.pressao;
				leituras_ok++;
			}
		}

		if (leituras_ok > 0) {
			sensor.pressao_ref = soma_pressao / (float)leituras_ok;
		} else {
			sensor.pressao_ref = 1013.25f; // Falha na calibração, usa o padrão do nível do mar
		}

		dadosVoo.estadoAtual = ESTADO_PRONTO; // Vai para a rampa de lançamento
		beep(500, 1); // Bip de calibração concluída
	}

	// INICIA O METRÔNOMO DE 10ms
	HAL_TIM_Base_Start_IT(htim_ms);
}

void processarLogicaVoo(void) {

	processarComandosUSB();
	MS5611_ReadData();
	filtroKalman.update(&filtroKalman, sensor.altitude, DT, &(dadosVoo.altitudeAtual), &(dadosVoo.velocidadeAtual));

	verificarErros();

    switch (dadosVoo.estadoAtual) {

    case ESTADO_CALIBRACAO:
        dadosVoo.estadoAtual = ESTADO_PRONTO;
        registrarLogVoo();
    	break;

    case ESTADO_PRONTO:

        if (dadosVoo.altitudeAtual > ALTITUDE_LANCAMENTO && dadosVoo.velocidadeAtual > 0) {
            // DETECTOU DECOLAGEM!
            dadosVoo.estadoAtual = ESTADO_EM_VOO;
            seguroVoo.tempo_inicio_ms = HAL_GetTick();
            filtroKalman.reiniciar(&filtroKalman, &(dadosVoo.altitudeAtual), &(dadosVoo.velocidadeAtual));

            // Agora sim, come�a a gravar
            registrarLogVoo();
            salvarCaixaPretaW25Q(sensor.pressao_ref, MAGIC_NUMBER_SIRIUS);

            printLCyan(">>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_EM_VOO]);
            beep(200, 1);
        }
        break;

    case ESTADO_EM_VOO:
		// Atualiza altitude máxima
		if (dadosVoo.altitudeAtual > seguroVoo.altitude_maxima) {
			seguroVoo.altitude_maxima = dadosVoo.altitudeAtual;
		}

		// Verifica falha de tempo primeiro (prioridade máxima de erro)
		if (HAL_GetTick() - seguroVoo.tempo_inicio_ms > TEMPO_MAX) {
			dadosVoo.estadoAtual = ESTADO_ERRO;
			registrarLogVoo();
			acionarEjecao("TIMEOUT DE VOO - APOGEU NAO DETECTADO");
		}
		// Ejeção exata ao atingir a meta OU detectar queda (apogeu natural)
		else if ((dadosVoo.altitudeAtual >= META_APOGEU - DESVIO_MIN) ||
				 (dadosVoo.altitudeAtual < (seguroVoo.altitude_maxima - DESCIDA_MINIMA))) {
			dadosVoo.estadoAtual = ESTADO_APOGEU;
		}

		registrarLogVoo();
        break;

    case ESTADO_APOGEU:

    	registrarLogVoo();
    	printLYellow(">>> %s CONFIRMADO <<<", PRINT_ESTADO[ESTADO_APOGEU]);
		acionarEjecao(PRINT_ESTADO[ESTADO_APOGEU]);
		dadosVoo.estadoAtual = ESTADO_RECUPERACAO;
		break;

    case ESTADO_RECUPERACAO:
        // Detecta Pouso (Baixa alt e Baixa dadosVoo.velocidadeAtual )
        if (dadosVoo.altitudeAtual < ALTITUDE_POUSO && dadosVoo.velocidadeAtual > -1.5f \
        		&& dadosVoo.velocidadeAtual < 1.5f) {
            dadosVoo.estadoAtual = ESTADO_POUSADO;
        }
        else {
        	registrarLogVoo();
        }
        break;

    case ESTADO_POUSADO:
    	if (!flagFimDeVoo) {
    		registrarLogVoo();
    		printLGreen(">>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_POUSADO]);
    		// -> CAIXA PRETA: Pousou seguro! Desativa a flag para não entrar em recuperação no próximo boot.
    		salvarCaixaPretaW25Q(1013.25f, 0x00000000);
			pararGravacaoW25Q();
			flagGravacaoParada = 1;
			flagFimDeVoo = 1;
    	}
    	static u32 timerPouso = 0;
    	resgatarFoguete(3000, 1000, &timerPouso);
    	break;

    case ESTADO_ERRO:
    	if (!flagFimDeVoo) {
			printRed("\n=================================================");
			printRed(">>>    %s!    <<<\r\n>>>    SISTEMA PARALISADO    <<<", PRINT_ESTADO[ESTADO_ERRO]);
			printRed("=================================================");
			pararGravacaoW25Q();
			flagGravacaoParada = 1;
			flagFimDeVoo = 1;
        }
    	static u32 timerErro = 0;
    	resgatarFoguete(600, 300, &timerErro);
    	break;

    default: break;
    }
}

#if defined(MOSFET_PORT) && defined(MOSFET_PIN)
	static void acionarEjecao(const char *razao_missao){

		printlnMagenta(T("\n================================================="));
		printlnLCyan(T("ACIONANDO EJECAO EM %.2f m. MOTIVO: %s"), dadosVoo.altitudeAtual, razao_missao);
		printlnMagenta(T("================================================="));
		writePinHigh(MOSFET_PORT, MOSFET_PIN);
		return;
	}
#else
	#error "MOSFET_PORT e MOSFET_PIN não foram definidos no main.h!"
#endif

static void verificarErros(void){

	// Se a pressão for irreal (ex: vácuo absoluto ou leitura morta da SPI)
	if (sensor.pressao < 100.0f) {
		contadorFalhasSensor++;
	} else {
		contadorFalhasSensor = 0; // Recuperou, zera o contador
	}

	// Se ficar 50 ticks (500 ms) seguidos cego no meio do voo
	if (contadorFalhasSensor > 50 && dadosVoo.estadoAtual == ESTADO_EM_VOO) {
		dadosVoo.estadoAtual = ESTADO_ERRO;
		acionarEjecao("FALHA CRITICA DE COMUNICACAO DO BAROMETRO");
	}

	// Proteção contra corrupção matemática
	if (isnan(dadosVoo.altitudeAtual) || isnan(dadosVoo.velocidadeAtual)) {
		if (dadosVoo.estadoAtual == ESTADO_EM_VOO) {
			dadosVoo.estadoAtual = ESTADO_ERRO;
			acionarEjecao("DIVERGENCIA LKF - VARIAVEIS NaN");
		}
	}

    if ( (dadosVoo.estadoAtual >= ESTADO_EM_VOO) \
    		&& ((HAL_GetTick() - seguroVoo.tempo_inicio_ms) >= TEMPO_MAX) \
    		&& (dadosVoo.estadoAtual != ESTADO_RECUPERACAO) && (!flagGravacaoParada)){
    	pararGravacaoW25Q();
    	flagGravacaoParada = 1;
    	printLYellow("AVISO: Tempo máximo de gravação atingido. Log encerrado.");
    }
    return;
}

static inline void registrarLogVoo(void) {
    if (!flagGravacaoParada) {
        adicionarLogW25Q(&hrtc, &dadosVoo);
    }
}

static inline void resgatarFoguete(u32 intervalo, u32 delay, u32 *ultimoBeep) {
    if (HAL_GetTick() - *ultimoBeep > intervalo) {
        writePinHigh(BUZZER_PORT, BUZZER_PIN);
        HAL_Delay(delay);
        writePinLow(BUZZER_PORT, BUZZER_PIN);
        *ultimoBeep = HAL_GetTick();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
/* Precisamos verificar QUEM chamou , pois todos os Timers caem aqui */
	if ( htim -> Instance == TIM_MS )
	{
		flagTickVoo = 1;
	}
	return;
}


