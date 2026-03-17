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

#define TEMPO_MAX 20000 // Tempo máximo de voo para parar de gravar
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
LKF_t* filtroKalman = {0};
volatile uint8_t flagTickVoo = 0;
TIM_TypeDef *TIM_MS;


#ifdef EM_VOO
	const float ALTITUDE_LANCAMENTO = 5.0f;
	const float ALTITUDE_POUSO = 3.0f;
	const float DESCIDA_MINIMA = 10.0f;
#else // EM_SOLO, testes em solo
	const float ALTITUDE_LANCAMENTO = 0.0f;
	const float ALTITUDE_POUSO = 0.0f;
	const float DESCIDA_MINIMA = 2.0f;
#endif


static void acionarEjecao(const char *razao_missao);



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
	LKF_Construtor(filtroKalman);
	filtroKalman->init(filtroKalman, &(sensor.altitude), &(dadosVoo.velocidadeAtual));
	TIM_MS = htim_ms->Instance;

	// INICIA O METRÔNOMO DE 10ms
	HAL_TIM_Base_Start_IT(htim_ms);
}

void processarLogicaVoo(void) {

	processarComandosUSB();
	MS5611_ReadData();
	filtroKalman->update(filtroKalman, sensor.altitude, DT, &(dadosVoo.altitudeAtual), &(dadosVoo.velocidadeAtual));

    switch (dadosVoo.estadoAtual) {
    case ESTADO_CALIBRACAO:
        dadosVoo.estadoAtual = ESTADO_PRONTO;
        adicionarLogW25Q(&hrtc, &dadosVoo);
    	break;
    case ESTADO_PRONTO:

        if (dadosVoo.altitudeAtual > ALTITUDE_LANCAMENTO) {
            // DETECTOU DECOLAGEM!
            dadosVoo.estadoAtual = ESTADO_EM_VOO;
            seguroVoo.tempo_inicio_ms = HAL_GetTick();
            filtroKalman->reiniciar(filtroKalman, &(dadosVoo.altitudeAtual), &(dadosVoo.velocidadeAtual));

            // Agora sim, come�a a gravar
            adicionarLogW25Q(&hrtc, &dadosVoo);

            printLCyan(">>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_EM_VOO]);
            beep(200, 1);
        }
        break;

    case ESTADO_EM_VOO:
        // Atualiza altitude m�xima
        if (dadosVoo.altitudeAtual > seguroVoo.altitude_maxima) {
            seguroVoo.altitude_maxima = dadosVoo.altitudeAtual;
        }

        // Detecta Queda (Apogeu - 10m)
        if (dadosVoo.altitudeAtual < (seguroVoo.altitude_maxima - DESCIDA_MINIMA)) {
            // Eje��o Aqui (Acionar MOSFET se houver)
            dadosVoo.estadoAtual = ESTADO_APOGEU;
        }
        else {
            adicionarLogW25Q(&hrtc, &dadosVoo);
        }
        break;
    case ESTADO_APOGEU:
    	adicionarLogW25Q(&hrtc, &dadosVoo);
    	printLYellow(">>> %s CONFIRMADO <<<", PRINT_ESTADO[ESTADO_APOGEU]);
		dadosVoo.estadoAtual = ESTADO_RECUPERACAO;
		acionarEjecao(PRINT_ESTADO[ESTADO_APOGEU]);
		break;
    case ESTADO_RECUPERACAO:
        // Detecta Pouso (Baixa alt e Baixa dadosVoo.velocidadeAtual )
        if (dadosVoo.altitudeAtual < ALTITUDE_POUSO && dadosVoo.velocidadeAtual > -1.5f && dadosVoo.velocidadeAtual < 1.5f) {
            dadosVoo.estadoAtual = ESTADO_POUSADO;
        }
        else {
            adicionarLogW25Q(&hrtc, &dadosVoo);
        }
        break;
    case ESTADO_POUSADO:
    	adicionarLogW25Q(&hrtc, &dadosVoo);
    	printLGreen(">>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_POUSADO]);
    	pararGravacaoW25Q();
    	beep(1000, 200);
    	break;
    case ESTADO_ERRO:

    	printRed("\n=================================================");
        printRed(">>>    %s!    <<<\r\n>>>    SISTEMA PARALISADO    <<<", PRINT_ESTADO[ESTADO_ERRO]);
        printRed("=================================================");
        // Abortar
        // Adicionar função de erro com beep e razão de erro
        // FORÇA A GRAVAÇÃO DO QUE SOBROU NO BUFFER DA W25Q
        pararGravacaoW25Q();
        acionarEjecao(PRINT_ESTADO[ESTADO_ERRO]);
    	beep(1000, 200);
    	break;
    default: break;
    }

    if ( HAL_GetTick() - seguroVoo.tempo_inicio_ms >= TEMPO_MAX ){ pararGravacaoW25Q(); }
}

#if defined(MOSFET_PORT) && defined(MOSFET_PIN)
	static void acionarEjecao(const char *razao_missao){
		if (dadosVoo.estadoAtual != ESTADO_EM_VOO) return;

		printlnMagenta(T("\n================================================="));
		printlnLCyan(T("ACIONANDO EJECAO EM %.2f m. MOTIVO: %s"), dadosVoo.altitudeAtual, razao_missao);
		printlnMagenta(T("================================================="));
		writePinHigh(MOSFET_PORT, MOSFET_PIN);
		dadosVoo.estadoAtual = ESTADO_RECUPERACAO;

	}
#else
	#error "MOSFET_PORT e MOSFET_PIN não foram definidos no main.h!"
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
/* Precisamos verificar QUEM chamou , pois todos os Timers caem aqui */
	if ( htim -> Instance == TIM_MS )
	{
		flagTickVoo = 1;
	}
}


