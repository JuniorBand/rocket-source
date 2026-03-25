/*
  ******************************************************************************
  * @file    config_voo.c
  * @date 	 14 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Source code que configura a parte complexa do código do foguete.
  ******************************************************************************
*/

/* OBS: Esta documentacao refere-se a arquitetura central do firmware (RAD).
 *
 * -> No config_voo.h e onde, de fato, esta implementada a logica e os parametros de
 * voo. Significa que o main.c fica limpo, utilizamos apenas duas funcoes desse arquivo
 * por la para iniciar as configs e implementacoes do RAD.
 *
 * -> O config_voo.h/.c e o cerebro do projeto, o utils.h e a mao direita do compilador
 * e o prints.h e o olho das Greias (que tudo ve, o Grande Irmao).
 *
 * COMO USAR:
 *
 * LEMBRE-SE:
 *
 * -> Codigos das pastas criadas pelo CubeMX precisam estar em areas
 * demarcadas como USER CODE, para que, caso voce atualize o projeto no CubeMX
 * e gere novamente as pastas padrao, seu codigo nao seja apagado.
 *
 * SETUP NO CUBEIDE:
 *
 * -> Cole a pasta RAD dentro do diretorio principal do projeto, onde fica
 * seu .ioc ou escolha um lugar fixo e seguro nas suas pastas pessoais. A
 * primeira escolha e recomendada para fins de facilidade de debug/visualizacao.
 *
 * -> Em [Project -> Properties], ou clicando com o lado direito do mouse na pasta
 * principal do projeto onde le-se "Properties", siga para:
 *
 *		1. Para garantir a compilacao dos arquivos fonte (Source Location):
 * - C/C++ General -> Paths and Symbols -> aba Source Location.
 * - Verifique se a pasta 'RAD' (ou 'RAD/Src') esta listada. Se nao estiver, clique
 * em 'Add Folder...' e selecione-a.
 *
 * 2. Para adicionar o diretorio dos cabecalhos (Include Path):
 * - C/C++ Build -> Settings -> Tool Settings -> MCU GCC Compiler -> Include paths.
 * - Clique no icone 'Add...' (papel com um + verde) no quadro "Include paths (-I)".
 * - Clique em 'Workspace...' e navegue ate selecionar a pasta 'Inc' dentro de 'RAD'
 * e tambem adicione a pasta 'utils' dentro de 'RAD/Inc' (ou seja, adicione ambas as
 * pastas: 'RAD/Inc' e 'RAD/Inc/utils').
 * (O caminho ficara parecido com "${workspace_loc:/${ProjName}/RAD/Inc}").
 *
 * 3. Para habilitar o printf com suporte a float:
 * - Ainda em C/C++ Build -> Settings -> Tool Settings, va para -> MCU Settings.
 * - Marque a opcao: "Use float with printf from newlib-nano (-u _printf_float)".
 * - Clique em 'Apply and Close' e faca um Rebuild (Ctrl+B) no projeto.
 *
 * INTEGRACAO NO CODIGO:
 *
 * -> No main.c inclua o header config_voo.h, chame setupVoo na funcao main e
 * chame processarLogicaVoo dentro do loop while(1).
 * E e so isso, a logica toda esta dentro da pasta RAD - Random Access Drivers.
 *
 * -> No config_voo.h (Seu Painel de Controle): defina a macro EM_VOO (para voo real ou
 * bancada) e a macro USE_W25Q (para escolher entre a memoria SPI externa ou nativa).
 * * -> Nota: A funcao interruptUSB (usb_com.c) precisa ser a primeira a ser chamada
 * dentro da funcao CDC_Receive_FS do arquivo usbd_cdc_if.c, dessa forma:
 * interruptUSB(Buf, Len); // lembre-se de fazer o #include <usb_com.h>
 *
 * ATENCAO (HARDWARE E COMPILADOR):
 *
 * -> Lembre de definir as macros dos pinos/portas no main.h.
 * As macros utilizadas aqui sao: MS5611_PORT, MS5611_CS_PIN, BUZZER_PIN, BUZZER_PORT,
 * MOSFET_PIN, MOSFET_PORT, W25Q_CS_PIN, W25Q_CS_PORT. Caso MOSFET_PIN e MOSFET_PORT
 * nao sejam definidos, a funcao acionarEjecao sera um mock seguro para simulacao.
 *
 * -> Nao use double, use float para nao haver erros em relacao ao nao uso da FPU
 * (doubles sao 64 bits no GCC para ARM, forcando conta via software).
 *
 */

#include <config_voo.h>
#include <main.h>
#include <utils.h>
#include <ms5611.h>
#include <math.h>
#include <usb_com.h> // Nao utilizado EM_VOO

#ifdef USE_W25Q
    #include <w25q.h>
#else
    #include <flash_stm.h>
#endif

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
static TIM_TypeDef *TIM_MS;
static RTC_HandleTypeDef *HRTC_SYS_PTR;
volatile u8 flagTickVoo = 0;
static u8 contadorFalhasSensor = 0;
static u8 flagGravacaoParada = 0;
static u8 flagFimDeVoo = 0;


static void acionarEjecao(const char *razao_missao);
static void verificarErros(void);
static inline void registrarLogVoo(void);
static inline void resgatarFoguete(u32 intervalo, u32 tempo_ligado, u32 *ultimo_beep);


void setupVoo(SPI_HandleTypeDef *hspi_mem, SPI_HandleTypeDef *hspi_sensor, TIM_HandleTypeDef *htim_ms, RTC_HandleTypeDef *hrtc_sys){

	printlnLCyan("\r\n=================================================");
	printlnLCyan("%*s%s", 9, "", "INICIANDO SISTEMA DE TELEMETRIA");
    //printlnLCyan("         INICIANDO SISTEMA DE TELEMETRIA         ");
	printlnLCyan("=================================================");

	dadosVoo.estadoAtual = ESTADO_CALIBRACAO;
	printlnMagenta("\r\n>>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_CALIBRACAO]);
	acenderLedPlaca();

	w25qInit(hspi_mem, htim_ms, W25Q_CS_PIN);
	MS5611_Init(hspi_sensor, htim_ms, &sensor);
	LKF_Construtor(&filtroKalman);
	filtroKalman.init(&filtroKalman, &(sensor.altitude), &(dadosVoo.velocidadeAtual));
	TIM_MS = htim_ms->Instance;
	HRTC_SYS_PTR = hrtc_sys;

	CaixaPreta_t rec = lerCaixaPretaW25Q();

	if (rec.status_voo == MAGIC_NUMBER_SIRIUS) {
		// --- MODO RECUPERACAO (REINICIO NO AR) ---
		printLYellow("\r\n>>> AVISO: REINICIO EM VOO DETECTADO! RECUPERANDO DADOS... <<<");

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
		printlnLCyan("\r\n>>> CALIBRANDO SENSOR DE PRESSAO NO SOLO... <<<");

		float soma_pressao = 0;
		u8 leituras_ok = 0;

		// Faz 15 leituras forçadas para estabilizar a referência do chão
		for (u8 i = 0; i < 15; i++) {
			// Como o MS5611_ReadData precisa de 3 ciclos (Pedir P -> Ler P -> Ler T)
			// Chamamos 3 vezes com 10ms de delay para simular o Timer funcionando.
			for(u8 j = 0; j < 3; j++) {
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
	apagarLedPlaca();
}

void processarLogicaVoo(void) {
	if (flagTickVoo == 1) {

		flagTickVoo = 0; // Abaixa a bandeira imediatamente

		// Agora sim, executa a lógica pesada de sensores e flash com tempo garantido
		#ifndef EM_VOO
			#pragma message("Utilizando conexão via USB.")
			processarComandosUSB();
		#endif

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

				printLCyan("\r\n>>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_EM_VOO]);
				beep(200, 1);
			}
			break;

		case ESTADO_EM_VOO:
			// Atualiza altitude máxima
			if (dadosVoo.altitudeAtual > seguroVoo.altitude_maxima) {
				seguroVoo.altitude_maxima = dadosVoo.altitudeAtual;
			}

			// Verifica falha de tempo primeiro (prioridade máxima de erro)
			if (HAL_GetTick() - seguroVoo.tempo_inicio_ms > TEMPO_MAX_VOO_MS) {
				dadosVoo.estadoAtual = ESTADO_ERRO;
				registrarLogVoo();
				acionarEjecao("\r\nTIMEOUT DE VOO - APOGEU NAO DETECTADO");
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
			printLYellow("\r\n>>> %s CONFIRMADO <<<", PRINT_ESTADO[ESTADO_APOGEU]);
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
				printLGreen("\r\n>>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_POUSADO]);
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
				printRed("\r\n=================================================");
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
}


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
    		&& ((HAL_GetTick() - seguroVoo.tempo_inicio_ms) >= TEMPO_MAX_VOO_MS) \
    		&& (dadosVoo.estadoAtual != ESTADO_RECUPERACAO) && (!flagGravacaoParada)){
    	pararGravacaoW25Q();
    	flagGravacaoParada = 1;
    	printLYellow("\r\nAVISO: Tempo máximo de gravação atingido. Log encerrado.");
    }
    return;
}

static inline void registrarLogVoo(void) {
    if (!flagGravacaoParada) {
        adicionarLogW25Q(HRTC_SYS_PTR, &dadosVoo);
    }
}

static inline void resgatarFoguete(u32 intervalo, u32 tempo_ligado, u32 *ultimoBeep) {
    u32 tempo_atual = HAL_GetTick();

    // Se passou do intervalo, liga o buzzer e atualiza a marcação de tempo
    if (tempo_atual - *ultimoBeep >= intervalo) {
    	BUZZER_ON();
        *ultimoBeep = tempo_atual;
    }
    // Desliga o buzzer se já passou o tempo_ligado desde a última marcação
    else if (tempo_atual - *ultimoBeep >= tempo_ligado) {
        writePinLow(BUZZER_PORT, BUZZER_PIN);
        BUZZER_OFF();
    }
}

#if defined(MOSFET_PORT) && defined(MOSFET_PIN)
	#pragma message("MOSFET_PORT e MOSFET_PIN foram definidos.")

	static void acionarEjecao(const char *razao_missao){

		printlnMagenta("\r\n=================================================");
		printlnLCyan("ACIONANDO EJECAO EM %.2f m. MOTIVO: %s", dadosVoo.altitudeAtual, razao_missao);
		printlnMagenta("=================================================");
		writePinHigh(MOSFET_PORT, MOSFET_PIN);
		return;
	}
#else
	#pragma message("MOSFET_PORT e MOSFET_PIN não foram definidos no main.h!")

	static void acionarEjecao(const char *razao_missao){
			printlnMagenta("\r\n=================================================");
			printlnLCyan("[SIMULACAO] ACIONANDO EJECAO EM %.2f m. MOTIVO: %s", dadosVoo.altitudeAtual, razao_missao);
			printlnMagenta("=================================================");
			return;
	}
#endif

#if defined(BUZZER_PORT) && defined(BUZZER_PIN)
	#pragma message("BUZZER_PORT e BUZZER_PIN foram definidos.")

	void beep(u32 duracao, u8 vezes) {
		for (u8 i = 0; i < vezes; i++) {
			writePinHigh(BUZZER_PORT, BUZZER_PIN); HAL_Delay(duracao);
			writePinLow(BUZZER_PORT, BUZZER_PIN); if (i < vezes - 1) { HAL_Delay(duracao); }
		}
	}
#else
	#error "BUZZER_PORT e BUZZER_PIN não foram definidos no main.h!"
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
/* Precisamos verificar QUEM chamou , pois todos os Timers caem aqui */
	if ( htim -> Instance == TIM_MS )
	{
		flagTickVoo = 1;
	}
	return;
}

// ============================================================================
// FUNCAO DE SIMULACAO (MOCK) DIRETO PARA O TERMINAL SERIAL (SEM W25Q)
// ============================================================================
void simularVooAoVivoUSB(void) {
    char buffer_saida[256];
    char barra_grafica[55];

    // Banner Inicial 100% simetrico (118 caracteres de largura exatos)
    printlnLCyan("\r\n======================================================================================================================");
    printlnLYellow("                                      SIMULACAO DE VOO (SITL) - TEMPO REAL                                      ");
    printlnLCyan("======================================================================================================================");
    printlnLMagenta("   HORA   |        TELEMETRIA (A/V/P/T)           |     STATUS     | GRAFICO (# = 10m)                                ");
    printlnLCyan("----------|---------------------------------------|----------------|--------------------------------------------------");

    float alt = 0.0f;
    float vel = 0.0f;
    u8 estado = 1; // 1 = ESTADO_PRONTO
    u32 tempo_ms = 0;
    u8 h = 14, m = 30, s = 0;
    u32 marca_pouso = 0;

    // --- RASTREADORES DE ESTATISTICAS ---
    float alt_max = 0.0f;
    u32 tempo_decolagem = 0;
    u32 tempo_toque_chao = 0;

    for (int step = 0; step < 12000; step++) {

        // --- FISICA BASICA DO FOGUETE ---
        if (estado == 1 && tempo_ms > 1000) {
            estado = 2; // Decola apos 1s
            tempo_decolagem = tempo_ms; // <-- Marca a hora que saiu do chao
        } else if (estado == 2) {
            if (tempo_ms < 3000) vel += 45.0f * 0.01f; // Motor
            else vel -= 9.81f * 0.01f; // Gravidade
            if (vel <= 0.0f && alt > 50.0f) estado = 3; // Apogeu
        } else if (estado == 3) {
            estado = 4; vel = -15.0f; // Paraquedas abre
        } else if (estado == 4) {
            vel += ( (-8.0f - vel) * 0.05f ); // Arrasto do paraquedas
            if (alt <= 0.0f) {
                estado = 5; vel = 0.0f; alt = 0.0f;
                tempo_toque_chao = tempo_ms; // <-- Marca a hora exata do pouso
            }
        }

        alt += vel * 0.01f;
        if (alt < 0) alt = 0.0f;

        // Atualiza a altitude maxima da simulacao
        if (alt > alt_max) alt_max = alt;

        float pressao = 1013.25f * powf(1.0f - (alt / 44330.0f), 5.255f);
        float temp = 30.0f - (alt * 0.0065f);

        tempo_ms += 10;
        if (tempo_ms % 1000 == 0) {
            s++;
            if (s > 59) { s = 0; m++; }
            if (m > 59) { m = 0; h++; }
        }

        // --- IMPRESSAO NO TERMINAL A CADA 100ms ---
        if (tempo_ms % 100 == 0) {

            int num_barras = (int)(alt / 10.0f);
            if (num_barras > 50) num_barras = 50;
            if (num_barras < 0) num_barras = 0;

            memset(barra_grafica, 0, sizeof(barra_grafica));
            for (int i = 0; i < num_barras; i++) barra_grafica[i] = '#';

            u8 st = estado;
            if (st > 6) st = 6;

            snprintf(buffer_saida, sizeof(buffer_saida),
                "%s %02d:%02d:%02d %s|"
                "%s A:%7.1f V:%7.1f P:%6.1f T:%6.1f %s|"
                "%s %-14s %s|"
                "%s %s",
                RESET, h, m, s, LMAGENTA,
                LCYAN, alt, vel, pressao, temp, LMAGENTA,
                LYELLOW, PRINT_ESTADO[st], LMAGENTA,
                LGREEN, barra_grafica);

            printf("%s%s\r\n", buffer_saida, RESET);
        }

        HAL_Delay(10);

        // Fica imprimindo os logs no chao por 2 segundos antes de encerrar
        if (estado == 5) {
            if (marca_pouso == 0) marca_pouso = tempo_ms;
            if (tempo_ms - marca_pouso > 2000) break;
        }
    }

    // --- CALCULO DAS ESTATISTICAS E RODAPE ---
    u32 tempo_voo_s = (tempo_toque_chao - tempo_decolagem) / 1000;
    u32 runtime_s = tempo_ms / 1000;

    printlnMagenta("======================================================================================================================");
    snprintf(buffer_saida, sizeof(buffer_saida),
            "ESTATISTICAS FINAIS:\r\n- Altitude Maxima: %.2f m; Tempo de Voo: %lu s; Runtime: %lu s.",
            alt_max, tempo_voo_s, runtime_s);
    printlnLGreen("%s", buffer_saida);
    printlnMagenta("======================================================================================================================");
    printlnLGreen("\r\n--- SIMULACAO CONCLUIDA. SISTEMA EM REPOUSO. ---");
}
