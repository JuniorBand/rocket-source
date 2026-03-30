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
#include <prints.h>
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
u8 flagGravacaoParada = 0;
u8 flagFimDeVoo = 0;

#ifdef VERBOSE
	static u32 time_verbose = 0;
#endif


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

	// INICIA O METRÔNOMO DE 10ms
	HAL_TIM_Base_Start_IT(htim_ms);

	CaixaPreta_t rec = lerCaixaPretaW25Q();

	if (rec.status_voo == MAGIC_NUMBER_SIRIUS) {
			// --- MODO RECUPERACAO (REINICIO NO AR) ---
			printLYellow("\r\n>>> AVISO: REINICIO EM VOO DETECTADO! RECUPERANDO DADOS... <<<");

			sensor.pressao_ref = rec.pressao_solo;

			// 1. DRENO DE LIXO (Warm-up do Hardware)
			// Roda a máquina de estados do sensor 10 vezes e joga os dados no lixo.
			// Isso purga o barramento SPI de ruídos elétricos gerados pelo religamento da energia.
			for (int i = 0; i < 10; i++) {
				MS5611_ReadData();
				HAL_Delay(10);
			}

			// 2. Extrai a primeira leitura termicamente estável
			if (sensor.pressao > 300.0f && sensor.pressao < 1200.0f) {
				sensor.altitude = 44330.0f * (1.0f - powf((sensor.pressao / sensor.pressao_ref), 0.190295f));
			} else {
				sensor.altitude = 0.0f; // Blindagem final caso o sensor tenha torrado fisicamente
			}

			// 3. Choque de Realidade no Kalman (Limpo)
			dadosVoo.velocidadeAtual = 0.0f;
			filtroKalman.reiniciar(&filtroKalman, &(sensor.altitude), &(dadosVoo.velocidadeAtual));

			seguroVoo.altitude_maxima = sensor.altitude;
			dadosVoo.estadoAtual = ESTADO_EM_VOO;
			seguroVoo.tempo_inicio_ms = HAL_GetTick();

			beep(1000, 1);
			HAL_Delay(100);
			beep(50, 3);
		} else {

		printlnLCyan("\r\n>>> CALIBRANDO SENSOR DE PRESSAO NO SOLO... <<<");

	}

	apagarLedPlaca();
}

void processarLogicaVoo(void) {
	if (flagTickVoo == 1) {

		flagTickVoo = 0; // Abaixa a bandeira imediatamente

		static u32 timer_estabilizacao_kalman = 0;

		// Agora sim, executa a lógica pesada de sensores e flash com tempo garantido
		#ifndef EM_VOO
			#pragma message("Utilizando conexão via USB.")
			processarComandosUSB();
		#endif

		MS5611_ReadData();

		if (sensor.pressao > 300.0f && sensor.pressao < 1200.0f && sensor.temperatura > -40.0f && sensor.temperatura < 85.0f) {
			filtroKalman.update(&filtroKalman, sensor.altitude, DT, &(dadosVoo.altitudeAtual), &(dadosVoo.velocidadeAtual));
			dadosVoo.temperaturaAtual = sensor.temperatura;
			dadosVoo.pressaoAtual = sensor.pressao;
		}

		#ifdef VERBOSE
			if(HAL_GetTick() - time_verbose >= 1000){
				printlnLCyan("\r\nDADOS FILTRADOS: P: %.2f; T: %.2f; A: %.2f; V: %.2f; E: %s", dadosVoo.pressaoAtual, \
						dadosVoo.temperaturaAtual, dadosVoo.altitudeAtual, dadosVoo.velocidadeAtual, PRINT_ESTADO[dadosVoo.estadoAtual]);
				time_verbose = HAL_GetTick();
			}
		#endif

		verificarErros();

		switch (dadosVoo.estadoAtual) {

			case ESTADO_CALIBRACAO:
				{
						static u16 ciclos_aquecimento = 0;
						static double soma_pressao_ref = 0.0f; // Usando double para não perder precisão decimal!
						static u16 amostras_validas = 0;

						// Fase 1: Ignora os primeiros 200 ticks (2 SEGUNDOS INTEIROS) pro chip esquentar
						if (ciclos_aquecimento < 200) {
							ciclos_aquecimento++;
						}
						// Fase 2: Coleta 100 amostras (mais 1 segundo tirando a média perfeita)
						else if (amostras_validas < 100) {
							// MURALHA DE FERRO DA CALIBRAÇÃO:
							// Só aceita somar na média se a pressão for da Terra (entre 300 e 1200)
							if (sensor.pressao > 300.0f && sensor.pressao < 1200.0f) {
								soma_pressao_ref += (double)sensor.pressao;
								amostras_validas++;
							}
							// Se vier glitch > 1200, ele simplesmente ignora o loop e tenta de novo 10ms depois
						}
						// Fase 3: Calibração finalizada!
						else {
							// Calcula a média com precisão máxima e converte de volta pra float
							sensor.pressao_ref = (float)(soma_pressao_ref / 100.0);

							// Força o cálculo da altitude agora para cravar o 0.00m
							sensor.altitude = 44330.0f * (1.0f - powf((sensor.pressao / sensor.pressao_ref), 0.190295f));

							sensor.altitude = 0.0f;
							dadosVoo.velocidadeAtual = 0.0f; // MATA A VELOCIDADE FANTASMA AQUI!

							// ESSENCIAL: O Filtro de Kalman acumulou lixo durante esses 3 segundos.
							// Avisamos ele: "Apaga tudo! A altitude oficial agora é 0.00m e estamos parados".
							filtroKalman.reiniciar(&filtroKalman, &(sensor.altitude), &(dadosVoo.velocidadeAtual));

							printlnLCyan("\r\n>>> CALIBRACAO CONCLUIDA! Pref: %.2f hPa <<<", sensor.pressao_ref);
							beep(500, 1);

							dadosVoo.estadoAtual = ESTADO_PRONTO;
							registrarLogVoo();

							ciclos_aquecimento = 0;
							soma_pressao_ref = 0.0f;
							amostras_validas = 0;
							timer_estabilizacao_kalman = 0;
						}
						break;
					}

			case ESTADO_PRONTO:
				{
					// Espera 1 segundo (100 ticks) APÓS a calibração pro Kalman "acalmar" os cálculos
					// de velocidade antes de autorizar o lançamento.
					if (timer_estabilizacao_kalman < 100) {
						timer_estabilizacao_kalman++;
						break; // Sai do switch e não tenta decolar
					}

					registrarLogVoo();

					// Regras de Lançamento (Blindadas)
					// Nota: Em testes de bancada, exija uma velocidade de subida "real"
					// (ex: > 1.5 m/s) para evitar que o ruído ative o voo.
					if ((dadosVoo.altitudeAtual > ALTITUDE_LANCAMENTO) && (dadosVoo.velocidadeAtual > VELOCIDADE_LANCAMENTO)) {
						// DETECTOU DECOLAGEM!
						dadosVoo.estadoAtual = ESTADO_EM_VOO;
						seguroVoo.tempo_inicio_ms = HAL_GetTick();

						// NUNCA reinicie o Kalman AQUI. Se você acabou de decolar,
						// reiniciar o Kalman apaga o vetor de velocidade e fode a estimativa.
						// filtroKalman.reiniciar(...); // <<-- REMOVA ISSO SE TIVER

						// Agora sim, começa a gravar
						registrarLogVoo();
						salvarCaixaPretaW25Q(sensor.pressao_ref, MAGIC_NUMBER_SIRIUS);

						printlnLCyan("\r\n>>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_EM_VOO]);
						beep(200, 1);
					}
					break;
				}
			case ESTADO_EM_VOO:
				{
					// Atualiza altitude máxima
					if (dadosVoo.altitudeAtual > seguroVoo.altitude_maxima) {
						seguroVoo.altitude_maxima = dadosVoo.altitudeAtual;
						#ifdef VERBOSE
							printlnGreen("RAW: %.2f | KALMAN: %.2f | VEL: %.2f", sensor.altitude, dadosVoo.altitudeAtual, dadosVoo.velocidadeAtual);
						#endif
					}

					// Verifica falha de tempo primeiro (prioridade máxima de erro)
					if (HAL_GetTick() - seguroVoo.tempo_inicio_ms > TEMPO_MAX_VOO_MS) {
						dadosVoo.estadoAtual = ESTADO_ERRO;
						registrarLogVoo();
						acionarEjecao("\r\nTIMEOUT DE VOO - APOGEU NAO DETECTADO");
					}
					// =========================================================================
					// CORREÇÃO: ALTITUDE LOCKOUT (TRAVA DE APOGEU NA BANCADA)
					// Só permite acionar apogeu se já passamos de 15 metros na vida real!
					// =========================================================================
					else if ( (dadosVoo.altitudeAtual >= (META_APOGEU - DESVIO_MIN)) ||
								((dadosVoo.altitudeAtual < (seguroVoo.altitude_maxima - DESCIDA_MINIMA))
								&& (seguroVoo.altitude_maxima > ALTITUDE_MIN_EJECAO))
							 ) {
						dadosVoo.estadoAtual = ESTADO_APOGEU;
					}

					registrarLogVoo();
					break;
				}
			case ESTADO_APOGEU:
				{
					registrarLogVoo();
					printlnLYellow("\r\n>>> %s CONFIRMADO <<<", PRINT_ESTADO[ESTADO_APOGEU]);
					acionarEjecao(PRINT_ESTADO[ESTADO_APOGEU]);
					dadosVoo.estadoAtual = ESTADO_RECUPERACAO;
					break;
				}
			case ESTADO_RECUPERACAO:
				{
					// Detecta Pouso (Baixa alt e Baixa dadosVoo.velocidadeAtual )
					if ((dadosVoo.altitudeAtual < ALTITUDE_POUSO) && ((dadosVoo.velocidadeAtual > -VELOCIDADE_POUSO) \
							&& (dadosVoo.velocidadeAtual < VELOCIDADE_POUSO))) {
						dadosVoo.estadoAtual = ESTADO_POUSADO;
					}
					else {
						registrarLogVoo();
					}
					break;
				}
			case ESTADO_POUSADO:
				{
					if (!flagFimDeVoo) {
						registrarLogVoo();
						printlnLGreen("\r\n>>> %s DETECTADO <<<", PRINT_ESTADO[ESTADO_POUSADO]);
						// -> CAIXA PRETA: Pousou seguro! Desativa a flag para não entrar em recuperação no próximo boot.
						salvarCaixaPretaW25Q(1013.25f, 0x00000000);
						pararGravacaoW25Q();
						flagGravacaoParada = 1;
						flagFimDeVoo = 1;
					}
					static u32 timerPouso = 0;
					resgatarFoguete(3000, 1000, &timerPouso);
					break;
				}
			case ESTADO_ERRO:
				{

					if (!flagFimDeVoo) {
						printlnRed("\r\n=================================================");
						printlnRed("           >>>   %s!   <<<", PRINT_ESTADO[ESTADO_ERRO]);
						printlnRed("           >>> SISTEMA PARALISADO <<<");
						printlnRed("=================================================");
						pararGravacaoW25Q();
						flagGravacaoParada = 1;
						flagFimDeVoo = 1;
					}
					static u32 timerErro = 0;
					resgatarFoguete(600, 300, &timerErro);
					break;
				}
			default: { break; }
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
		printlnRed("           >>> Altitude Maxima: %.2f m!   <<<", seguroVoo.altitude_maxima);
		printlnMagenta("=================================================");
		writePinHigh(MOSFET_PORT, MOSFET_PIN);
		return;
	}
#else
	#pragma message("MOSFET_PORT e MOSFET_PIN não foram definidos no main.h!")

	static void acionarEjecao(const char *razao_missao){
			printlnMagenta("\r\n=========================================================");
			printlnLCyan("[SIMULACAO] ACIONANDO EJECAO EM %.2f m. MOTIVO: %s", dadosVoo.altitudeAtual, razao_missao);
			printlnRed("           >>> Altitude Maxima: %.2f m!   <<<", seguroVoo.altitude_maxima);
			printlnMagenta("=========================================================");
			return;
	}
#endif

void beep(u32 duracao, u8 vezes) {
	for (u8 i = 0; i < vezes; i++) {
		BUZZER_OFF(); HAL_Delay(duracao);
		BUZZER_ON(); if (i < vezes - 1) { HAL_Delay(duracao); }
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
