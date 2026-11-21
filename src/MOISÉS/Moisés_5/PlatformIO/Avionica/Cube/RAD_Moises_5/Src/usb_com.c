/*
  ******************************************************************************
  * @file    usb_com.c
  * @date    4 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Gerenciador de Comunicacao USB CDC (Estacao Solo)
  ******************************************************************************
*/

/* ==============================================================================
 * MANUAL DE OPERACAO USB
 * ==============================================================================
 *
 * DESCRICAO GERAL:
 * Este arquivo atua como o despachante de comandos (Command Dispatcher)
 * entre o foguete (STM32) e a Estacao Solo (PC). Ele intercepta
 * caracteres enviados via terminal serial e executa as rotinas de
 * leitura, gravacao ou simulacao correspondentes.
 *
 * ARQUITETURA NON-BLOCKING (SAFE IRQ):
 * Para evitar o travamento do microcontrolador durante a comunicacao,
 * este modulo utiliza uma arquitetura baseada em FLAG:
 * A funcao 'interruptUSB' roda dentro da interrupcao de hardware,
 * apenas salva o caractere na variavel 'comando_pendente' e encerra
 * imediatamente. A execucao pesada (prints e flash) ocorre de forma
 * segura no laco principal atraves da 'processarComandosUSB()'.
 *
 * FUNCOES PRINCIPAIS:
 * - _write()             : Sobrescreve a syscall nativa do GCC,
 * redirecionando qualquer 'printf' para o USB.
 * Possui trava de seguranca (timeout) caso o
 * cabo seja desconectado no meio do voo.
 * - interruptUSB()       : Callback de recepcao (IRQ). Levanta a flag.
 * - processarComandosUSB(): Funcao de pooling. Roteia para o switch correto.
 *
* COMANDOS SUPORTADOS (COM 'USE_W25Q' DEFINIDO - MODO VOO):
 * [V] - Visualizar Todos: Imprime a tabela de telemetria completa.
 * [U] - Ultimo Log      : Imprime apenas o bloco mais recente do voo.
 * [I] - Idle / Parar    : Pausa a gravacao imediatamente (sem descarregar o buffer).
 * [R] - Retomar Gravacao: Retoma a gravacao se o sistema nao estiver paralisado.
 * [A] - Apagar Logs     : Limpeza Inteligente (Smart Erase) dos blocos usados.
 * [$] - Apagar TUDO     : Formata o chip W25Q inteiro (Processo Critico).
 * [S] - Simular Ao Vivo : Executa o SITL e printa a fisica no terminal (10Hz).
 * [M] - Mock de Memoria : Gera um voo falso e salva fisicamente na W25Q (100Hz).
 * [E] - Forcar Erro     : Aborta o voo e forca o sistema para o Estado de Erro.
 *
 * COMANDOS SUPORTADOS (FLASH NATIVA DO STM32 - MODO BANCADA):
 * [L] - Log Teste (UP)  : Grava dado ficticio de subida e liga LED PC13.
 * [D] - Log Teste (DOWN): Grava dado ficticio de descida e apaga LED PC13.
 * [R] - Read (Ler)      : Imprime todos os logs armazenados na flash interna.
 * [C] - Clear (Apagar)  : Apaga a pagina alocada da flash interna.
 * ==============================================================================
 */

#include <usb_com.h>
#include <config_voo.h>

#ifdef USE_W25Q
	#pragma message("USE_W25Q foi definido.")
    #include <w25q.h>
    static void chamarComandos(u8 comando);
#else
	#pragma message("USE_W25Q não foi definido, utilizando a flash nativa.")
    #include <flash_stm.h>
    static void chamarComandosFlashSTM(u8 comando);  // Para uso direto da Flash do STM32.
#endif



// Variável global que serve como um "aviso" para o laço principal
volatile char comando_pendente = '\0';
extern USBD_HandleTypeDef hUsbDeviceFS;

// O novo _write sem HAL_Delay. Ele aguarda o USB liberar, mas tem trava de segurança.
int _write(int file, char *ptr, int len){
    // Fica tentando enviar enquanto o barramento USB estiver ocupado (USBD_BUSY)
    while(CDC_Transmit_FS((u8*)ptr, len) == USBD_BUSY) {
        // Trava de segurança: se o cabo for puxado, ele sai do loop e não trava o micro
        if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) break;
    }
    return (len);
}

void interruptUSB(u8* Buf, u32 *Len){
    if (Buf[0] != '\0') {
        // 1. O ECO: Envia a letra que chegou de volta para o PC para você poder enxergar
        //CDC_Transmit_FS(Buf, *Len);

        // 2. A FLAG: Salva o comando e sai da interrupção imediatamente!
        comando_pendente = Buf[0];
    }
    return;
}

// Esta é a nova função que vai rodar no main.c, fora do perigo das interrupções.
void processarComandosUSB(void) {
    if (comando_pendente != '\0') {

        //chamarComandosFlashSTM(comando_pendente); // Para uso direto da Flash do STM32.
    	chamarComandos(comando_pendente);

        // Limpa a flag para mostrar que já executou
        comando_pendente = '\0';
    }
}


#ifdef USE_W25Q

static void chamarComandos(u8 comando){

	switch(comando){

		case 'V': // Visualizar/Printar Tudo o que já tem no flash.
		case 'v':
			// ==========================================================
			// INTERLOCK DE SEGURANÇA MÁXIMA (BARREIRA TOTAL)
			// ==========================================================
			// Bloqueia qualquer I/O pesado se o foguete estiver armado, subindo ou caindo!
			if (dadosVoo.estadoAtual == ESTADO_PRONTO || \
				dadosVoo.estadoAtual == ESTADO_EM_VOO || \
				dadosVoo.estadoAtual == ESTADO_APOGEU || \
				dadosVoo.estadoAtual == ESTADO_RECUPERACAO) {

				printlnLRed("\r\n>>> COMANDO NEGADO: FOGUETE ATIVO! <<<");
				printlnLYellow(">>> Aguarde o Pouso ou force um Erro para acessar a memoria. <<<");
				break; // Ejeta do switch e não deixa o comando rodar
			}

			printlnLBlue("\r\nLigando LED do PC13!\r\n");
			acenderLedPlaca(); // Liga PC13

			printlnLGreen("\r\nComando { %c } - Visualizar Todos os Logs do W25Q.\r\n", comando);
			visualizarLogsW25Q();

			break;
		case 'U':// Printa somente o último Log
		case 'u':
			// ==========================================================
			// INTERLOCK DE SEGURANÇA MÁXIMA (BARREIRA TOTAL)
			// ==========================================================
			// Bloqueia qualquer I/O pesado se o foguete estiver armado, subindo ou caindo!
			if (dadosVoo.estadoAtual == ESTADO_PRONTO || \
				dadosVoo.estadoAtual == ESTADO_EM_VOO || \
				dadosVoo.estadoAtual == ESTADO_APOGEU || \
				dadosVoo.estadoAtual == ESTADO_RECUPERACAO) {

				printlnLRed("\r\n>>> COMANDO NEGADO: FOGUETE ATIVO! <<<");
				printlnLYellow(">>> Aguarde o Pouso ou force um Erro para acessar a memoria. <<<");
				break; // Ejeta do switch e não deixa o comando rodar
			}

			printlnLBlue("\r\nLigando LED do PC13!\r\n");
			acenderLedPlaca(); // Liga PC13

			printlnLCyan("\r\nComando { %c } - Visualizar Último Log do W25Q.\r\n", comando);
			visualizarUltimoLogW25Q();

			break;
		case 'I': // Idle/Parar
		case 'i':
			printlnLBlue("\r\nDesligando LED do PC13!\r\n");
			apagarLedPlaca(); // Liga PC13

			printlnLRed("\r\nComando { %c } - Parar Gravação do W25Q.\r\n", comando);
			//pararGravacaoW25Q();
			flagGravacaoParada = 1; // Levanta a bandeira de bloqueio

			printlnLYellow("\r\n>>> GRAVACAO MANUALMENTE PAUSADA <<<");

			break;
		case 'R': // Retomar Gravação
		case 'r':
			// Só deixa retomar se o foguete NÃO estiver nos estados finais
		    if (dadosVoo.estadoAtual != ESTADO_POUSADO && dadosVoo.estadoAtual != ESTADO_ERRO) {
		    	flagGravacaoParada = 0; // Abaixa a bandeira de bloqueio
		        printlnLGreen("\r\n>>> GRAVACAO RETOMADA COM SUCESSO <<<");
		    } else {
		        printlnLRed("\r\n>>> RECUSADO: Foguete ja pousou ou esta em falha critica! <<<");
		    }
		    break;
		case 'A': // Apagar só a memória até onde os logs foram escritos.
		case 'a':
			printlnLYellow("\r\nDesligando LED do PC13!\r\n");
			apagarLedPlaca(); // Desliga PC13

			printlnLRed("\r\nComando { %c } - Apagar Todos os Logs do W25Q.\r\n", comando);
			apagarLogsW25Q();

			dadosVoo.estadoAtual = ESTADO_CALIBRACAO;

			// Zera as variáveis cruciais de segurança
			seguroVoo.altitude_maxima = 0.0f;
			seguroVoo.tempo_inicio_ms = 0;
			flagFimDeVoo = 0;           // Libera para um novo voo
			flagGravacaoParada = 0;     // Garante que a gravação está pronta pra iniciar

			printlnLYellow(">>> SISTEMA REINICIADO. RECALIBRANDO SENSOR NO SOLO... <<<");
			break;
		case '$': // (CUIDADO!) Apagar absolutamente toda o W25Q.
			printlnLYellow("\r\nDesligando LED do PC13!\r\n");
			apagarLedPlaca(); // Desliga PC13

			printlnLRed("\r\nComando { %c } - Apagar Todo o W25Q.\r\n", comando);
			printlnLYellow("Apagando memoria, aguarde 40s...");
			apagarTudoW25Q();

			dadosVoo.estadoAtual = ESTADO_CALIBRACAO;

			// Zera as variáveis cruciais de segurança
			seguroVoo.altitude_maxima = 0.0f;
			seguroVoo.tempo_inicio_ms = 0;
			flagFimDeVoo = 0;           // Libera para um novo voo
			flagGravacaoParada = 0;     // Garante que a gravação está pronta pra iniciar

			printlnLYellow(">>> SISTEMA REINICIADO. RECALIBRANDO SENSOR NO SOLO... <<<");
			break;
		case 'S': // Simular voo ao vivo (SITL) sem gravar na flash
		case 's':
			printlnLBlue("\r\nLigando LED do PC13!\r\n");
			acenderLedPlaca();
			printlnLGreen("\r\nComando { %c } - Simular Voo Ao Vivo (SITL) Sem Gravar Na Flash.\r\n", comando);
			simularVooAoVivoUSB();
			apagarLedPlaca();
			break;
		case 'M': // Gerar voo mock na memoria W25Q sem usar sensor MS5611
		case 'm':
			printlnLBlue("\r\nLigando LED do PC13!\r\n");
			acenderLedPlaca();
			printlnLGreen("\r\nComando { %c } - Gerar Voo Simulado Com W25Q.\r\n", comando);
			gerarVooSimuladoW25Q();
			apagarLedPlaca();
			break;
		case 'E':
		case 'e':
			printlnLYellow("\r\nDesligando LED do PC13!\r\n");
			apagarLedPlaca();
			printlnLMagenta("\r\nComando { %c } - Forcando Erro.\r\n", comando);
			dadosVoo.estadoAtual = ESTADO_ERRO;
			break;
		default:
			printlnLRed("\r\nComando desconhecido: { %c }.\r\n", comando);
			break;

	}
	return;
}

#else

static void chamarComandosFlashSTM(u8 comando){ // A lógica só tem teste, já que não será utilizada em voo.
	static DadosVoo_t dados = {0};

	switch(comando){

		case 'L':
			printlnLBlue("\r\nVocê mandou L - Ligando LED do PC13!\r\n");
			ligarLedPlaca();
			dados = (DadosVoo_t){0, 1000.0, 2000.0, 1000.0, 2000.0, 3};
			salvarDado(&dados);
			break;
		case 'D':
			printlnLYellow("\r\nVocê mandou D - Desligando LED do PC13!\r\n");
			apagarLedPlaca();
			dados = (DadosVoo_t){0, -1000.0, -2000.0, -1000.0, -2000.0, 1};
			salvarDado(&dados);
			break;
		case 'R':
			printFlash();
			break;
		case 'C':
			printlnYellow("\r\nVocê mandou C - Apagar Memória!\r\n");
			apagarCaixaPreta();
			break;
		default:
			printlnLRed("\r\nComando desconhecido: %c\r\n", comando);
			break;

	}
	return;
}

#endif
