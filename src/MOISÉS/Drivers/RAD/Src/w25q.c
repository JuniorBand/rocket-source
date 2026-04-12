/*
  ******************************************************************************
  * @file    w25q.c
  * @date    3 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Driver de armazenamento SPI para a memoria Flash W25Q128 (Caixa Preta)
  ******************************************************************************
*/

/* ==============================================================================
 * MANUAL DA ARQUITETURA DE MEMORIA (W25Q128)
 * ==============================================================================
 *
 * DESCRICAO GERAL:
 * Este arquivo eh o "Gravador de Voo" do foguete. Ele gerencia a gravacao e
 * leitura de alta velocidade na memoria externa SPI (16 Megabytes). Ele eh
 * responsavel por guardar o historico dinamico a 100Hz e recuperar variaveis
 * de estado criticas em caso de falta de energia (reboot em voo).
 *
 * ESTRATEGIA DE BUFFERING (DESGASTE E PERFORMANCE):
 * A memoria Flash leva tempo para ser gravada e tem limite de ciclos de vida.
 * Para resolver isso, utilizamos a tecnica de 'Page Buffer':
 * Os dados gerados pelo Filtro de Kalman nao vao direto para a Flash. Eles
 * enchem uma array na RAM (bufferRAM) struct por struct. Quando a array
 * atinge o tamanho de uma Pagina da W25Q (256 bytes), o laco dispara uma unica
 * rajada SPI (Page Program).
 *
 * ARQUITETURA DA CAIXA PRETA (SETOR ISOLADO):
 * O ultimo setor de 4KB do chip (Endereco 0xFFF000) esta isolado do resto
 * dos dados. Ele serve como uma EEPROM virtual. Variaveis criticas (como a
 * Pressao de Referencia no solo) sao gravadas la para que, se o foguete
 * reiniciar no ar, ele saiba a altitude correta em vez de achar que esta no chao.
 *
 * MECANISMOS DE PROTECAO (FAIL-SAFES):
 * 1. Limite de Chip : Se a memoria lotar, a gravacao eh travada e o voo continua.
 * 2. Timeout        : Todas as funcoes de 'espera' possuem limite de tempo. Se o
 * chip queimar no ar, o loop principal nao trava.
 * 3. Smart Erase    : A funcao de limpeza apaga SOMENTE os blocos sujos,
 * reduzindo o tempo de apagamento de 40s para < 1s.
 *
 * FUNCOES PRINCIPAIS:
 * - w25qInit()             : Sincroniza ponteiros e busca onde os ultimos dados estao.
 * - adicionarLogW25Q()     : Adiciona dado ao buffer e descarrega quando cheio.
 * - recuperarEnderecoW25Q(): Escaneamento rapido para achar o final dos dados.
 * - visualizarLogsW25Q()   : Le os dados brutos e gera a dashboard colorida no PC.
 * - gerarVooSimuladoW25Q() : Mock de Software-in-the-Loop (SITL).
 * ==============================================================================
 *
 * PENDENCIA FUTURA: Implementar extrair os dados diretamente em um .csv.
 */

#include <w25q.h>
#include <utils.h>
#include <config_voo.h>
#include <stdio.h>   // Para o snprintf não gerar warning
#include <string.h>  // Para o memset não gerar warning
#include <math.h>
#ifndef EM_VOO
	#include <usb_com.h>
#endif


#define W25Q_SECTOR_SIZE 4096 // 4 KB para apagar de uma vez (mínimo = tamanho de um setor do W25Q)
#define W25Q_BLOCK_SIZE 65536 // 64 KB para apagar de uma vez
#define W25Q_CAIXA_PRETA_ADDR 0xFFF000 // Último setor de 4KB do chip (16MB)
#define DELAY 5
#define PAGE_PROGRAM_MAX 256 // bytes
#define LOGS_PER_PAGE(nome_struct_t) (PAGE_PROGRAM_MAX/sizeof(nome_struct_t))   // Quantos logs podem ser gravados de uma vez em 256 bytes (1 P�gina da Flash)
#define TIME_OUT 500

// ============================================================================
// INSTRUCOES DO CHIP W25Q (Datasheet)
// ============================================================================
#define CMD_WRITE_ENABLE      0x06
#define CMD_READ_STATUS_1     0x05
#define CMD_READ_DATA         0x03
#define CMD_PAGE_PROGRAM      0x02
#define CMD_SECTOR_ERASE_4K   0x20
#define CMD_BLOCK_ERASE_64K   0xD8
#define CMD_CHIP_ERASE        0xC7


static SPI_HandleTypeDef* w25q_spi;
static TIM_HandleTypeDef* w25q_timer;
W25Q_State_t sensor_state = IDLE;
u32 currentAddr = 0;
static LogData_t bufferRAM[LOGS_PER_PAGE(LogData_t)] = {0};
static u32 bufferIndex = 0;
static GPIO_TypeDef *CS_PORT = 0;
static u16 CS_PIN = 0;


static void w25qWritePage(void);
static void w25q_WriteEnable(void);
static void recuperarEnderecoW25Q(void);
static void w25q_WaitForWriteEnd(u32 timeout_ms);
static void w25q_EraseBlock64K(u32 BlockAddr);
//static void w25q_EraseSector(u32 SectorAddr);




static inline void w25q_CS_LOW(void) { writePinLow(CS_PORT, CS_PIN); }
static inline void w25q_CS_HIGH(void) { writePinHigh(CS_PORT, CS_PIN); }

void w25qInit(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim,  GPIO_TypeDef *port, u16 pin_mask){
	w25q_spi = hspi;
	w25q_timer = htim;
	CS_PORT = port;
	CS_PIN = pin_mask;

	printlnLCyan("\r\n--- VERIFICANDO INTEGRIDADE DO HARDWARE (W25Q) ---");

	// =========================================================
	// O POLÍGRAFO PERMANENTE (Sanity Check de Hardware)
	// =========================================================
	u8 id_cmd = 0x9F; // Comando JEDEC ID
	u8 id_res[3] = {0, 0, 0};

	// Pequeno delay pro chip acordar junto com a placa
	HAL_Delay(10);

	w25q_CS_LOW();
	HAL_SPI_Transmit(w25q_spi, &id_cmd, 1, TIME_OUT);
	HAL_SPI_Receive(w25q_spi, id_res, 3, TIME_OUT);
	w25q_CS_HIGH();

	// Se voltar tudo Zero (Cabo solto) ou tudo F (Pino em curto com VCC)
	if ((id_res[0] == 0x00 && id_res[1] == 0x00) || (id_res[0] == 0xFF && id_res[1] == 0xFF)) {
		printlnRed(">>> ERRO FATAL: MEMORIA W25Q NAO RESPONDE! (ID: %02X %02X %02X) <<<", id_res[0], id_res[1], id_res[2]);
		printlnRed(">>> VERIFIQUE A FIACAO SPI (CS, MISO, MOSI) <<<");

		// Opcional: Você pode forçar o estado do foguete para ERRO aqui se quiser travar o boot!
		// dadosVoo.estadoAtual = ESTADO_ERRO;
	} else {
		printlnLGreen(">>> W25Q ONLINE E SAUDAVEL! (ID: %02X %02X %02X) <<<", id_res[0], id_res[1], id_res[2]);

		// Se o hardware está vivo, aí sim deixamos ele escanear a memória
		recuperarEnderecoW25Q();
	}
}

void visualizarLogsW25Q(void) {
	// PRECAUÇÃO: Descarrega o buffer da RAM para a Flash física
	// antes de tentar ler a tabela toda!
	if (bufferIndex > 0) {
		descarregarBuffer();
	}

    LogData_t leitura;
    u32 printAddr = 0;
    char buffer_saida[256];
    char barra_grafica[55]; // <-- Aumentado para suportar 50 barras

    // Banner Inicial 100% simetrico (118 caracteres de largura exatos)
    printlnLCyan("\r\n======================================================================================================================");
    printlnLYellow("                                     RELATORIO DE VOO - VISUALIZADOR (v5.0)                                     ");
    printlnLCyan("======================================================================================================================");
    printlnLMagenta("   HORA   |        TELEMETRIA (A/V/P/T)           |     STATUS     | GRAFICO (# = 10m)                                ");
    printlnLCyan("----------|---------------------------------------|----------------|--------------------------------------------------");

    while (printAddr < LIMIT) {
    	// PRECAUÇÃO: Proteção contra cabo solto ou Timeout SPI
    	memset(&leitura, 0xFF, sizeof(LogData_t));

        // Leitura Fisica da Flash W25Q
        w25q_CS_LOW();
        u8 cmd[4] = { CMD_READ_DATA, (printAddr >> 16) & 0xFF, (printAddr >> 8) & 0xFF, printAddr & 0xFF };
        HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
        HAL_SPI_Receive(w25q_spi, (u8*)&leitura, sizeof(LogData_t), 20);
        w25q_CS_HIGH();

        if (leitura.hora == 0xFF) { break; } // Encontrou memoria vazia, fim.

        // Gera Barra Grafica (#) proporcional a altitude (usando int para aceitar numeros negativos momentaneos)
        int num_barras = (int)(leitura.altitude / 10.0f);
        if (num_barras > 50) num_barras = 50;
        if (num_barras < 0) num_barras = 0;

        memset(barra_grafica, 0, sizeof(barra_grafica));
        for (int i = 0; i < num_barras; i++) barra_grafica[i] = '#';

        // Protecao de indice do label
        u8 st = (u8)leitura.estado;
        if (st > 6) st = 6;

        // MAGICA DO ALINHAMENTO EXATO (Usando os dados reais da 'leitura')
        snprintf(buffer_saida, sizeof(buffer_saida),
            "%s %02d:%02d:%02d %s|"                               // Col 1: 10 chars
            "%s A:%7.1f V:%7.1f P:%6.1f T:%6.1f %s|"              // Col 2: 39 chars
            "%s %-14s %s|"                                        // Col 3: 16 chars
            "%s %s",                                              // Col 4: Barras
            RESET, leitura.hora, leitura.min, leitura.seg, LMAGENTA,
            LCYAN, leitura.altitude, leitura.velocidade, leitura.pressao, leitura.temperatura, LMAGENTA,
            LYELLOW, PRINT_ESTADO[st], LMAGENTA,
            LGREEN, barra_grafica);

        printf("%s%s\r\n", buffer_saida, RESET);

        HAL_Delay(DELAY);

        printAddr += sizeof(LogData_t);
    }

    u32 tempo_voo = 0; // Começa zerado por padrão

	// Só calcula o tempo real se o foguete já tiver saído da bancada
	if ((dadosVoo.estadoAtual >= ESTADO_EM_VOO) && (dadosVoo.estadoAtual != ESTADO_ERRO)) {
		tempo_voo = ((HAL_GetTick() - seguroVoo.tempo_inicio_ms) / 1000);
	}

    // Rodape Estatistico ajustado para a nova largura de 118 caracteres
    printlnMagenta("======================================================================================================================");
    snprintf(buffer_saida, sizeof(buffer_saida),
            "ESTATISTICAS FINAIS:\r\n- Altitude Maxima: %.2f m; Tempo de Voo: %lu s; Runtime: %lu s.",
            seguroVoo.altitude_maxima, tempo_voo, (HAL_GetTick()/1000));
    printlnLGreen("%s", buffer_saida);
    printlnMagenta("======================================================================================================================");
}


void visualizarUltimoLogW25Q(void){
    LogData_t leitura;
    char buffer_saida[256];
    char barra_grafica[55];

    // Banner Inicial 100% simetrico
    printlnLCyan("\r\n======================================================================================================================");
    printlnLYellow("                                     RELATORIO DE VOO - VISUALIZADOR (v5.0)                                     ");
    printlnLCyan("======================================================================================================================");
    printlnLMagenta("   HORA   |        TELEMETRIA (A/V/P/T)           |     STATUS     | GRAFICO (# = 10m)                                ");
    printlnLCyan("----------|---------------------------------------|----------------|--------------------------------------------------");

    // 1. O dado mais recente está na RAM? Puxa de lá!
    if (bufferIndex > 0) {
        leitura = bufferRAM[bufferIndex - 1]; // Pega a última struct preenchida
    }
    // 2. A RAM está vazia, mas a Flash tem dados? Puxa da Flash!
    else if (currentAddr > 0 && currentAddr < LIMIT) {

        // Se bufferIndex é zero, o último dado gravado está exatamente 1 struct ATRÁS do currentAddr!
        u32 printAddr = currentAddr - sizeof(LogData_t);


        // PRECAUÇÃO: Proteção contra cabo solto ou Timeout SPI
		memset(&leitura, 0xFF, sizeof(LogData_t));

        // Leitura Fisica da Flash W25Q
        w25q_CS_LOW();
        u8 cmd[4] = { CMD_READ_DATA, (printAddr >> 16) & 0xFF, (printAddr >> 8) & 0xFF, printAddr & 0xFF };
        HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
        HAL_SPI_Receive(w25q_spi, (u8*)&leitura, sizeof(LogData_t), 20);
        w25q_CS_HIGH();

        if (leitura.hora == 0xFF) {
            printlnRed("Nenhum dado salvo na Flash (Leitura Vazia)!");
            return;
        }
    }
    // 3. Tudo zerado
    else {
        printlnRed("\r\nNenhum dado salvo na Flash ou na RAM!\r\n");
        return;
    }

    // ========================================================================
    // TUDO QUE FICA AQUI FORA DOS IFs VAI SER EXECUTADO PARA A RAM E PRA FLASH
    // ========================================================================

    int num_barras = (int)(leitura.altitude / 10.0f);
    if (num_barras > 50) num_barras = 50;
    if (num_barras < 0) num_barras = 0;

    memset(barra_grafica, 0, sizeof(barra_grafica));
    for (int i = 0; i < num_barras; i++) barra_grafica[i] = '#';

    u8 st = (u8)leitura.estado;
    if (st > 6) st = 6;

    snprintf(buffer_saida, sizeof(buffer_saida),
        "%s %02d:%02d:%02d %s|"
        "%s A:%7.1f V:%7.1f P:%6.1f T:%6.1f %s|"
        "%s %-14s %s|"
        "%s %s",
        RESET, leitura.hora, leitura.min, leitura.seg, LMAGENTA,
        LCYAN, leitura.altitude, leitura.velocidade, leitura.pressao, leitura.temperatura, LMAGENTA,
        LYELLOW, PRINT_ESTADO[st], LMAGENTA,
        LGREEN, barra_grafica);

    printf("%s%s\r\n", buffer_saida, RESET);

    u32 tempo_voo = 0; // Começa zerado por padrão

	// Só calcula o tempo real se o foguete já tiver saído da bancada
	if ((dadosVoo.estadoAtual >= ESTADO_EM_VOO) && (dadosVoo.estadoAtual != ESTADO_ERRO)) {
		tempo_voo = ((HAL_GetTick() - seguroVoo.tempo_inicio_ms) / 1000);
	}


    printlnMagenta("======================================================================================================================");
    snprintf(buffer_saida, sizeof(buffer_saida),
            "ESTATISTICAS FINAIS:\r\n- Altitude Maxima: %.2f m; Tempo de Voo: %lu s; Runtime: %lu s.",
            seguroVoo.altitude_maxima, tempo_voo, (HAL_GetTick()/1000));
    printlnLGreen("%s", buffer_saida);
    printlnMagenta("======================================================================================================================");
}

// ============================================================================
// 3. STORAGE & SENSOR DRIVERS
// ============================================================================

// Adiciona log ao Buffer RAM e descarrega na Flash quando cheio
void adicionarLogW25Q(RTC_HandleTypeDef* hrtc_log, DadosVoo_t *dadosVoo) {

	// TRAVA 1 (Economia de CPU): Se a memoria ja esta cheia, sai imediatamente.
	// Nao gasta tempo calculando RTC nem enchendo buffer RAM a toa.
	if (currentAddr >= LIMIT) {
		return;
	}

	RTC_TimeTypeDef sTime = {0};
    HAL_RTC_GetTime(hrtc_log, &sTime, RTC_FORMAT_BIN);
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetDate(hrtc_log, &sDate, RTC_FORMAT_BIN); // Leitura dummy necess�ria

    bufferRAM[bufferIndex].hora = sTime.Hours;
    bufferRAM[bufferIndex].min = sTime.Minutes;
    bufferRAM[bufferIndex].seg = sTime.Seconds;
    bufferRAM[bufferIndex].pressao = dadosVoo->pressaoAtual;
    bufferRAM[bufferIndex].temperatura = dadosVoo->temperaturaAtual;
    bufferRAM[bufferIndex].altitude = dadosVoo->altitudeAtual;
    bufferRAM[bufferIndex].velocidade = dadosVoo->velocidadeAtual;
    bufferRAM[bufferIndex].estado = (u8)(dadosVoo->estadoAtual);

    bufferIndex++;

    // Se encheu a p�gina (32 registros), grava na Flash
    if (bufferIndex >= LOGS_PER_PAGE(LogData_t)) {
    	w25qWritePage();
    	bufferIndex = 0; // Reseta buffer
    }
}



static void w25qWritePage(void) {

	// TRAVA 2 (Protecao Fisica): Garante que os bytes que vamos somar agora
	// nao vao estourar o limite maximo do chip durante a escrita SPI.
	if ((currentAddr + (bufferIndex * sizeof(LogData_t))) > LIMIT) {
		return;
	}

	w25q_WriteEnable();

    w25q_CS_LOW();
    // Comando 0x02 = Page Program
    u8 cmd[4] = { CMD_PAGE_PROGRAM, (currentAddr >> 16) & 0xFF, (currentAddr >> 8) & 0xFF, currentAddr & 0xFF };
    HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
    HAL_SPI_Transmit(w25q_spi, (u8*)bufferRAM, bufferIndex * sizeof(LogData_t), TIME_OUT);
    w25q_CS_HIGH();

    currentAddr += (bufferIndex * sizeof(LogData_t));
    w25q_WaitForWriteEnd(20);
}


void descarregarBuffer(void) {
    if (bufferIndex > 0) {
        // 1. Resgata o último log real e válido que o foguete gerou
        LogData_t ultimo_log_valido = bufferRAM[bufferIndex - 1];

        // 2. Preenche todos os "espaços vazios" para não gravar Zeros
        for (u32 i = bufferIndex; i < LOGS_PER_PAGE(LogData_t); i++) {
            bufferRAM[i] = ultimo_log_valido;
        }

        // 3. O PULO DO GATO: Finge que o buffer tem 8 logs para que
        // a 'w25qWritePage' envie exatos 256 bytes e não quebre o alinhamento!
        bufferIndex = LOGS_PER_PAGE(LogData_t);

        // 4. Grava na Flash
        w25qWritePage();

        bufferIndex = 0; // Reseta para o próximo ciclo
    }
}


void apagarLogsW25Q(void) {
    char buffer_msg[64];
    printlnLYellow("\r\n--- INICIANDO LIMPEZA DOS LOGS DE VOO (SMART ERASE) ---");


    // 2. Descobre o número do bloco atual para apagar tudo, incluindo ele.
    // Divisão inteira (shift) diz em qual bloco estamos. Multiplicar + 1 dá o limite cravado.
    u32 num_bloco_atual = currentAddr / W25Q_BLOCK_SIZE;
    u32 limite_apagamento = (num_bloco_atual + 1) * W25Q_BLOCK_SIZE;

    // 3. Trava de seguranca para nunca apagar a memoria toda sem querer
    if (limite_apagamento > LIMIT) {
        limite_apagamento = LIMIT;
    }

    // 4. Apaga APENAS os blocos que tem lixo
    for (u32 addr = 0; addr < limite_apagamento; addr += W25Q_BLOCK_SIZE) {

        w25q_EraseBlock64K(addr);

        // Printa o progresso para acalmar quem esta olhando a tela
        snprintf(buffer_msg, sizeof(buffer_msg), "Apagando... Endereco: 0x%06lX\r\n", addr);
        printLYellow("%s", buffer_msg);
    }

    // 5. Zera o ponteiro global e limpa o buffer
    currentAddr = 0;
    bufferIndex = 0; // Garante que a RAM não vai vazar lixo pro próximo voo

    printlnLGreen("--- LIMPEZA INTELIGENTE CONCLUIDA! PRONTO PARA VOO ---\r\n");
}

static void w25q_EraseBlock64K(u32 BlockAddr) {
    w25q_WriteEnable();

    w25q_CS_LOW();
    u8 cmd[4];
    cmd[0] = CMD_BLOCK_ERASE_64K; // Instrução Block Erase (64 KB)
    cmd[1] = (BlockAddr >> 16) & 0xFF;
    cmd[2] = (BlockAddr >> 8) & 0xFF;
    cmd[3] = BlockAddr & 0xFF;

    HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
    w25q_CS_HIGH();

    w25q_WaitForWriteEnd(3000); // O chip demora de 150ms a 200ms para apagar o bloco
}

/*
static void w25q_EraseSector(u32 SectorAddr) {  // Apaga todo um setor do W25Q
    w25q_WriteEnable(); // Sempre chamar antes!

    w25q_CS_LOW();
    u8 cmd[4];
    cmd[0] = 0x20; // Instrução Sector Erase
    cmd[1] = (SectorAddr >> 16) & 0xFF;
    cmd[2] = (SectorAddr >> 8) & 0xFF;
    cmd[3] = SectorAddr & 0xFF;

    HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
    w25q_CS_HIGH();

    w25q_WaitForWriteEnd(); // Espera os ~45ms para o chip limpar o setor
}
*/

void apagarTudoW25Q(void){ // Apaga todo o W25Q.
	w25q_WriteEnable(); // Sempre chamar antes!

	w25q_CS_LOW();
	u8 cmd = CMD_CHIP_ERASE; // Instrução Chip Erase
	HAL_SPI_Transmit(w25q_spi, &cmd, 1, TIME_OUT);
	w25q_CS_HIGH();

	w25q_WaitForWriteEnd(210000);
}

// Ativa a permissão de escrita/apagamento na memória
static void w25q_WriteEnable(void) {
    w25q_CS_LOW();
    u8 cmd = CMD_WRITE_ENABLE;
    HAL_SPI_Transmit(w25q_spi, &cmd, 1, TIME_OUT);
    w25q_CS_HIGH();
}

// Trava o código até a memória terminar a operação interna
static void w25q_WaitForWriteEnd(u32 timeout_ms) {
    u8 cmd = CMD_READ_STATUS_1;
    u8 status = 0x01; // <-- PRECAUÇÃO: Assume ocupado por padrão!
    u32 tickstart = HAL_GetTick();

    w25q_CS_LOW();
    HAL_SPI_Transmit(w25q_spi, &cmd, 1, TIME_OUT);
    do {
        // PRECAUÇÃO: Se a leitura falhar por causa do USB, mantém ocupado
        if (HAL_SPI_Receive(w25q_spi, &status, 1, 10) != HAL_OK) {
            status = 0x01;
        }

        if ((HAL_GetTick() - tickstart) >= timeout_ms) {
            break;
        }
    } while ((status & 0x01) == 0x01);
    w25q_CS_HIGH();
}


static void recuperarEnderecoW25Q(void) {
    LogData_t leitura;
    u32 addr = 0;
    char buffer_msg[64];

    printlnLYellow("\r\n--- ESCANEANDO MEMORIA PARA INICIALIZACAO/RECUPERACAO DE SESSAO ---");

    // A leitura pula de 256 em 256 bytes, tornando a busca por 4MB quase instantânea.
    while (addr < LIMIT) {
    	// PRECAUÇÃO: Proteção contra cabo solto ou Timeout SPI
    	memset(&leitura, 0xFF, sizeof(LogData_t));

        w25q_CS_LOW();
        u8 cmd[4] = { 0x03, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF };
        HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);

        // Lê apenas a primeira struct (32 bytes) da página
        HAL_SPI_Receive(w25q_spi, (u8*)&leitura, sizeof(LogData_t), TIME_OUT);
        w25q_CS_HIGH();

        // Se a hora for 0xFF, significa que a página inteira está virgem
        if (leitura.hora == 0xFF) {
            break;
        }

        addr += PAGE_PROGRAM_MAX; // Avança para a próxima página de 256 bytes
    }

    currentAddr = addr;

    snprintf(buffer_msg, sizeof(buffer_msg), "Ponteiro inicializado/restaurado no endereco: 0x%06lX\r\n", currentAddr);
    printlnLGreen("%s", buffer_msg);
}


void salvarCaixaPretaW25Q(float pressao_ref, u32 status) {
    CaixaPreta_t cfg = {pressao_ref, status};
    u8 cmd_erase[4] = {CMD_SECTOR_ERASE_4K, (W25Q_CAIXA_PRETA_ADDR >> 16) & 0xFF, (W25Q_CAIXA_PRETA_ADDR >> 8) & 0xFF, W25Q_CAIXA_PRETA_ADDR & 0xFF};
    u8 cmd_write[4] = {CMD_PAGE_PROGRAM, (W25Q_CAIXA_PRETA_ADDR >> 16) & 0xFF, (W25Q_CAIXA_PRETA_ADDR >> 8) & 0xFF, W25Q_CAIXA_PRETA_ADDR & 0xFF};

    // 1. Apaga o último setor
    w25q_WriteEnable();
    w25q_CS_LOW();
    HAL_SPI_Transmit(w25q_spi, cmd_erase, 4, TIME_OUT);
    w25q_CS_HIGH();
    w25q_WaitForWriteEnd(500);

    // 2. Grava a nova struct
    w25q_WriteEnable();
    w25q_CS_LOW();
    HAL_SPI_Transmit(w25q_spi, cmd_write, 4, TIME_OUT);
    HAL_SPI_Transmit(w25q_spi, (u8*)&cfg, sizeof(CaixaPreta_t), TIME_OUT);
    w25q_CS_HIGH();
    w25q_WaitForWriteEnd(20);
}

CaixaPreta_t lerCaixaPretaW25Q(void) {
    CaixaPreta_t cfg = {1013.25f, 0}; // Valores padrão de segurança
    u8 cmd_read[4] = {0x03, (W25Q_CAIXA_PRETA_ADDR >> 16) & 0xFF, (W25Q_CAIXA_PRETA_ADDR >> 8) & 0xFF, W25Q_CAIXA_PRETA_ADDR & 0xFF};

    w25q_CS_LOW();
    HAL_SPI_Transmit(w25q_spi, cmd_read, 4, TIME_OUT);
    HAL_SPI_Receive(w25q_spi, (u8*)&cfg, sizeof(CaixaPreta_t), TIME_OUT);
    w25q_CS_HIGH();

    return cfg;
}


// ============================================================================
// FUNCAO DE SIMULACAO (MOCK) PARA TESTES DA ESTACAO SOLO
// ============================================================================
void gerarVooSimuladoW25Q(void) {
    printlnLYellow("\r\n--- GERANDO VOO SIMULADO NA FLASH (SITL) ---");

    // 1. Apaga a memoria para comecar limpo
    apagarLogsW25Q();

    LogData_t log_mock = {0};
    float alt = 0.0f;
    float vel = 0.0f;
    u8 estado = 1; // 1 = ESTADO_PRONTO

    u32 tempo_ms = 0;
    u8 h = 14, m = 30, s = 0; // Hora de lancamento fake (14:30:00)

    printlnLCyan("Calculando fisica e gravando paginas... Aguarde.");

    // Simula ate 120 segundos de voo a 100Hz (12.000 iteracoes)
    for (u16 step = 0; step < 12000; step++) {

        // --- FISICA BASICA DO FOGUETE ---
        if (estado == 1) { // PRONTO
            if (tempo_ms > 1000) { estado = 2; } // Decola apos 1 segundo
        }
        else if (estado == 2) { // EM_VOO
            if (tempo_ms < 4000) {
                vel += 45.0f * 0.01f; // Queima do motor (Aceleracao positiva)
            } else {
                vel -= 9.81f * 0.01f; // Voo livre (Gravidade puxando)
            }

            if (vel <= 0.0f && alt > 50.0f) {
                estado = 3; // APOGEU
            }
        }
        else if (estado == 3) { // APOGEU
            estado = 4; // Vai direto para RECUPERACAO
            vel = -15.0f; // Paraquedas abre (choque para velocidade terminal)
        }
        else if (estado == 4) { // RECUPERACAO
            vel += ( (-8.0f - vel) * 0.05f ); // Desacelera ate estabilizar em -8 m/s
            if (alt <= 0.0f) {
                estado = 5; // POUSADO
                vel = 0.0f;
                alt = 0.0f;
            }
        }

        // Integracao da posicao (Euler)
        alt += vel * 0.01f;
        if (alt < 0) alt = 0;

        // --- SENSORES FAKE ---
        // Pressao baseada na altitude (Inverso da formula barometrica)
        float pressao = 1013.25f * powf(1.0f - (alt / 44330.0f), 5.255f);
        float temp = 30.0f - (alt * 0.0065f); // Esfria com a subida

        // --- PREENCHE A STRUCT ---
        log_mock.hora = h;
        log_mock.min = m;
        log_mock.seg = s;
        log_mock.estado = estado;
        log_mock.pressao = pressao;
        log_mock.temperatura = temp;
        log_mock.altitude = alt;
        log_mock.velocidade = vel;

        // --- GRAVA DIRETO NO BUFFER DA W25Q ---
        bufferRAM[bufferIndex] = log_mock;
        bufferIndex++;
        if (bufferIndex >= LOGS_PER_PAGE(LogData_t)) {
            w25q_WriteEnable();
            w25q_CS_LOW();
            u8 cmd[4] = { CMD_PAGE_PROGRAM, (currentAddr >> 16) & 0xFF, (currentAddr >> 8) & 0xFF, currentAddr & 0xFF };
            HAL_SPI_Transmit(w25q_spi, cmd, 4, 2);
            HAL_SPI_Transmit(w25q_spi, (u8*)bufferRAM, bufferIndex * sizeof(LogData_t), 10);
            w25q_CS_HIGH();
            currentAddr += (bufferIndex * sizeof(LogData_t));
            w25q_WaitForWriteEnd(20);

            bufferIndex = 0; // Reseta buffer
        }

        // --- RELOGIO FAKE ---
        tempo_ms += 10; // dt = 10ms
        if (tempo_ms % 1000 == 0) {
            s++;
            if (s > 59) { s = 0; m++; }
            if (m > 59) { m = 0; h++; }
        }

        // Se ja pousou e passou 2 segundos de margem, encerra a simulacao
        if (estado == 5 && tempo_ms > 60000) break;
    }

    descarregarBuffer(); // Descarrega os ultimos logs presos no buffer
    printlnLGreen("--- VOO SIMULADO GRAVADO COM SUCESSO! ---");
    printlnLCyan("Pressione 'V' para ver a tabela.");
}
