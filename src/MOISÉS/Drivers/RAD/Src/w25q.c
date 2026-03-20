/*
  ******************************************************************************
  * @file    w25q.c
  * @date 	 3 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Source code do driver para o flash W25Q128.
  ******************************************************************************
*/


#include <w25q.h>
#include <utils.h>
#include <config_voo.h>
#include <stdio.h>   // Para o snprintf não gerar warning
#include <string.h>  // Para o memset não gerar warning
#ifndef EM_VOO
	#include <usb_com.h>
#endif


#define W25Q_SECTOR_SIZE 4096 // 4 KB para apagar de uma vez (mínimo = tamanho de um setor do W25Q)
#define W25Q_BLOCK_SIZE 65536 // 64 KB para apagar de uma vez
#define W25Q_CAIXA_PRETA_ADDR 0xFFF000 // Último setor de 4KB do chip (16MB)
#define DELAY 5
#define PAGE_PROGRAM_MAX 256 // bytes
#define LOGS_PER_PAGE(nome_struct_t) (PAGE_PROGRAM_MAX/sizeof(nome_struct_t))   // Quantos logs podem ser gravados de uma vez em 256 bytes (1 P�gina da Flash)
#define TIME_OUT 2


static SPI_HandleTypeDef* w25q_spi;
static TIM_HandleTypeDef* w25q_timer;
W25Q_State_t sensor_state = IDLE;
static volatile u8 timer_ready_flag = 0;
static u8 w25q_cs = 0;
u32 currentAddr = 0;
static LogData_t bufferRAM[LOGS_PER_PAGE(LogData_t)] = {0};
static u32 bufferIndex = 0;


static void w25qWritePage(void);
static void w25q_WriteEnable(void);
static void recuperarEnderecoW25Q(void);
static void w25q_WaitForWriteEnd(u32 timeout_ms);
static void w25q_EraseBlock64K(uint32_t BlockAddr);
//static void w25q_EraseSector(uint32_t SectorAddr);



#if defined(W25Q_CS_PORT) && defined(W25Q_CS_PIN)
	static inline void w25q_CS_LOW(void) { writePinLow(W25Q_CS_PORT, W25Q_CS_PIN); }
	static inline void w25q_CS_HIGH(void) { writePinHigh(W25Q_CS_PORT, W25Q_CS_PIN); }
#else
	#error "W25Q_CS_PORT e/ou W25Q_CS_PIN não foram definidos no main.h!"
#endif

void w25qInit(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim, u8 W25Q_CS){
	w25q_spi = hspi;
	w25q_timer = htim;
	w25q_cs = W25Q_CS;
	recuperarEnderecoW25Q();
}


void visualizarLogsW25Q(void) {
    LogData_t leitura;
    u32 printAddr = 0;
    char buffer_saida[128];
    char barra_grafica[50];

    // Banner com 84 caracteres para englobar toda a tabela nova
    printlnLCyan("\r\n====================================================================================");
    printlnLYellow("                       RELATORIO DE VOO - VISUALIZADOR (v5.0)                       ");
    printlnLCyan("====================================================================================");

    // Cabeçalho unificado e alinhado: HORA (10) | TELEMETRIA (32) | STATUS (12) | GRAFICO
    printlnLMagenta("   HORA   |       TELEMETRIA (A/V/P/T)     |    STATUS    | GRAFICO (# = 10m)");
    printlnLCyan("----------|--------------------------------|--------------|-------------------------");

    while (printAddr < LIMIT) {
        // Leitura Física da Flash W25Q
        w25q_CS_LOW();
        u8 cmd[4] = { 0x03, (printAddr >> 16) & 0xFF, (printAddr >> 8) & 0xFF, printAddr & 0xFF };
        HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
        HAL_SPI_Receive(w25q_spi, (u8*)&leitura, sizeof(LogData_t), 20);
        w25q_CS_HIGH();

        if (leitura.hora == 0xFF) { break; }// Encontrou memória vazia, fim.

        // Gera Barra Gráfica (#) proporcional à altitude
        int num_barras = (int)(leitura.altitude / 10.0f);
        if (num_barras > 40) num_barras = 40;
        if (num_barras < 0) num_barras = 0;

        // Limpa o array com zeros ('\0') e depois preenche com '#'.
        memset(barra_grafica, 0, sizeof(barra_grafica));
        for (int i = 0; i < num_barras; i++) barra_grafica[i] = '#';

        // Proteção de índice do label
        u8 st = (u8)leitura.estado;
        if (st > 6) st = 6;

        // Formatação perfeitamente alinhada com as colunas do cabeçalho
        // Espaço extra antes do %02d centraliza a hora na coluna de 10 caracteres
        snprintf(buffer_saida, sizeof(buffer_saida),
            " %02d:%02d:%02d | A:%5.1f V:%5.1f P:%6.1f T:%5.1f | %-12s | %s",
            leitura.hora, leitura.min, leitura.seg,
            leitura.altitude,
            leitura.velocidade,
            leitura.pressao,
            leitura.temperatura, // Corrigido o erro de T:V:
            PRINT_ESTADO[st],
            barra_grafica);

        printlnLGreen("%s", buffer_saida);
        HAL_Delay(DELAY);

        printAddr += sizeof(LogData_t);
    }

    u32 tempo_ligado = ((HAL_GetTick() - seguroVoo.tempo_inicio_ms)/1000);

        // Rodapé Estatístico ajustado para usar snprintf
        printlnMagenta("====================================================================================");
        snprintf(buffer_saida, sizeof(buffer_saida),
                "ESTATISTICAS FINAIS:\r\n- Altitude Maxima: %.2f m; Tempo de Voo: %lu s; Runtime: %lu s.",
                seguroVoo.altitude_maxima, (seguroVoo.tempo_inicio_ms/1000), tempo_ligado);
        printlnLGreen("%s", buffer_saida);
        printlnMagenta("====================================================================================");
}


void visualizarUltimoLogW25Q(void){
    LogData_t leitura;
    u32 printAddr = currentAddr - (bufferIndex * sizeof(LogData_t)); // Último endereço que foi printado
    char buffer_saida[128];
    char barra_grafica[50];

    // Banner e cabeçalhos idênticos à função principal
    printlnLCyan("\r\n====================================================================================");
    printlnLYellow("                       RELATORIO DE VOO - VISUALIZADOR (v5.0)                       ");
    printlnLCyan("====================================================================================");
    printlnLMagenta("   HORA   |       TELEMETRIA (A/V/P/T)     |    STATUS    | GRAFICO (# = 10m)");
    printlnLCyan("----------|--------------------------------|--------------|-------------------------");

    if (currentAddr < LIMIT) {
        // Leitura Física da Flash W25Q
        w25q_CS_LOW();
        u8 cmd[4] = { 0x03, (printAddr >> 16) & 0xFF, (printAddr >> 8) & 0xFF, printAddr & 0xFF };
        HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
        HAL_SPI_Receive(w25q_spi, (u8*)&leitura, sizeof(LogData_t), 20);
        w25q_CS_HIGH();

        if (leitura.hora == 0xFF) { printlnRed("Nenhum dado salvo!"); return; }

        // Gera Barra Gráfica (#) proporcional à altitude
        int num_barras = (int)(leitura.altitude / 10.0f);
        if (num_barras > 40) num_barras = 40;
        if (num_barras < 0) num_barras = 0;

        memset(barra_grafica, 0, sizeof(barra_grafica));
        for (int i = 0; i < num_barras; i++) barra_grafica[i] = '#';

        u8 st = (u8)leitura.estado;
        if (st > 6) st = 6;

        snprintf(buffer_saida, sizeof(buffer_saida),
            " %02d:%02d:%02d | A:%5.1f V:%5.1f P:%6.1f T:%5.1f | %-12s | %s",
            leitura.hora, leitura.min, leitura.seg,
            leitura.altitude,
            leitura.velocidade,
            leitura.pressao,
            leitura.temperatura, // Já estava aqui, mas agora faz sentido com o cabeçalho
            PRINT_ESTADO[st],
            barra_grafica);

        printlnLGreen("%s", buffer_saida);
        HAL_Delay(DELAY);
    }

    u32 tempo_ligado = ((HAL_GetTick() - seguroVoo.tempo_inicio_ms)/1000);

    // Rodapé Estatístico ajustado para usar snprintf
    printlnMagenta("====================================================================================");
    snprintf(buffer_saida, sizeof(buffer_saida),
            "ESTATISTICAS FINAIS:\r\n- Altitude Maxima: %.2f m; Tempo de Voo: %lu s; Runtime: %lu s.",
            seguroVoo.altitude_maxima, (seguroVoo.tempo_inicio_ms/1000), tempo_ligado);
    printlnLGreen("%s", buffer_saida);
    printlnMagenta("====================================================================================");
}

// ============================================================================
// 3. STORAGE & SENSOR DRIVERS
// ============================================================================

// Adiciona log ao Buffer RAM e descarrega na Flash quando cheio
void adicionarLogW25Q(RTC_HandleTypeDef* hrtc_log, DadosVoo_t *dadosVoo) {

	RTC_TimeTypeDef sTime;
    HAL_RTC_GetTime(hrtc_log, &sTime, RTC_FORMAT_BIN);
    RTC_DateTypeDef sDate; HAL_RTC_GetDate(hrtc_log, &sDate, RTC_FORMAT_BIN); // Leitura dummy necess�ria

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

	w25q_WriteEnable();

    w25q_CS_LOW();
    // Comando 0x02 = Page Program
    u8 cmd[4] = { 0x02, (currentAddr >> 16) & 0xFF, (currentAddr >> 8) & 0xFF, currentAddr & 0xFF };
    HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
    HAL_SPI_Transmit(w25q_spi, (u8*)bufferRAM, bufferIndex * sizeof(LogData_t), TIME_OUT);
    w25q_CS_HIGH();

    currentAddr += (bufferIndex * sizeof(LogData_t));
    w25q_WaitForWriteEnd(20);
}

void pararGravacaoW25Q(void) {
    // Acesse o buffer RAM e o índice que você declarou como 'static'
    // dentro de adicionarLog(). Sugiro mover essas duas variáveis para
    // o escopo global do w25q.c para poder acessá-las aqui.

    if (bufferIndex > 0) {
        w25qWritePage(); // Grava os logs restantes
        bufferIndex = 0;
    }

    currentAddr = LIMIT; // Trava futuras gravações
}


void apagarLogsW25Q(void) {
    char buffer_msg[64];
    printlnLYellow("\r\n--- INICIANDO LIMPEZA DO W25Q (BLOCOS DE 64KB) ---");

    for (uint32_t addr = 0; addr < LIMIT; addr += W25Q_BLOCK_SIZE) {

        w25q_EraseBlock64K(addr);

        // Printa o progresso a cada 1 bloco apagado
        snprintf(buffer_msg, sizeof(buffer_msg), "Apagando... Endereco: 0x%06lX\r\n", addr);
        printLYellow("%s", buffer_msg);
    }

    currentAddr = 0;
    printlnLGreen("--- LIMPEZA CONCLUIDA! PRONTO PARA VOO ---\r\n");
}

static void w25q_EraseBlock64K(uint32_t BlockAddr) {
    w25q_WriteEnable();

    w25q_CS_LOW();
    uint8_t cmd[4];
    cmd[0] = 0xD8; // Instrução Block Erase (64 KB)
    cmd[1] = (BlockAddr >> 16) & 0xFF;
    cmd[2] = (BlockAddr >> 8) & 0xFF;
    cmd[3] = BlockAddr & 0xFF;

    HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);
    w25q_CS_HIGH();

    w25q_WaitForWriteEnd(3000); // O chip demora de 150ms a 200ms para apagar o bloco
}

/*
static void w25q_EraseSector(uint32_t SectorAddr) {  // Apaga todo um setor do W25Q
    w25q_WriteEnable(); // Sempre chamar antes!

    w25q_CS_LOW();
    uint8_t cmd[4];
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
	uint8_t cmd = 0xC7; // Instrução Chip Erase
	HAL_SPI_Transmit(w25q_spi, &cmd, 1, TIME_OUT);
	w25q_CS_HIGH();

	w25q_WaitForWriteEnd(210000);
}

// Ativa a permissão de escrita/apagamento na memória
static void w25q_WriteEnable(void) {
    w25q_CS_LOW();
    uint8_t cmd = 0x06; // Instrução Write Enable
    HAL_SPI_Transmit(w25q_spi, &cmd, 1, TIME_OUT);
    w25q_CS_HIGH();
}

// Trava o código até a memória terminar a operação interna
static void w25q_WaitForWriteEnd(u32 timeout_ms) {
    u8 cmd = 0x05; // Instrução Read Status Register-1
    u8 status = 0;
    u32 tickstart = HAL_GetTick();

    w25q_CS_LOW();
    HAL_SPI_Transmit(w25q_spi, &cmd, 1, TIME_OUT);
    do {
        // Timeout de 1ms na leitura SPI, quem gerencia o limite de tempo é o tickstart
        HAL_SPI_Receive(w25q_spi, &status, 1, 1);

        // Proteção anti-travamento: Se o chip demorar demais ou o MISO desconectar (0xFF constante)
        if ((HAL_GetTick() - tickstart) >= timeout_ms) {
            break; // Aborta e salva o sistema de travar completamente
        }
    } while ((status & 0x01) == 0x01); // Fica preso aqui enquanto o bit 0 (BUSY) for 1
    w25q_CS_HIGH();
}


static void recuperarEnderecoW25Q(void) {
    LogData_t leitura;
    uint32_t addr = 0;
    char buffer_msg[64];

    printlnLYellow("\r\n--- ESCANEANDO MEMORIA PARA INICIALIZACAO/RECUPERACAO DE SESSAO ---");

    // A leitura pula de 256 em 256 bytes, tornando a busca por 4MB quase instantânea.
    while (addr < LIMIT) {
        w25q_CS_LOW();
        uint8_t cmd[4] = { 0x03, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF };
        HAL_SPI_Transmit(w25q_spi, cmd, 4, TIME_OUT);

        // Lê apenas a primeira struct (32 bytes) da página
        HAL_SPI_Receive(w25q_spi, (uint8_t*)&leitura, sizeof(LogData_t), TIME_OUT);
        w25q_CS_HIGH();

        // Se a hora for 0xFF, significa que a página inteira está virgem
        if (leitura.hora == 0xFF) {
            break;
        }

        addr += PAGE_PROGRAM_MAX; // Avança para a próxima página de 256 bytes
    }

    currentAddr = addr;

    snprintf(buffer_msg, sizeof(buffer_msg), "Ponteiro inicializado/restaurado no endereco: 0x%06lX\r\n", currentAddr);
    printLGreen("%s", buffer_msg);
}


void salvarCaixaPretaW25Q(float pressao_ref, uint32_t status) {
    CaixaPreta_t cfg = {pressao_ref, status};
    uint8_t cmd_erase[4] = {0x20, (W25Q_CAIXA_PRETA_ADDR >> 16) & 0xFF, (W25Q_CAIXA_PRETA_ADDR >> 8) & 0xFF, W25Q_CAIXA_PRETA_ADDR & 0xFF};
    uint8_t cmd_write[4] = {0x02, (W25Q_CAIXA_PRETA_ADDR >> 16) & 0xFF, (W25Q_CAIXA_PRETA_ADDR >> 8) & 0xFF, W25Q_CAIXA_PRETA_ADDR & 0xFF};

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
    HAL_SPI_Transmit(w25q_spi, (uint8_t*)&cfg, sizeof(CaixaPreta_t), TIME_OUT);
    w25q_CS_HIGH();
    w25q_WaitForWriteEnd(20);
}

CaixaPreta_t lerCaixaPretaW25Q(void) {
    CaixaPreta_t cfg = {1013.25f, 0}; // Valores padrão de segurança
    uint8_t cmd_read[4] = {0x03, (W25Q_CAIXA_PRETA_ADDR >> 16) & 0xFF, (W25Q_CAIXA_PRETA_ADDR >> 8) & 0xFF, W25Q_CAIXA_PRETA_ADDR & 0xFF};

    w25q_CS_LOW();
    HAL_SPI_Transmit(w25q_spi, cmd_read, 4, TIME_OUT);
    HAL_SPI_Receive(w25q_spi, (uint8_t*)&cfg, sizeof(CaixaPreta_t), TIME_OUT);
    w25q_CS_HIGH();

    return cfg;
}

