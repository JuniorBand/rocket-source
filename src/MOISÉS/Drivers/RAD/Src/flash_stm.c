/*
  ******************************************************************************
  * @file    flash_stm.c
  * @author  Júnior Bandeira
  * @brief   Source code do driver para controlar a flash nativa dos STM32F411/STM32F401
  ******************************************************************************
*/

#include <flash_stm.h>
#include <utils.h>
#include <usb_com.h>


// Descobre quantas structs inteiras cabem nesses 128 KB
const u16 max_structs = (u16) (TAMANHO_SETOR_FLASH / sizeof(DadosVoo_t));
// Por exemplo, se for o F411 e DadosVoo_t tiver 96 bits ou 12 bytes, temos max_structs = 10666 (lembrando que ele é inteiro)

void salvarDado(DadosVoo_t *dados) {
    // Aponta para o início absoluto do setor
    DadosVoo_t *flash_ptr = (DadosVoo_t *) ENDERECO_FLASH_ALVO;
    u16 index = 0;

    // ---------------------------------------------------------
    // 1. A BUSCA PELO NOVO ENDEREÇO
    // ---------------------------------------------------------
    while (index < max_structs) {
        // Lemos os primeiros 4 bytes. Como 'magic_number' é o primeiro
        // campo da struct, estamos lendo ele diretamente.
        u32 primeiro_dado = *(u32 *)&flash_ptr[index];

        // Se o dado for 0xFFFFFFFF, o bloco está virgem/vazio.
        if (primeiro_dado == 0xFFFFFFFF) {
            break;
        }
        index++;
    }

    // Proteção: Se o index chegou no limite, aborta para não travar o microcontrolador
    if (index >= max_structs) {
        printRed("FLASH CHEIA!!! Gravacao abortada para proteger dados anteriores.\r\n");
        return;
    }

    // ---------------------------------------------------------
    // 2. PREPARAÇÃO E GRAVAÇÃO NA FLASH
    // ---------------------------------------------------------
    HAL_FLASH_Unlock();

    // Limpa as flags de erro de operações passadas. Se um pico de tensão ou
    // interrupção causou um erro silencioso na Flash antes, isso evita que
    // a HAL aborte a gravação atual.
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    // Carrega a assinatura de validade na struct antes de jogar para a memória
    dados->magicNumber = MAGIC_NUMBER_SIRIUS;

    u32 *ponteiro_dados = (u32 *)dados;

    // Calcula quantas palavras de 32 bits formam a struct.
    // O __attribute__((aligned(4))) no .h garante que a divisão seja exata.
    u16 tamanho_em_words = (u16) ((sizeof(DadosVoo_t) + 3) / 4);

    for (u32 i = 0; i < tamanho_em_words; i++) {
        // Calcula o offset de memória para cada palavra de 32 bits
        u32 endereco_atual = (u32)&flash_ptr[index] + (i * 4);

        // Grava a palavra no novo endereço
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, endereco_atual, ponteiro_dados[i]);
    }

    HAL_FLASH_Lock();
}

void printFlash(void) {
    DadosVoo_t *flash_ptr = (DadosVoo_t *)ENDERECO_FLASH_ALVO;
    char buffer_usb[128];

    snprintf(buffer_usb, sizeof(buffer_usb), "\r\n--- INICIANDO LEITURA DA CAIXA PRETA ---\r\n");
    printYellow("%s", buffer_usb);
    HAL_Delay(50); // Dá um tempo para o terminal do PC processar

    u32 index = 0;

    printLGreen("--- DADOS RECUPERADOS ---\r\n");
    while (index < max_structs) {
        u32 primeiro_dado = *(u32 *)&flash_ptr[index];

        // Se o bloco está virgem, a leitura encerra aqui.
        if (primeiro_dado == 0xFFFFFFFF) {
            break;
        }

        // Se o bloco tem dados, mas não possui a assinatura correta,
        // considera como lixo de memória e pula para o próximo registro.
        if (primeiro_dado != MAGIC_NUMBER_SIRIUS) {
            index++;
            continue;
        }

        snprintf(buffer_usb, sizeof(buffer_usb),
        		"[%lu] P:%.1f T:%.1f A:%.1f V:%.1f E:%u\r\n",
                 index,
                 flash_ptr[index].pressaoAtual,
                 flash_ptr[index].temperaturaAtual,
                 flash_ptr[index].altitudeAtual,
                 flash_ptr[index].velocidadeAtual,
                 flash_ptr[index].estadoAtual);
        printCyan("%s", buffer_usb);

        // Pausa crucial para não estourar o buffer do controlador USB
        HAL_Delay(10);

        index++;
    }

    snprintf(buffer_usb, sizeof(buffer_usb), "--- FIM DA LEITURA (Total: %lu registros) ---\r\n\r\n", index);
    printYellow("%s", buffer_usb);
}

void apagarCaixaPreta(void) {
    HAL_FLASH_Unlock();

    // Também é estritamente necessário limpar as flags antes de invocar o apagamento
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseInitTypeDef EraseInitStruct;
    u32 SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = SETOR_FLASH_ALVO;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
        printlnLGreen("\r\nMemória apagada com sucesso. Pronta para novo voo.\r\n");
    } else {
        printlnLRed("\r\nERRO crítico ao apagar a memoria.\r\n");
    }

    HAL_FLASH_Lock();
}
