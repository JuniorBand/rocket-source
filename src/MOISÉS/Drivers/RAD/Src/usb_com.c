/*
  ******************************************************************************
  * @file    usb_com.c
  * @date 	 4 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Source code de utilidades para o USB.
  ******************************************************************************
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

			printlnLBlue("\r\nLigando LED do PC13!\r\n");
			acenderLedPlaca(); // Liga PC13

			printlnLGreen("\r\nComando { %c } - Visualizar Todos os Logs do W25Q.\r\n", comando);
			visualizarLogsW25Q();

			break;
		case 'U':// Printa somente o último Log

			printlnLBlue("\r\nLigando LED do PC13!\r\n");
			acenderLedPlaca(); // Liga PC13

			printlnLCyan("\r\nComando { %c } - Visualizar Último Log do W25Q.\r\n", comando);
			visualizarUltimoLogW25Q();

			break;
		case 'I': // Idle/Parar
			printlnLBlue("\r\nDesligando LED do PC13!\r\n");
			apagarLedPlaca(); // Liga PC13

			printlnLRed("\r\nComando { %c } - Parar Gravação do W25Q.\r\n", comando);
			pararGravacaoW25Q();

			break;
		case 'A': // Apagar só a memória até onde os logs foram escritos.
			printlnLYellow("\r\nDesligando LED do PC13!\r\n");
			apagarLedPlaca(); // Desliga PC13

			printlnLRed("\r\nComando { %c } - Apagar Todos os Logs do W25Q.\r\n", comando);
			apagarLogsW25Q();

			break;
		case '$': // (CUIDADO!) Apagar absolutamente toda o W25Q.
			printlnLYellow("\r\nDesligando LED do PC13!\r\n");
			apagarLedPlaca(); // Desliga PC13

			printlnLRed("\r\nComando { %c } - Apagar Todo o W25Q.\r\n", comando);
			printlnLYellow("Apagando memoria, aguarde 40s...");
			apagarTudoW25Q();

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
