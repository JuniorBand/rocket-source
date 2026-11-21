/*
  ******************************************************************************
  * @file    macros_config.h
  * @date 	 20 de nov. de 2026
  * @author  Junior Bandeira
  * @brief   PAINEL DE CONTROLE DAS MACROS DA MISSÃO
  * Ative ou desative as macros abaixo para alterar o comportamento
  * do firmware em tempo de compilação.
  ******************************************************************************
*/

#ifndef MACROS_CONFIG_H_
#define MACROS_CONFIG_H_


// ####$ ADAPTAR PARA O PROJETO MOISÉS 5


// ==============================================================================
// 1. ESTADO DA MISSÃO (Comente para MODO BANCADA, Descomente para MODO VOO)
// ==============================================================================
//#define EM_VOO // Pode usar a IDE para definir o EM_VOO.
// ^ Se definido: Desliga prints USB para economizar CPU, ativa rotinas rígidas.

// ==============================================================================
// 2. HARDWARE DE ARMAZENAMENTO (Comente para usar a Flash interna do STM32)
// ==============================================================================
#define USE_W25Q
// ^ Se definido: Roteia o sistema de arquivos para o chip externo W25Q.
// Caso contrario, utiliza a flash nativa da placa.

// ATENCAO: Use o USE_FILTER, pois o não uso dele pode causar erros de altitude negativa e
// outros glitches imprevistos, já que o filtro Kalman é responsável por suavizar os dados
// do sensor e fornecer uma estimativa mais estável da altitude e velocidade.
// A velocidade ainda não foi ajustada para o modo sem filtro, então, se você desativar o filtro,
// o sistema de lançamento automático pode não funcionar corretamente, pois ele depende de uma
// leitura de velocidade confiável para detectar a decolagem.
#define USE_KALMAN_FILTER

#define USE_PASSA_BAIXA_FILTER

#define MANDAR_DADOS_TELEMETRIA

// Defina a macro USE_BUZZER se você tiver um buzzer conectado e quiser usar a
// função de beep para feedback sonoro durante a simulação ou voo real.
#define USE_BUZZER

// Printa os dados do sensor e do filtro a cada 1 segundo (100 ticks) para debug em tempo real.
#define VERBOSE

// Permite gravar os logs na memoria (Caso nao queira que ele grave mais: comente)
#define GRAVAR_LOGS

// Permite colocar cores ou tirar as cores dos prints (Caso nao queira cores: comente)
#define PRINT_COLORS

// ============================================================
// PINOS - devem corresponder à fiação atual da placa
// ============================================================

#define BUZZER_PIN PB1
#define LED_PIN    PC13       // LED onboard da Black Pill: ativo em LOW

// W25Q64: SPI1 padrão da Black Pill (PA5 SCK / PA6 MISO / PA7 MOSI)
#define W25Q_CS   PA4

// BMP280: mesmos pinos da versão anterior
#define BMP_CS     PA15
#define BMP_SCK    PB3
#define BMP_MISO   PB4
#define BMP_MOSI   PB5

// MPU6050 soldado nas trilhas do BMI088 (PB14 = antigo MISO; PB15 = antigo MOSI)
// Usa I2C por software; NAO habilitar SPI2 nesses pinos.
#define MPU_SCL    PB14
#define MPU_SDA    PB15
#define MPU_AD0    PB2
#define MPU_ADDR   0x68

// UART2: PA3 RX, PA2 TX. Somente protocolo binario v1, 115200 baud.
// Telemetria 20 Hz + telecomandos/ACK/NACK + transferencia de log.
// Nao existe mais modo texto do Tera Term nesta interface.

// 1 = SPI3 em hardware nos pinos PB3/PB4/PB5 (função alternativa ALT1)
// 0 = SPI por software (igual à versão anterior). Use 0 se o BMP280
//     não for encontrado com o SPI em hardware.
#define BMP_USE_HW_SPI 1



#endif /* MACROS_CONFIG_H_ */
