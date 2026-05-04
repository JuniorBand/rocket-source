/*
  ******************************************************************************
  * @file    macros_config.h
  * @date 	 3 de mai. de 2026
  * @author  Junior Bandeira
Y  * @brief   PAINEL DE CONTROLE DAS MACROS DA MISSÃO
  * Ative ou desative as macros abaixo para alterar o comportamento
  * do firmware em tempo de compilação.
  ******************************************************************************
*/


#ifndef MACROS_CONFIG_H_
#define MACROS_CONFIG_H_



// ==============================================================================
// 1. ESTADO DA MISSÃO (Comente para MODO BANCADA, Descomente para MODO VOO)
// ==============================================================================
#define EM_VOO // Pode usar a IDE para definir o EM_VOO.
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
#define USE_FILTER

// Defina a macro USE_BUZZER se você tiver um buzzer conectado e quiser usar a
// função de beep para feedback sonoro durante a simulação ou voo real.
#define USE_BUZZER

// Printa os dados do sensor e do filtro a cada 1 segundo (100 ticks) para debug em tempo real.
//#define VERBOSE

// Permite gravar os logs na memoria (Caso nao queira que ele grave mais: comente)
#define GRAVAR_LOGS




#endif /* MACROS_CONFIG_H_ */
