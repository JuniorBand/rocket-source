/*
  ******************************************************************************
  * @file    ms5611.h
  * @date 	 3 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Header do driver para o MS5611.
  ******************************************************************************
*/


/* Pend�ncias
	Fazer Drivers de SPI para MS5611 e MS5611
    Organizar tipos, constantes e funções em um utils file
    Usar mais LL (bare-metal)
	Usar Interrupts para ler o sensor (ex: Timer a cada 10ms) ao inv�s de polling
    Usar LKF como filtro
	Opcional: Fazer o LKF de verdade (com sigma points) para compara��o
	Diminuir incerteza do processo (Q) e da medida (R) para ver mais efeito do filtro
    Em Processar_Logica_Voo (case ESTADO_PRONTO), voc� detecta o lan�amento , muda o estado, mas n�o reinicia a covari�ncia do filtro
    cmd_rst = 0x1E; Oq é?
	Apagar memória de fato (atualmente ele só sobrescreve a partir do início, mas seria bom ter um comando que realmente apague a memória, ou pelo menos marque como "vazia" para não confundir na hora de ler os logs)
    FlightConfig faltando
    Python para salvar logs
 */

#ifndef MS5611_H
#define MS5611_H

#include "utils.h"
#include "stm32f4xx_hal.h"
#include "config_voo.h"

typedef struct MS5611_s {
    u16 c[8];              // Coeficientes C0-C7 lidos da PROM interna do sensor
    float temperatura;          // Temperatura real ja calculada (Celsius)
    float pressao;              // Pressao real ja calculada (hPa)
    float altitude;             // Altitude final relativa ao chao (Metros)
    float pressao_ref;   		// Pressao salva no momento em que a placa ligou no chao (P0)
} MS5611_t;

extern MS5611_t sensor;

// Vari�veis de Controle do Sensor (Non-Blocking)
extern uint32_t sensor_timer;
// Variáveis de estado escondidas do resto do sistema (encapsulamento)
typedef enum {
    MS_PEDIR_PRESSAO,
    MS_LER_PRESSAO,
    MS_LER_TEMP_E_CALCULAR
} MS5611_State_t;

extern uint8_t temp_skip;
extern uint8_t flag_novo_dado;

void MS5611_ReadData(void);
void MS5611_Init(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim, MS5611_t *sensor_rec);






#endif /* MS5611_H */
