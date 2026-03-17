/*
  ******************************************************************************
  * @file    ms5611.c
  * @date 	 3 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Source code do driver para o MS5611.
  ******************************************************************************
*/

#include "ms5611.h"
#include <math.h>


static SPI_HandleTypeDef *ms5611_spi;
static TIM_HandleTypeDef *ms5611_timer;
static MS5611_State_t estado_ms5611;
static volatile uint8_t timer_ready_flag = 0;
uint32_t sensor_timer = 0;
uint8_t temp_skip = 0;
uint8_t flag_novo_dado = 0;
static uint32_t D1;
static uint32_t D2_cache = 8000000;
static int64_t dT, OFF, SENS, P;
static MS5611_t *sensor_ptr;


static inline void MS5611_Select(void);
static inline void MS5611_Unselect(void);
static void MS5611_Reset(void);
static void MS5611_Read_PROM(void);

void MS5611_Init(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim, MS5611_t *sensor_rec) {
    ms5611_spi = hspi;
    ms5611_timer = htim;
    sensor_ptr = sensor_rec;

    MS5611_Reset();
    MS5611_Read_PROM();
}


static inline void MS5611_Select(void) { writePinLow(GPIOA, MS5611_CS_PIN); }
static inline void MS5611_Unselect(void) { writePinHigh(GPIOA, MS5611_CS_PIN); }

/**
 * Reseta o chip do sensor_ptr-> Obrigatorio no boot para limpar registradores internos.
 */
static void MS5611_Reset(void) {
    uint8_t res_alt = 0x1E; // Comando de Reset da folha de dados (Datasheet)
    MS5611_Select(); HAL_SPI_Transmit(ms5611_spi, &res_alt, 1, 20); MS5611_Unselect();
    HAL_Delay(10); // Tempo para o capacitor interno do sensor carregar
}

/**
 * O MS5611 possui imperfeicoes de fabrica no silicio.
 * Ele guarda 6 parametros de calibracao na sua ROM interna. Precisamos ler isso 1x no boot.
 */
static void MS5611_Read_PROM(void) { // OK!!!
    uint8_t cmd, data[2];
    for (int i = 0; i < 8; i++) {
        cmd = 0xA0 + (i * 2); // Os enderecos da PROM sao A0, A2, A4...
        MS5611_Select();
        HAL_SPI_Transmit(ms5611_spi, &cmd, 1, 10);
        HAL_SPI_Receive(ms5611_spi, data, 2, 10); // Recebe 16 bits de calibracao
        MS5611_Unselect();
        sensor_ptr->c[i] = (uint16_t)(data[0] << 8) | data[1]; // Junta os dois blocos de 8 bits
    }
}


static inline void MS5611_ReadPress(void) {
	u8 cmd = 0x00;
	u8 data[3];


	MS5611_Select();
	HAL_SPI_Transmit(ms5611_spi, &cmd, 1, 10);
	HAL_SPI_Receive(ms5611_spi, data, 3, 10);
	MS5611_Unselect();

	D1 = TRANSF_U24_BIT_BIGEND(data, 0);

	// C�lculo MS5611
	dT = (int64_t)D2_cache - ((int64_t)sensor_ptr->c[5] << 8);
	OFF = ((int64_t)sensor_ptr->c[2] << 16) + (((int64_t)sensor_ptr->c[4] * dT) >> 7);
	SENS = ((int64_t)sensor_ptr->c[1] << 15) + (((int64_t)sensor_ptr->c[3] * dT) >> 8);
	P = (((D1 * SENS) >> 21) - OFF) >> 15;

	sensor_ptr->pressao = (float)P / 100.0f;

	// F�rmula Barom�trica
	if (sensor_ptr->pressao_ref > 0){
		sensor_ptr->altitude = 44330.0f * (1.0f - powf((sensor_ptr->pressao / sensor_ptr->pressao_ref), 0.190295f));
	}

	flag_novo_dado = 1;
	estado_ms5611 = 0;
}

static inline void MS5611_ReadTemp(void) {
	uint8_t cmd = 0x00;
	u8 data[3];


	MS5611_Select();
	HAL_SPI_Transmit(ms5611_spi, &cmd, 1, 10);
	HAL_SPI_Receive(ms5611_spi, data, 3, 10);
	MS5611_Unselect();

	D2_cache = TRANSF_U24_BIT_BIGEND(data, 0);

	dT = (int64_t)D2_cache - ((int64_t)sensor_ptr->c[5] << 8);
    int64_t TEMP = 2000 + ((dT * (int64_t)sensor_ptr->c[6]) >> 23); // 2000 = 20.00 Celsius


	// 4. Matematica de 2a Ordem (Compensacao Termica Rigorosa para frio de altitude)
	    if (TEMP < 2000) { // Se estiver abaixo de 20 Graus Celsius
	        int64_t T2 = (dT * dT) >> 31;
	        int64_t OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
	        int64_t SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;
	        TEMP -= T2; OFF -= OFF2; SENS -= SENS2;
	    }


	    sensor_ptr->temperatura = (float)TEMP / 100.0f; // Ex: 2500 vira 25.00 Graus

	    estado_ms5611 = 0;

}

// Máquina de Estados para ler MS5611 sem delays bloqueantes
void MS5611_ReadData(void) {
    // Como essa função agora SÓ é chamada a cada 10ms pelo Timer,
    // não precisamos mais de HAL_GetTick() nem de delays.

    switch (estado_ms5611) {

        case MS_PEDIR_PRESSAO:
            // 1. Manda o comando para iniciar conversão de Pressão (D1)
            uint8_t cmd_p = 0x48;
            MS5611_Select();
            HAL_SPI_Transmit(ms5611_spi, &cmd_p, 1, 10);
            MS5611_Unselect();

            estado_ms5611 = MS_LER_PRESSAO; // No próximo tick (daqui a 10ms), o dado estará pronto
            break;

        case MS_LER_PRESSAO:
            // 2. Já passaram 10ms. Resgata a pressão e faz o cálculo
            MS5611_ReadPress();

            // Avalia se já está na hora de atualizar a temperatura
            if (temp_skip++ > 20) {
                uint8_t cmd_t = 0x58; // Manda converter Temperatura (D2)
                MS5611_Select();
                HAL_SPI_Transmit(ms5611_spi, &cmd_t, 1, 10);
                MS5611_Unselect();

                estado_ms5611 = MS_LER_TEMP_E_CALCULAR;
                temp_skip = 0;
            } else {
                estado_ms5611 = MS_PEDIR_PRESSAO; // Volta pro início para o próximo tick
            }
            break;

        case MS_LER_TEMP_E_CALCULAR:
            // 3. Já passaram mais 10ms. Resgata a temperatura
            MS5611_ReadTemp();
            estado_ms5611 = MS_PEDIR_PRESSAO; // Ciclo completo, volta pro início
            break;

        default:
            estado_ms5611 = MS_PEDIR_PRESSAO;
            break;
    }
}
