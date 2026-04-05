/*
  ******************************************************************************
  * @file    ms5611.c
  * @date    3 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Driver de alta precisao para o Barometro MS5611 (Via SPI)
  ******************************************************************************
*/

/* ==============================================================================
 * MANUAL DA ARQUITETURA DO BAROMETRO (MS5611)
 * ==============================================================================
 *
 * DESCRICAO GERAL:
 * Este driver gerencia o sensor de pressao estatica, responsavel por fornecer
 * a altitude ao Filtro de Kalman. O MS5611 eh um sensor de 24 bits que exige
 * compensacao matematica rigorosa de segunda ordem para operar em grandes altitudes.
 *
 * MAQUINA DE ESTADOS (NON-BLOCKING):
 * O MS5611 leva cerca de 9ms para realizar uma conversao interna em alta
 * resolucao (OSR 4096). Para nao travar o processador esperando o sensor, este
 * driver utiliza uma Maquina de Estados sincronizada com o Timer de 10ms:
 * 1. Tick N   : Envia comando de conversao.
 * 2. Tick N+1 : Le o dado convertido e calcula a fisica.
 * Isso garante que o loop principal do foguete nunca pare.
 *
 * MOTOR MATEMATICO (COMPENSACAO DE 2a ORDEM):
 * A pressao lida (D1) varia com a temperatura (D2). O driver executa a
 * "Matematica de Curva" oficial da Measurement Specialties, incluindo a
 * correcao para baixas temperaturas (abaixo de 20C), garantindo que a
 * altitude nao sofra deriva (drift) conforme o foguete sobe e o ar esfria.
 *
 * OTIMIZACAO DE BANDA:
 * Como a temperatura ambiente nao muda tao rapido quanto a pressao no voo,
 * este driver le a temperatura (D2) apenas 1 vez a cada 20 leituras de pressao.
 * Isso economiza barramento SPI e foca o processamento na subida rapida.
 * ==============================================================================
 */

#include <ms5611.h>
#include <math.h>
#include <usb_com.h>

// ============================================================================
// COMANDOS DO MS5611 (Datasheet - Modo OSR 4096)
// ============================================================================
#define MS5611_CMD_RESET       0x1E
#define MS5611_CMD_READ_ADC    0x00
#define MS5611_CMD_PROM_READ   0xA0
#define MS5611_CMD_CONV_D1_OSR 0x48  // Pressao (OSR 4096)
#define MS5611_CMD_CONV_D2_OSR 0x58  // Temperatura (OSR 4096)

static SPI_HandleTypeDef *ms5611_spi;
static TIM_HandleTypeDef *ms5611_timer;
static MS5611_State_t estado_ms5611;

u32 sensor_timer = 0;
u8 temp_skip = 0;
u8 flag_novo_dado = 0;
static u32 time_verbose = 0;
static u8 d2_lido = 0; // Flag para garantir que a temperatura real chegou

static u32 D1;
static u32 D2_cache = 8000000; // Valor seguro de inicialização
static MS5611_t *sensor_ptr;


static inline void MS5611_Select(void);
static inline void MS5611_Unselect(void);
static void MS5611_Reset(void);
static void MS5611_Read_PROM(void);
static void MS5611_CalcularTudo(void);

void MS5611_Init(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim, MS5611_t *sensor_rec) {
    ms5611_spi = hspi;
    ms5611_timer = htim;
    sensor_ptr = sensor_rec;

    MS5611_Reset();
    MS5611_Read_PROM();
}

static inline void MS5611_Select(void) { writePinLow(MS5611_CS_PORT, MS5611_CS_PIN); }
static inline void MS5611_Unselect(void) { writePinHigh(MS5611_CS_PORT, MS5611_CS_PIN); }

static void MS5611_Reset(void) {
    u8 res_alt = MS5611_CMD_RESET;
    MS5611_Select();
    HAL_SPI_Transmit(ms5611_spi, &res_alt, 1, 20);
    MS5611_Unselect();
    HAL_Delay(10);
}

static void MS5611_Read_PROM(void) {
    u8 cmd, data[2];
    for (int i = 0; i < 8; i++) {
        cmd = MS5611_CMD_PROM_READ + (i * 2);
        MS5611_Select();
        HAL_SPI_Transmit(ms5611_spi, &cmd, 1, 10);
        HAL_SPI_Receive(ms5611_spi, data, 2, 10);
        MS5611_Unselect();
        sensor_ptr->c[i] = transf_u16_bigend(data, 0);
    }
}

// ==============================================================================
// LEITURAS BRUTAS (Apenas barramento SPI, sem matemática)
// ==============================================================================

static inline void MS5611_ReadPress_Raw(void) {
    u8 cmd = MS5611_CMD_READ_ADC;
    u8 data[3];

    MS5611_Select();
    HAL_SPI_Transmit(ms5611_spi, &cmd, 1, 10);
    HAL_SPI_Receive(ms5611_spi, data, 3, 10);
    MS5611_Unselect();

    D1 = transf_u24_bigend(data, 0);
}

static inline void MS5611_ReadTemp_Raw(void) {
    u8 cmd = MS5611_CMD_READ_ADC;
    u8 data[3];

    MS5611_Select();
    HAL_SPI_Transmit(ms5611_spi, &cmd, 1, 10);
    HAL_SPI_Receive(ms5611_spi, data, 3, 10);
    MS5611_Unselect();

    D2_cache = transf_u24_bigend(data, 0);
}

// ==============================================================================
// MOTOR MATEMÁTICO (Roda sempre que D1 é atualizado)
// ==============================================================================
static void MS5611_CalcularTudo(void) {
	if (!d2_lido){ return; }

    i64 dT = (i64)D2_cache - ((i64)sensor_ptr->c[5] << 8);
    i64 TEMP = 2000 + ((dT * (i64)sensor_ptr->c[6]) >> 23);

    i64 OFF = ((i64)sensor_ptr->c[2] << 16) + (((i64)sensor_ptr->c[4] * dT) >> 7);
    i64 SENS = ((i64)sensor_ptr->c[1] << 15) + (((i64)sensor_ptr->c[3] * dT) >> 8);

    // Compensação Térmica Rigorosa (2ª Ordem)
    if (TEMP < 2000) {
        i64 T2 = (dT * dT) >> 31;
        i64 OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
        i64 SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;

        // Compensação extra para frio extremo (Abaixo de -15°C)
        if (TEMP < -1500) {
            OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500));
            SENS2 += 11 * ((TEMP + 1500) * (TEMP + 1500)) >> 1;
        }

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    // Com o OFF e SENS rigorosamente corrigidos, calculamos a Pressão
    i64 P = (((D1 * SENS) >> 21) - OFF) >> 15;

    // Atualiza a Struct
        sensor_ptr->temperatura = (float)TEMP / 100.0f;
        sensor_ptr->pressao = (float)P / 100.0f;

        // --- BLOQUEIO DE LIXO DO BARRAMENTO SPI ---
        // Se a pressão lida for irreal (vácuo ou absurdamente alta), descarta a leitura inteira.
        if (sensor_ptr->pressao < 10.0f || sensor_ptr->pressao > 1500.0f) {
            #ifdef VERBOSE
                if (HAL_GetTick() - time_verbose >= 1000) {
                    printlnRed("\r\n>>> GLITCH DO SENSOR IGNORADO (P: %.2f) <<<", sensor_ptr->pressao);
                    time_verbose = HAL_GetTick();
                }
            #endif
            return; // Aborta e não calcula a altitude!
        }

        // Se passou do if acima, a pressão é confiável.
        flag_novo_dado = 1;

        // --- CÁLCULO DE ALTITUDE (SÓ DEPOIS DE CALIBRADO) ---
        // Só tenta calcular a altitude se a calibração do solo já terminou (Pref > 0)
        if (sensor_ptr->pressao_ref > 0.0f) {
            sensor_ptr->altitude = 44330.0f * (1.0f - powf((sensor_ptr->pressao / sensor_ptr->pressao_ref), 0.190295f));

            #ifdef VERBOSE
                if (HAL_GetTick() - time_verbose >= 1000) {
                    printlnLGreen("\r\nMS5611: Pref %.2f; P %.2f; T %.2f; A %.2f", sensor_ptr->pressao_ref, sensor_ptr->pressao, sensor_ptr->temperatura, sensor_ptr->altitude);
                    time_verbose = HAL_GetTick();
                }
            #endif
        }

}

// ==============================================================================
// MÁQUINA DE ESTADOS PRINCIPAL (Chamada a cada 10ms pelo Timer)
// ==============================================================================
void MS5611_ReadData(void) {
    switch (estado_ms5611) {

        case MS_PEDIR_PRESSAO:
            // 1. Manda converter Pressão (D1)
            u8 cmd_p = MS5611_CMD_CONV_D1_OSR;
            MS5611_Select();
            HAL_SPI_Transmit(ms5611_spi, &cmd_p, 1, 10);
            MS5611_Unselect();

            estado_ms5611 = MS_LER_PRESSAO;
            break;

        case MS_LER_PRESSAO:
            // 2. Resgata a pressão crua (D1)
            MS5611_ReadPress_Raw();

            // 3. Processa TODA a matemática com o D1 atual e o D2 antigo
            MS5611_CalcularTudo();

            // 4. Verifica se está na hora de atualizar o D2 (Temperatura)
            if (temp_skip++ > 20) {
                u8 cmd_t = MS5611_CMD_CONV_D2_OSR;
                MS5611_Select();
                HAL_SPI_Transmit(ms5611_spi, &cmd_t, 1, 10);
                MS5611_Unselect();

                estado_ms5611 = MS_LER_TEMP_E_CALCULAR;
                temp_skip = 0;
            } else {
                estado_ms5611 = MS_PEDIR_PRESSAO; // Volta pro início do loop rápido
            }
            break;

        case MS_LER_TEMP_E_CALCULAR:
            // 5. Resgata a temperatura crua (D2)
            MS5611_ReadTemp_Raw();
            d2_lido = 1;
            // O próximo ciclo já vai usar o D2 novo para calcular a pressão,
            // então não precisamos forçar a matemática aqui de novo.
            estado_ms5611 = MS_PEDIR_PRESSAO;
            break;

        default:
            estado_ms5611 = MS_PEDIR_PRESSAO;
            break;
    }
}
