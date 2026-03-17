/*
  ******************************************************************************
  * @file    w25q.h
  * @date 	 3 de mar. de 2026
  * @author  Júnior Bandeira
  * @brief   Header do driver para o flash W25Q128.
  ******************************************************************************
*/

#ifndef W25Q_H
#define W25Q_H

#include "stm32f4xx_hal.h"
#include "utils.h"
#include "config_voo.h"


//#if defined(PROJETO_USA_RTC)

typedef struct {
    u8 hora;
    u8 min;
    u8 seg;
    float altitude;
    float pressao;
    float velocidade;
    EstadoSistema estado;
} LogData_t;

typedef enum {
    IDLE,
    CMD_WRITE,
    READY,
} W25Q_State_t;

extern const char* PRINT_ESTADO[];
extern W25Q_State_t sensor_state;


void w25qInit(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim, u8 W25Q_CS);
void adicionarLogW25Q(RTC_HandleTypeDef *hrtc, DadosVoo_t *dadosVoo);
void visualizarLogsW25Q(void);
void visualizarUltimoLogW25Q(void);
void pararGravacaoW25Q(void);
void apagarLogsW25Q(void);
void apagarTudoW25Q(void);

extern u32 currentAddr;
#define LIMIT 0x8700 //CORRIGIR
#define MAGIC_NUMBER_SIRIUS 0xAA55AA55 // Assinatura hexadecimal para validar se o bloco de memória realmente contém
// dados legítimos do voo. Evita que leituras de sensores corrompidas
// (que gerem 0xFFFFFFFF por acaso) enganem o sistema de varredura.



/*
#else

    #error  PROJETO_USA_RTC nao foi declarado! Por favor, declare-o no main.h para reconhecer o uso do RTC.

#endif
*/

#endif  /* W25Q_H */
