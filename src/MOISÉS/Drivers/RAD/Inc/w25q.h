/*
  ******************************************************************************
  * @file    w25q.h
  * @date 	 3 de mar. de 2026
  * @author  Junior Bandeira
  * @brief   Header do driver para o Flash W25Q128.
  ******************************************************************************
*/

#ifndef W25Q_H
#define W25Q_H

#include <stm32f4xx_hal.h>
#include <utils.h>
#include <config_voo.h>


typedef struct {
    u8 hora;
    u8 min;
    u8 seg;
    u8 estado;          // Fazer casting para (u8) no enum EstadoSistema do .c
    float pressao;
    float temperatura;
    float altitude;
    float velocidade;
    u32 _reserved[3];  // Padding para preencher 32 bytes.
} __attribute__((aligned(4))) LogData_t; // Total: 32 bytes.

typedef enum {
    IDLE,
    CMD_WRITE,
    READY,
} W25Q_State_t;

extern const char* PRINT_ESTADO[];
extern W25Q_State_t sensor_state;


void w25qInit(SPI_HandleTypeDef *hspi, TIM_HandleTypeDef *htim, GPIO_TypeDef *port, u16 pin_mask);
void adicionarLogW25Q(RTC_HandleTypeDef *hrtc, DadosVoo_t *dadosVoo);
void visualizarLogsW25Q(void);
void visualizarUltimoLogW25Q(void);
void descarregarBuffer(void);
void apagarLogsW25Q(void);
void apagarTudoW25Q(void);
void salvarCaixaPretaW25Q(float pressao_ref, uint32_t status);
CaixaPreta_t lerCaixaPretaW25Q(void);
void gerarVooSimuladoW25Q(void);

extern u32 currentAddr;

// 4 MB garante cerca de 34 minutos de telemetria contínua a 100Hz.
#define LIMIT 0x400000 // 4 Megabytes (O tamanho máximo do módulo é de 16 Megabytes)
#define MAGIC_NUMBER_SIRIUS 0xAA55AA55 // Assinatura hexadecimal para validar se o bloco de memória realmente contém
// dados legítimos do voo. Evita que leituras de sensores corrompidas
// (que gerem 0xFFFFFFFF por acaso) enganem o sistema de varredura.



#endif  /* W25Q_H */
