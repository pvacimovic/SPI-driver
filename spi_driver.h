/*
 * spi_driver.h
 *
 *      Author: Petar
 * 
 */

#ifndef STM32F4XX_HAL_DRIVER_INC_SPI_DRIVER_H_
#define STM32F4XX_HAL_DRIVER_INC_SPI_DRIVER_H_

// #include <stdlib.h>
// #include <stdint.h>
#include "stm32f446xx.h"

/*
 * SPI Registers
 */
typedef struct
{
    volatile uint32_t CR1;      /* Address offset: 0x00 */
    volatile uint32_t CR2;      /* Address offset: 0x04 */
    volatile uint32_t SR;       /* Address offset: 0x08 */
    volatile uint32_t DR;       /* Address offset: 0x0C */
    volatile uint32_t CRCPR;    /* Address offset: 0x10 */
    volatile uint32_t RXCRCR;   /* Address offset: 0x14 */
    volatile uint32_t TXCRCR;   /* Address offset: 0x18 */
    volatile uint32_t I2SCFGR;  /* Address offset: 0x1C */
    volatile uint32_t I2SPR;    /* Address offset: 0x20 */
}SPI_Regs;

/*
 * SPI bases
 */
#define SPI1 (SPI_Regs*)(0x40013000)
#define SPI2 (SPI_Regs*)(0x40003800)
#define SPI3 (SPI_Regs*)(0x40003C00)
#define SPI4 (SPI_Regs*)(0x40013400)

/* macros */
#define OK 0
#define WRONG_ADDRESS 1
#define UNDEFINED_INPUT 2
#define NULL_ERROR 3
#define ENABLE 1
#define DISABLE 0
#define SPI_READY 0
#define SPI_RECEIVING 1
#define SPI_TRANSMITING 2

/*
 * SPI Configuration
 */
typedef struct
{
    uint8_t BusConfig;      /* Bus Config */
    uint8_t SclkSpeed;      /* Clock speed*/
    uint8_t DFF;            /* Data frame format (8 or 16 bits)*/
    uint8_t CPOL;           /* Clock polarity (1 - high IDLE, 0 - low IDLE) */
    uint8_t CPHA;           /* Clock phase (1 - trailing edge, 0 - leading edge) */
    uint8_t SSM;            /* 1 - software client management, 0 - hardware client management*/
}SPI_Config;

/* Config macros */
#define BUS_FD 1
#define BUS_HD 2
#define BUS_SIM_RX 3
#define SCLK_SPEED_2 0
#define SCLK_SPEED_4 1
#define SCLK_SPEED_8 2
#define DFF_8BIT 0
#define DFF_16BIT 1
#define CPOL_LOW 0
#define CPOL_HIGH 1
#define CPHA_LOW 0
#define CPHA_HIGH 1
#define SSM_HW 0
#define SSM_SF 1

/* Interrupts */
#define SPI1_IRQn 35
#define SPI2_IRQn 36
#define SPI3_IRQn 51
#define SPI3_IRQn 84
#define NVIC_ISER0 ((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1 ((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2 ((volatile uint32_t*)0xE000E108)
#define NVIC_ICER0 ((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1 ((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2 ((volatile uint32_t*)0xE000E188)
#define NVIC_PR ((volatile uint32_t*)0xE000E400)

/*
 * Handle structure
 */
typedef struct
{
    SPI_Regs *p_SPIx;
    SPI_Config *p_SPI_Config;
    uint8_t *p_tx_buffer;
    uint8_t *p_rx_buffer;
    uint32_t tx_length;
    uint32_t rx_length;
    uint8_t tx_state;
    uint8_t rx_state;
    
}SPI_Handle;

/*
 * Functions
 */
int SPI_Clk_Enable(SPI_Regs *p_SPI);
int SPI_Init(SPI_Handle *p_SPI_Handle);

/* General functions, info from cortex m4 generic user guide */
void Interrupt_Config(uint8_t IRQn, uint8_t action);
void Priority_Config(uint8_t IRQn, uint8_t priority);

/* Blocking type (no interrupt) */
int SPI_Transmit_Data(SPI_Regs *p_SPI, uint8_t *p_tx_buffer, uint32_t length);
int SPI_Receive_Data(SPI_Regs *p_SPI, uint8_t *p_rx_buffer, uint32_t length);

/* Non blocking type */
int SPI_Transmit_Data_Interrupt(SPI_Handle *p_SPI_Handle, uint8_t *p_tx_buffer, uint32_t length);
int SPI_Receive_Data_Interrupt(SPI_Handle *p_SPI_Handle, uint8_t *p_rx_buffer, uint32_t length);
void SPI_IRQ_Handler(SPI_Handle *p_SPI_Handle);

/* Help functions */
uint8_t Get_StatusRegister_BitInfo(SPI_Regs *p_SPI, uint8_t bit_num);

#endif /* STM32F4XX_HAL_DRIVER_INC_SPI_DRIVER_H_ */
