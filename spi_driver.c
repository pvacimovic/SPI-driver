/*
 * spi_driver.c
 *
 *  Author: Petar
 * 
 */

#include "spi_driver.h"

uint8_t Get_StatusRegister_BitInfo(SPI_Regs *p_SPI, uint8_t bit_num)
{
    return (p_SPI->SR & (1 << bit_num));
}

int SPI_Clk_Enable(SPI_Regs *p_SPI)
{
    if(p_SPI == SPI1)
    {
        RCC->APB2ENR |= (1 << 12);
    }
    else if(p_SPI == SPI2)
    {
        RCC->APB1ENR |= (1 << 14);
    }
    else if(p_SPI == SPI3)
    {
        RCC->APB1ENR |= (1 << 15);
    }
    else if(p_SPI == SPI4)
    {
        RCC->APB2ENR |= (1 << 13);
    }
    else
        return WRONG_ADDRESS;
    return OK;
}

int SPI_Init(SPI_Handle *p_SPI_Handle)
{
    // control register 1
    uint32_t spi_cr1 = 0;

    /* Bus Config */
    if(p_SPI_Handle->p_SPI_Config->BusConfig == BUS_FD)
    {
        // BIDIMODE 0
        spi_cr1 &= ~(1 << 15);
    }
    else if(p_SPI_Handle->p_SPI_Config->BusConfig == BUS_HD)
    {
        // BIDIMODE 1
        spi_cr1 |= 1 << 15;
    }
    else if(p_SPI_Handle->p_SPI_Config->BusConfig == BUS_SIM_RX)
    {
        // BIDIMODE 0, RXONLY 1
        spi_cr1 &= ~(1 << 15);
        spi_cr1 |= 1 << 10;
    }
    else
    {
        return UNDEFINED_INPUT;
    }

    /* Clock Speed */
    spi_cr1 |= p_SPI_Handle->p_SPI_Config->SclkSpeed << 3;

    /* DFF */
    spi_cr1 |= p_SPI_Handle->p_SPI_Config->DFF << 11;

    /* CPOL */
    spi_cr1 |= p_SPI_Handle->p_SPI_Config->CPOL << 1;

    /* CPHA */
    spi_cr1 |= p_SPI_Handle->p_SPI_Config->CPHA << 0;

    /* SSM */
    spi_cr1 |= p_SPI_Handle->p_SPI_Config->SSM << 9;

    /* put the new value of cr1 register in the register */
    p_SPI_Handle->p_SPIx->CR1 = spi_cr1;

    return OK;
}

void Interrupt_Config(uint8_t IRQn, uint8_t action)
{
    if(action == ENABLE)
    {
        if(IRQn < 32)
        {
            // ISER0
            *NVIC_ISER0 |= (1<<IRQn);
        }
        else if(IRQn < 64)
        {
            // ISER1
            *NVIC_ISER1 |= (1 << (IRQn%32));
        }
        else if(IRQn < 96)
        {
            // ISER2
            *NVIC_ISER2 |= (1 << (IRQn%64));
        }
    }
    else
    {
        if(IRQn < 32)
        {
            // ICER0
            *NVIC_ICER0 |= (1<<IRQn);
        }
        else if(IRQn < 64)
        {
            // ICER1
            *NVIC_ICER1 |= (1 << (IRQn%32));
        }
        else if(IRQn < 96)
        {
            // ICER2
            *NVIC_ICER2 |= (1 << (IRQn%64));
        }
    }
}
void Priority_Config(uint8_t IRQn, uint8_t priority)
{
    uint8_t IPRn = IRQn / 4;

    *(NVIC_PR + (IPRn * 4)) |= priority << (8 * (IRQn % 4) + 4);
}

// Blocking type (no interrupt)
int SPI_Transmit_Data(SPI_Regs *p_SPI, uint8_t *p_tx_buffer, uint32_t length)
{
    if(p_tx_buffer == NULL)
    {
        return NULL_ERROR;
    }
    if(length <= 0)
    {
        return UNDEFINED_INPUT;
    }

    while(length > 0)
    {
        /* wait if the TXE bit in status register is not 1 (not empty)*/
        while(!Get_StatusRegister_BitInfo(p_SPI, 1))
        {
        }

        /* check DFF to see the format */
        if(p_SPI->CR1 & (1 << 11))
        {
            // 16-bit
            p_SPI->DR = *((uint16_t*)p_tx_buffer); // fill the register with data
            length = length - 2;
            (uint16_t*)p_tx_buffer++;
        }
        else
        {
            // 8-bit
            p_SPI->DR = *(p_tx_buffer); // fill the register with data
            length = length - 1;
            p_tx_buffer++;
        }
    }
    return OK;
}

int SPI_Receive_Data(SPI_Regs *p_SPI, uint8_t *p_rx_buffer, uint32_t length)
{
    if(p_rx_buffer == NULL)
    {
        return NULL_ERROR;
    }
    if(length <= 0)
    {
        return UNDEFINED_INPUT;
    }

    while(length > 0)
    {
        /* wait if the RXE bit in status register is not 1 (not empty)*/
        while(!Get_StatusRegister_BitInfo(p_SPI, 0))
        {
        }

        /* check DFF to see the format */
        if(p_SPI->CR1 & (1 << 11))
        {
            // 16-bit
            *((uint16_t*)p_rx_buffer) = p_SPI->DR; // get data from the register
            length = length - 2;
            (uint16_t*)p_rx_buffer++;
        }
        else
        {
            // 8-bit
            *p_rx_buffer = p_SPI->DR; // get data from the register
            length = length - 1;
            p_rx_buffer++;
        }
    }
    return OK;
}

int SPI_Transmit_Data_Interrupt(SPI_Handle *p_SPI_Handle, uint8_t *p_tx_buffer, uint32_t length)
{
    if(p_SPI_Handle->tx_state != SPI_TRANSMITING)
    {
        p_SPI_Handle->p_tx_buffer = p_tx_buffer;
        p_SPI_Handle->tx_length = length;

        // SPI is busy
        p_SPI_Handle->tx_state = SPI_TRANSMITING;

        // Tx buffer empty interrupt enable
        p_SPI_Handle->p_SPIx->CR2 |= 1 << 7;

        // ready for transmition (IRQ handles)
    }
    return (p_SPI_Handle->tx_state);
}

int SPI_Receive_Data_Interrupt(SPI_Handle *p_SPI_Handle, uint8_t *p_rx_buffer, uint32_t length)
{
    if(p_SPI_Handle->rx_state != SPI_RECEIVING)
    {
        p_SPI_Handle->p_rx_buffer = p_rx_buffer;
        p_SPI_Handle->rx_length = length;

        // SPI is busy
        p_SPI_Handle->rx_state = SPI_RECEIVING;

        //  RX buffer not empty interrupt enable
        p_SPI_Handle->p_SPIx->CR2 |= 1 << 6;

        // ready for receiving (IRQ handles)
    }
    return (p_SPI_Handle->rx_state);
}

void SPI_IRQ_Handler(SPI_Handle *p_SPI_Handle)
{
    // figure out what event triggered the interrupt
    
    uint8_t tx_buffer_empty = p_SPI_Handle->p_SPIx->SR & (1<<1);
    uint8_t tx_buffer_empty_int_enabled = p_SPI_Handle->p_SPIx->CR2 & (1<<7);

    if(tx_buffer_empty && tx_buffer_empty_int_enabled)
    {
        // do transmition
        SPI_tx_int_handler(p_SPI_Handle);
    }

    uint8_t rx_buffer_empty = p_SPI_Handle->p_SPIx->SR & (1<<0);
    uint8_t rx_buffer_empty_int_enabled = p_SPI_Handle->p_SPIx->CR2 & (1<<6);

    if(rx_buffer_empty && rx_buffer_empty_int_enabled)
    {
        // receiving
        SPI_rx_int_handler(p_SPI_Handle);
    }

    // check overrun (completing the reception of new data while reading the previous frame from rx is not done)
    uint8_t ovr_flag = p_SPI_Handle->p_SPIx->SR & (1<<6);
    uint8_t error_int_enabled = p_SPI_Handle->p_SPIx->CR2 & (1<<5);

    if(ovr_flag && error_int_enabled && !(p_SPI_Handle->tx_state))
    {
        /* Clearing the OVR bit is done by a read access to the SPI_DR register
         * followed by a read access to the SPI_SR register. */
        uint32_t read_access;
        read_access = p_SPI_Handle->p_SPIx->DR;
        read_access = p_SPI_Handle->p_SPIx->SR;
    }
}

// do not use anywhere else
static void SPI_tx_int_handler(SPI_Handle *p_SPI_Handle);
static void SPI_rx_int_handler(SPI_Handle *p_SPI_Handle);

void SPI_tx_int_handler(SPI_Handle *p_SPI_Handle)
{
    /* check DFF to see the format */
    if(p_SPI_Handle->p_SPIx->CR1 & (1 << 11))
    {
        // 16-bit
        p_SPI_Handle->p_SPIx->DR = *((uint16_t*)(p_SPI_Handle->p_tx_buffer)); // fill the register with data
        p_SPI_Handle->tx_length = p_SPI_Handle->tx_length - 2;
        (uint16_t*)(p_SPI_Handle->p_tx_buffer)++;
    }
    else
    {
        // 8-bit
        p_SPI_Handle->p_SPIx->DR = *(p_SPI_Handle->p_tx_buffer); // fill the register with data
        p_SPI_Handle->tx_length = p_SPI_Handle->tx_length - 1;
        (p_SPI_Handle->p_tx_buffer)++;
    }
    if(p_SPI_Handle->tx_length == 0)
    {
        // transmition is over
        p_SPI_Handle->p_SPIx->CR2 &= ~(1 << 7); // stop interrupts setting the flag
        p_SPI_Handle->p_tx_buffer = NULL;
        p_SPI_Handle->tx_state = SPI_READY;
    }
}

void SPI_rx_int_handler(SPI_Handle *p_SPI_Handle)
{
        /* check DFF to see the format */
        if(p_SPI_Handle->p_SPIx->CR1 & (1 << 11))
        {
            // 16-bit
            *((uint16_t*)p_SPI_Handle->p_rx_buffer) = p_SPI_Handle->p_SPIx->DR; // get data from the register
            p_SPI_Handle->rx_length = p_SPI_Handle->rx_length - 2;
            (uint16_t*)(p_SPI_Handle->p_rx_buffer)++;
        }
        else
        {
            // 8-bit
            *p_SPI_Handle->p_rx_buffer = p_SPI_Handle->p_SPIx->DR; // get data from the register
            p_SPI_Handle->rx_length = p_SPI_Handle->rx_length - 1;
            (p_SPI_Handle->p_rx_buffer)++;
        }
        if(p_SPI_Handle->rx_length == 0)
        {
            // receiving is over
            p_SPI_Handle->p_SPIx->CR2 &= ~(1 << 6); // stop interrupts setting the flag
            p_SPI_Handle->p_rx_buffer = NULL;
            p_SPI_Handle->rx_state = SPI_READY;
        }
}