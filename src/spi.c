/*
 * spi.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "spi.h"

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#include "gpio.h"
#include "rcc_reg.h"
#include "spi_reg.h"
#include "types.h"

/*** SPI local macros ***/

#define SPI_ACCESS_TIMEOUT_COUNT    1000000

/*** SPI local structures ***/

/*******************************************************************/
typedef struct {
    SPI_registers_t* peripheral;
    volatile uint32_t* rcc_enr;
    uint32_t rcc_mask;
} SPI_descriptor_t;

/*******************************************************************/
typedef struct {
    uint8_t init_count[SPI_INSTANCE_LAST];
} SPI_context_t;

/*** SPI local global variables ***/

static const SPI_descriptor_t SPI_DESCRIPTOR[SPI_INSTANCE_LAST] = {
    { SPI1, &(RCC->APB2ENR), (0b1 << 12) },
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    { SPI2, &(RCC->APB1ENR), (0b1 << 14) },
#endif
};
static SPI_context_t spi_ctx = { .init_count = { 0 } };

/*** SPI local functions ***/

/*******************************************************************/
#define _SPI_write_read(instance, tx_data, rx_data, transfer_size, type) { \
    /* Local variables */ \
    SPI_status_t status = SPI_SUCCESS; \
    uint8_t transfer_idx = 0; \
    uint32_t loop_count = 0; \
    volatile type* spi_dr_ptr = ((volatile type*) &(SPI_DESCRIPTOR[instance].peripheral -> DR)); \
    /* Check instance */ \
    if (instance >= SPI_INSTANCE_LAST) { \
        status = SPI_ERROR_INSTANCE; \
        goto errors; \
    } \
    /* Check state */ \
    if (spi_ctx.init_count[instance] == 0) { \
        status = SPI_ERROR_UNINITIALIZED; \
        goto errors; \
    } \
    /* Check parameters */ \
    if ((tx_data == NULL) || (rx_data == NULL)) { \
        status = SPI_ERROR_NULL_PARAMETER; \
        goto errors; \
    } \
    /* Transfer loop */ \
    for (transfer_idx = 0; transfer_idx < transfer_size; transfer_idx++) { \
        /* Dummy read to DR to clear RXNE flag */ \
        rx_data[transfer_idx] = (*spi_dr_ptr); \
        /* Wait for TXE flag */ \
        while (((SPI_DESCRIPTOR[instance].peripheral -> SR) & (0b1 << 1)) == 0) { \
            /* Wait for TXE='1' or timeout */ \
            loop_count++; \
            if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) { \
                status = SPI_ERROR_TX_BUFFER_EMPTY; \
                goto errors; \
            } \
        } \
        /* Send TX data */ \
        (*spi_dr_ptr) = tx_data[transfer_idx]; \
        /* Wait for incoming data */ \
        loop_count = 0; \
        while (((SPI_DESCRIPTOR[instance].peripheral -> SR) & (0b1 << 0)) == 0) { \
            /* Wait for RXNE='1' or timeout */ \
            loop_count++; \
            if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) { \
                status = SPI_ERROR_RX_TIMEOUT; \
                goto errors; \
            } \
        } \
        rx_data[transfer_idx] = (*spi_dr_ptr); \
    } \
errors: \
    return status; \
}

/*******************************************************************/
#define _SPI_write(instance, tx_data, type) { \
    /* Send TX data */ \
    volatile type* spi_dr_ptr = ((volatile type*) &(SPI_DESCRIPTOR[instance].peripheral -> DR)); \
    (*spi_dr_ptr) = tx_data; \
    /* Wait for BSY flag */ \
    while (((SPI_DESCRIPTOR[instance].peripheral -> SR) & (0b1 << 7)) != 0); \
}

/*** SPI functions ***/

/*******************************************************************/
SPI_status_t SPI_init(SPI_instance_t instance, const SPI_gpio_t* pins, SPI_configuration_t* configuration) {
    // Local variables.
    SPI_status_t status = SPI_SUCCESS;
    // Check instance.
    if (instance >= SPI_INSTANCE_LAST) {
        status = SPI_ERROR_INSTANCE;
        goto errors;
    }
    // Check parameters.
    if ((pins == NULL) || (configuration == NULL)) {
        status = SPI_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Enable peripheral clock.
    (*SPI_DESCRIPTOR[instance].rcc_enr) |= SPI_DESCRIPTOR[instance].rcc_mask;
    // Disable peripheral during configuration.
    SPI_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 6); // SPE='0'.
    // Master mode (MSTR='1').
    SPI_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 2);
    // Data format.
    switch (configuration->data_format) {
    case SPI_DATA_FORMAT_8_BITS:
        SPI_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 11);
        break;
    case SPI_DATA_FORMAT_16_BITS:
        SPI_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 11);
        break;
    default:
        status = SPI_ERROR_DATA_FORMAT;
        goto errors;
    }
    // Set polarity.
    switch (configuration->clock_polarity) {
    case SPI_CLOCK_POLARITY_LOW:
        SPI_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b11 << 0); // CPOL='0' and CPHA='0'.
        break;
    case SPI_CLOCK_POLARITY_HIGH:
        SPI_DESCRIPTOR[instance].peripheral->CR1 |= (0b11 << 0); // CPOL='1' and CPHA='1'.
        break;
    default:
        status = SPI_ERROR_CLOCK_POLARITY;
        goto errors;
    }
    // Baud rate.
    if ((configuration->baud_rate_prescaler) >= SPI_BAUD_RATE_PRESCALER_LAST) {
        status = SPI_ERROR_BAUD_RATE_PRESCALER;
        goto errors;
    }
    SPI_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b111 << 3);
    SPI_DESCRIPTOR[instance].peripheral->CR1 |= ((configuration->baud_rate_prescaler) << 3);
    // Enable output (SSOE='1').
    SPI_DESCRIPTOR[instance].peripheral->CR2 |= (0b1 << 2);
    // Enable peripheral.
    SPI_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 6); // SPE='1'.
    // Configure GPIOs.
    GPIO_configure((pins->sclk), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
    GPIO_configure((pins->mosi), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
    GPIO_configure((pins->miso), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
    // Update initialization count.
    spi_ctx.init_count[instance]++;
errors:
    return status;
}

/*******************************************************************/
SPI_status_t SPI_de_init(SPI_instance_t instance, const SPI_gpio_t* pins) {
    // Local variables.
    SPI_status_t status = SPI_SUCCESS;
    // Check instance.
    if (instance >= SPI_INSTANCE_LAST) {
        status = SPI_ERROR_INSTANCE;
        goto errors;
    }
    // Check parameters.
    if (pins == NULL) {
        status = SPI_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Update initialization count.
    if (spi_ctx.init_count[instance] > 0) {
        spi_ctx.init_count[instance]--;
    }
    // Check initialization count.
    if (spi_ctx.init_count[instance] > 0) goto errors;
    // Disable SPI alternate function.
    GPIO_configure((pins->sclk), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->mosi), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->miso), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Disable peripheral.
    SPI_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 6); // SPE='0'.
    // Disable peripheral clock.
    (*SPI_DESCRIPTOR[instance].rcc_enr) &= ~(SPI_DESCRIPTOR[instance].rcc_mask);
errors:
    return status;
}

/*******************************************************************/
SPI_status_t SPI_write_read_8(SPI_instance_t instance, uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size) {
    _SPI_write_read(instance, tx_data, rx_data, transfer_size, uint8_t);
}

/*******************************************************************/
SPI_status_t SPI_write_read_16(SPI_instance_t instance, uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size) {
    _SPI_write_read(instance, tx_data, rx_data, transfer_size, uint16_t);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) SPI_write_8(SPI_instance_t instance, uint8_t tx_data) {
    _SPI_write(instance, tx_data, uint8_t);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) SPI_write_16(SPI_instance_t instance, uint16_t tx_data) {
    _SPI_write(instance, tx_data, uint16_t);
}
