/*
 * dma.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "dma.h"

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "dma_registers.h"
#include "nvic.h"
#include "rcc_registers.h"
#include "types.h"

/*** DMA local macros ***/

#define DMA_REQUEST_NUMBER_MAX                  15

#define DMA_NVIC_SHARED_CHANNEL_MASK_CH1        0x01
#define DMA_NVIC_SHARED_CHANNEL_MASK_CH2_CH3    0x06
#define DMA_NVIC_SHARED_CHANNEL_MASK_CH4_CH7    0x78

/*** DMA local structures ***/

/*******************************************************************/
typedef struct {
    NVIC_interrupt_t nvic_interrupt;
    uint8_t nvic_shared_mask;
} DMA_descriptor_t;

/*******************************************************************/
typedef struct {
    uint8_t enabled_channels_mask;
    uint8_t started_channels_mask;
    DMA_transfer_complete_irq_cb_t channel_irq_callbacks[DMA_CHANNEL_LAST];
} DMA_context_t;

/*** DMA local global variables ***/

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)
static const DMA_descriptor_t DMA_DESCRIPTOR[DMA_CHANNEL_LAST] = {
    { NVIC_INTERRUPT_DMA1_CH_1,   DMA_NVIC_SHARED_CHANNEL_MASK_CH1 },
    { NVIC_INTERRUPT_DMA1_CH_2_3, DMA_NVIC_SHARED_CHANNEL_MASK_CH2_CH3 },
    { NVIC_INTERRUPT_DMA1_CH_2_3, DMA_NVIC_SHARED_CHANNEL_MASK_CH2_CH3 },
    { NVIC_INTERRUPT_DMA1_CH_4_7, DMA_NVIC_SHARED_CHANNEL_MASK_CH4_CH7 },
    { NVIC_INTERRUPT_DMA1_CH_4_7, DMA_NVIC_SHARED_CHANNEL_MASK_CH4_CH7 },
#if (STM32L0XX_REGISTERS_MCU_CATEGORY > 1)
    { NVIC_INTERRUPT_DMA1_CH_4_7, DMA_NVIC_SHARED_CHANNEL_MASK_CH4_CH7 },
    { NVIC_INTERRUPT_DMA1_CH_4_7, DMA_NVIC_SHARED_CHANNEL_MASK_CH4_CH7 },
#endif
};
static DMA_context_t dma_ctx = { .enabled_channels_mask = 0, .started_channels_mask = 0, .channel_irq_callbacks = { NULL } };
#endif

/*** DMA local functions ***/

/*******************************************************************/
#define _DMA_check_channel(void) { \
    if (channel >= DMA_CHANNEL_LAST) { \
        status = DMA_ERROR_CHANNEL; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _DMA_check_channel_state(void) { \
    if ((dma_ctx.enabled_channels_mask & (0b1 << channel)) == 0) { \
        status = DMA_ERROR_UNINITIALIZED; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _DMA_irq_handler(channel) { \
    /* Check flag */ \
    if (((DMA1->ISR) & (0b1 << ((channel << 2) + 1))) != 0) { \
        /* Check mask and callback */ \
        if ((((DMA1->CHx[channel].CCR) & (0b1 << 1)) != 0) && (dma_ctx.channel_irq_callbacks[channel] != NULL)) { \
            /* Execute callback */ \
            dma_ctx.channel_irq_callbacks[channel](); \
        } \
        /* Clear flag */ \
        DMA1->IFCR |= (0b1 << ((channel << 2) + 1)); \
    } \
}

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_NVIC_SHARED_CHANNEL_MASK_CH1) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_Channel1_IRQHandler(void) {
#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_CH1) != 0)
    _DMA_irq_handler(DMA_CHANNEL_1);
#endif
}
#endif

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_NVIC_SHARED_CHANNEL_MASK_CH2_CH3) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_Channel2_3_IRQHandler(void) {
#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_CH2) != 0)
    _DMA_irq_handler(DMA_CHANNEL_2);
#endif
#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_CH3) != 0)
    _DMA_irq_handler(DMA_CHANNEL_3);
#endif
}
#endif

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_NVIC_SHARED_CHANNEL_MASK_CH4_CH7) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) DMA1_Channel4_5_6_7_IRQHandler(void) {
#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_CH4) != 0)
    _DMA_irq_handler(DMA_CHANNEL_4);
#endif
#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_CH5) != 0)
    _DMA_irq_handler(DMA_CHANNEL_5);
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY > 1)
#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_CH6) != 0)
    _DMA_irq_handler(DMA_CHANNEL_6);
#endif
#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_CH7) != 0)
    _DMA_irq_handler(DMA_CHANNEL_7);
#endif
#endif
}
#endif

/*** DMA functions ***/

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)
/*******************************************************************/
DMA_status_t DMA_init(DMA_channel_t channel, DMA_configuration_t* configuration) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check channel.
    _DMA_check_channel();
    // Check parameters.
    if (configuration == NULL) {
        status = DMA_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if ((configuration->request_number) > DMA_REQUEST_NUMBER_MAX) {
        status = DMA_ERROR_REQUEST_NUMBER;
        goto errors;
    }
    // Enable peripheral clock.
    RCC->AHBENR |= (0b1 << 0);
    // Reset configuration register.
    DMA1->CHx[channel].CCR = 0;
    // Direction.
    switch (configuration->direction) {
    case DMA_DIRECTION_PERIPHERAL_TO_MEMORY:
        // Nothing to do.
        break;
    case DMA_DIRECTION_MEMORY_TO_PERIPHERAL:
        DMA1->CHx[channel].CCR |= (0b1 << 4);
        break;
    default:
        status = DMA_ERROR_DIRECTION;
        goto errors;
    }
    // Flags.
    DMA1->CHx[channel].CCR |= ((configuration->flags).circular_mode << 5);
    DMA1->CHx[channel].CCR |= ((configuration->flags).peripheral_increment << 6);
    DMA1->CHx[channel].CCR |= ((configuration->flags).memory_increment << 7);
    // Memory data size.
    switch (configuration->memory_data_size) {
    case DMA_DATA_SIZE_8_BITS:
        // Nothing to do.
        break;
    case DMA_DATA_SIZE_16_BITS:
        DMA1->CHx[channel].CCR |= (0b01 << 10);
        break;
    case DMA_DATA_SIZE_32_BITS:
        DMA1->CHx[channel].CCR |= (0b10 << 10);
        break;
    default:
        status = DMA_ERROR_MEMORY_DATA_SIZE;
        goto errors;
    }
    // Peripheral data size.
    switch (configuration->peripheral_data_size) {
    case DMA_DATA_SIZE_8_BITS:
        // Nothing to do.
        break;
    case DMA_DATA_SIZE_16_BITS:
        DMA1->CHx[channel].CCR |= (0b01 << 8);
        break;
    case DMA_DATA_SIZE_32_BITS:
        DMA1->CHx[channel].CCR |= (0b10 << 8);
        break;
    default:
        status = DMA_ERROR_PERIPHERAL_DATA_SIZE;
        goto errors;
    }
    // Priority.
    switch (configuration->priority) {
    case DMA_PRIORITY_LOW:
        // Nothing to do.
        break;
    case DMA_PRIORITY_MEDIUM:
        DMA1->CHx[channel].CCR |= (0b01 << 12);
        break;
    case DMA_PRIORITY_HIGH:
        DMA1->CHx[channel].CCR |= (0b10 << 12);
        break;
    case DMA_PRIORITY_VERY_HIGH:
        DMA1->CHx[channel].CCR |= (0b11 << 12);
        break;
    default:
        status = DMA_ERROR_PRIORITY;
        goto errors;
    }
    // Number of data.
    DMA1->CHx[channel].CNDTR = (configuration->number_of_data);
    // Memory address.
    DMA1->CHx[channel].CMAR = (configuration->memory_address);
    // Peripheral address.
    DMA1->CHx[channel].CPAR = (configuration->peripheral_address);
    // Channel selection.
    DMA1->CSELR &= ~(0b1111 << (channel << 2));
    DMA1->CSELR |= ((configuration->request_number) << (channel << 2));
    // Set interrupt priority.
    NVIC_set_priority(DMA_DESCRIPTOR[channel].nvic_interrupt, (configuration->nvic_priority));
    // Register callback.
    dma_ctx.channel_irq_callbacks[channel] = (configuration->irq_callback);
    // Enable transfer complete interrupt.
    DMA1->CHx[channel].CCR |= (0b1 << 1);
    // Update mask.
    dma_ctx.enabled_channels_mask |= (0b1 << channel);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)
/*******************************************************************/
DMA_status_t DMA_de_init(DMA_channel_t channel) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check channel.
    _DMA_check_channel();
    // Disable channel.
    status = DMA_stop(channel);
    // Update mask.
    dma_ctx.enabled_channels_mask &= ~(0b1 << channel);
    dma_ctx.started_channels_mask &= ~(0b1 << channel);
    // Disable peripheral clock.
    if (dma_ctx.enabled_channels_mask == 0) {
        RCC->AHBENR &= ~(0b1 << 0);
    }
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)
/*******************************************************************/
DMA_status_t DMA_start(DMA_channel_t channel) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check channel.
    _DMA_check_channel();
    _DMA_check_channel_state();
    // Clear all flags.
    DMA1->IFCR |= (0b1111 << (channel << 2));
    // Enable interrupt.
    NVIC_enable_interrupt(DMA_DESCRIPTOR[channel].nvic_interrupt);
    // Update mask.
    dma_ctx.started_channels_mask |= (0b1 << channel);
    // Start transfer.
    DMA1->CHx[channel].CCR |= (0b1 << 0);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)
/*******************************************************************/
DMA_status_t DMA_stop(DMA_channel_t channel) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check channel.
    _DMA_check_channel();
    _DMA_check_channel_state();
    // Update mask.
    dma_ctx.started_channels_mask &= ~(0b1 << channel);
    // Stop transfer.
    DMA1->CHx[channel].CCR &= ~(0b1 << 0);
    // Disable interrupt.
    if ((dma_ctx.started_channels_mask & DMA_DESCRIPTOR[channel].nvic_shared_mask) == 0) {
        NVIC_disable_interrupt(DMA_DESCRIPTOR[channel].nvic_interrupt);
    }
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)
/*******************************************************************/
DMA_status_t DMA_set_memory_address(DMA_channel_t channel, uint32_t memory_addr, uint16_t number_of_data) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check channel.
    _DMA_check_channel();
    _DMA_check_channel_state();
    // Set memory address and transfer size.
    DMA1->CHx[channel].CMAR = memory_addr;
    DMA1->CHx[channel].CNDTR = number_of_data;
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_DMA_CHANNEL_MASK & DMA_CHANNEL_MASK_ALL) != 0)
/*******************************************************************/
DMA_status_t DMA_set_peripheral_address(DMA_channel_t channel, uint32_t peripheral_addr, uint16_t number_of_data) {
    // Local variables.
    DMA_status_t status = DMA_SUCCESS;
    // Check channel.
    _DMA_check_channel();
    _DMA_check_channel_state();
    // Set memory address and transfer size.
    DMA1->CHx[channel].CPAR = peripheral_addr;
    DMA1->CHx[channel].CNDTR = number_of_data;
errors:
    return status;
}
#endif
