/*
 * usart.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "usart.h"

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "exti.h"
#include "gpio.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "types.h"
#include "usart_reg.h"

/*** USART local macros ***/

#define USART_TIMEOUT_COUNT     100000

/*** USART local structures ***/

/*******************************************************************/
typedef struct {
    USART_registers_t* peripheral;
    volatile uint32_t* rcc_enr;
    volatile uint32_t* rcc_smenr;
    uint32_t rcc_mask;
    RCC_clock_t clock_source;
    NVIC_interrupt_t nvic_interrupt;
} USART_descriptor_t;

/*******************************************************************/
typedef struct {
    uint8_t init_count[USART_INSTANCE_LAST];
#if (STM32L0XX_DRIVERS_USART_MODE == 0)
    USART_rx_irq_cb_t rxne_callback[USART_INSTANCE_LAST];
#endif
#if (STM32L0XX_DRIVERS_USART_MODE == 1)
    USART_character_match_irq_cb_t cmf_callback[USART_INSTANCE_LAST];
#endif
} USART_context_t;

/*** USART local global variables ***/

// @formatter:off
static const USART_descriptor_t USART_DESCRIPTOR[USART_INSTANCE_LAST] = {
    { USART2, &(RCC->APB1ENR), &(RCC->APB1SMENR), (0b1 << 17), RCC_CLOCK_HSI, NVIC_INTERRUPT_USART2 },
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    { USART1, &(RCC->APB2ENR), &(RCC->APB2SMENR), (0b1 << 14), RCC_CLOCK_HSI, NVIC_INTERRUPT_USART1 },
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    { USART4, &(RCC->APB1ENR), &(RCC->APB1SMENR), (0b1 << 19), RCC_CLOCK_SYSTEM, NVIC_INTERRUPT_USART4_USART5 },
    { USART5, &(RCC->APB1ENR), &(RCC->APB1SMENR), (0b1 << 20), RCC_CLOCK_SYSTEM, NVIC_INTERRUPT_USART4_USART5 },
#endif
};
static USART_context_t usart_ctx = {
    .init_count = { 0 },
#if (STM32L0XX_DRIVERS_USART_MODE == 0)
    .rxne_callback = { NULL },
#endif
#if (STM32L0XX_DRIVERS_USART_MODE == 1)
    .cmf_callback = { NULL },
#endif
};
// @formatter:on
/*** USART local functions ***/

#if (STM32L0XX_DRIVERS_USART_MODE == 0)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _USART_irq_handler_rxne(USART_instance_t instance, USART_registers_t* peripheral) {
    // Local variables.
    uint8_t rx_byte = 0;
    // RXNE interrupt.
    if (((peripheral->ISR) & (0b1 << 5)) != 0) {
        // Read incoming byte.
        rx_byte = (peripheral->RDR);
        // Transmit byte to upper layer.
        if ((((peripheral->CR1) & (0b1 << 5)) != 0) && (usart_ctx.rxne_callback[instance] != NULL)) {
            usart_ctx.rxne_callback[instance](rx_byte);
        }
    }
}
#endif

#if (STM32L0XX_DRIVERS_USART_MODE == 1)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _USART_irq_handler_cmf(USART_instance_t instance, USART_registers_t* peripheral) {
    // CMF interrupt.
    if (((peripheral->ISR) & (0b1 << 17)) != 0) {
        // Notify upper layer.
        if ((((peripheral->CR1) & (0b1 << 14)) != 0) && (usart_ctx.cmf_callback[instance] != NULL)) {
            usart_ctx.cmf_callback[instance]();
        }
        // Clear CM flag.
        peripheral->ICR |= (0b1 << 17);
    }
}
#endif

/*******************************************************************/
void __attribute__((optimize("-O0"))) USART2_IRQHandler(void) {
    // Execute internal callback.
#if (STM32L0XX_DRIVERS_USART_MODE == 0)
    _USART_irq_handler_rxne(USART_INSTANCE_USART2, USART2);
#endif
#if (STM32L0XX_DRIVERS_USART_MODE == 1)
    _USART_irq_handler_cmf(USART_INSTANCE_USART2, USART2);
#endif
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_USART2);
}

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) USART1_IRQHandler(void) {
    // Execute internal callback.
#if (STM32L0XX_DRIVERS_USART_MODE == 0)
    _USART_irq_handler_rxne(USART_INSTANCE_USART1, USART1);
#endif
#if (STM32L0XX_DRIVERS_USART_MODE == 1)
    _USART_irq_handler_cmf(USART_INSTANCE_USART1, USART1);
#endif
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_USART1);
}
#endif

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) USART4_5_IRQHandler(void) {
    // Execute internal callback.
#if (STM32L0XX_DRIVERS_USART_MODE == 0)
    _USART_irq_handler_rxne(USART_INSTANCE_USART4, USART4);
    _USART_irq_handler_rxne(USART_INSTANCE_USART5, USART5);
#endif
#if (STM32L0XX_DRIVERS_USART_MODE == 1)
    _USART_irq_handler_cmf(USART_INSTANCE_USART4, USART4);
    _USART_irq_handler_cmf(USART_INSTANCE_USART5, USART5);
#endif
}
#endif

/*** USART functions ***/

/*******************************************************************/
USART_status_t USART_init(USART_instance_t instance, const USART_gpio_t* pins, USART_configuration_t* configuration) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    uint32_t usart_clock_hz = 0;
    uint32_t brr = 0;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check parameters.
    if ((pins == NULL) || (configuration == NULL)) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Get clock source frequency.
    RCC_get_frequency_hz(USART_DESCRIPTOR[instance].clock_source, &usart_clock_hz);
    // Select HSI as peripheral clock for USART1 and USART2.
    RCC->CCIPR &= ~(0b1111 << 0); // Reset bits 2-3.
    RCC->CCIPR |= (0b1010 << 0); // USARTxSEL='10'.
    // Enable HSI in stop mode.
    RCC->CR |= (0b1 << 1); // HSI16KERON='1'.
    // Enable peripheral clock.
    (*USART_DESCRIPTOR[instance].rcc_enr) |= USART_DESCRIPTOR[instance].rcc_mask;
    (*USART_DESCRIPTOR[instance].rcc_smenr) |= USART_DESCRIPTOR[instance].rcc_mask;
    // Disable overrun detection (OVRDIS='1') and enable clock in stop mode (UCESM='1').
    USART_DESCRIPTOR[instance].peripheral->CR3 |= (0b1 << 12) | (0b1 << 23);
    // Baud rate.
    brr = ((usart_clock_hz) / (configuration->baud_rate));
    USART_DESCRIPTOR[instance].peripheral->BRR = (brr & 0x000FFFFF); // BRR = (fCK)/(baud rate).
    // Configure peripheral.
#if (STM32L0XX_DRIVERS_USART_MODE == 0)
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 5); // Enable RXNE interrupt (RXNEIE='1').
#endif
#if (STM32L0XX_DRIVERS_USART_MODE == 1)
    USART_DESCRIPTOR[instance].peripheral->CR2 |= ((configuration->match_character) << 24);
    USART_DESCRIPTOR[instance].peripheral->CR3 |= (0b1 << 6); // Transfer is performed after each RXNE event.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 14); // Enable CM interrupt (CMIE='1').
#endif
    // Set interrupt priority.
    NVIC_set_priority(USART_DESCRIPTOR[instance].nvic_interrupt, (configuration->nvic_priority));
    // Enable transmitter and receiver.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b11 << 2); // TE='1' and RE='1'.
    // Enable peripheral.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b11 << 0); // UE='1' and UESM='1'.
    // Configure GPIOs.
    GPIO_configure((pins->tx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->rx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Register callback.
#if (STM32L0XX_DRIVERS_USART_MODE == 0)
    usart_ctx.rxne_callback[instance] = (configuration->rxne_callback);
#endif
#if (STM32L0XX_DRIVERS_USART_MODE == 1)
    usart_ctx.cmf_callback[instance] = (configuration->cmf_callback);
#endif
    // Update initialization count.
    usart_ctx.init_count[instance]++;
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_de_init(USART_instance_t instance, const USART_gpio_t* pins) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Update initialization count.
    if (usart_ctx.init_count[instance] > 0) {
        usart_ctx.init_count[instance]--;
    }
    // Check initialization count.
    if (usart_ctx.init_count[instance] > 0) goto errors;
    // Check parameters.
    if (pins == NULL) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Disable USART alternate function.
    GPIO_configure((pins->tx), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->rx), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Disable peripheral.
    USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 0); // UE='0'.
    // Disable peripheral clock.
    (*USART_DESCRIPTOR[instance].rcc_enr) &= ~(USART_DESCRIPTOR[instance].rcc_mask);
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_enable_rx(USART_instance_t instance) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check state.
    if (usart_ctx.init_count[instance] == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Clear RXNE flag if needed.
    if (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 5)) != 0) {
        USART_DESCRIPTOR[instance].peripheral->RQR |= (0b1 << 3);
    }
    // Enable interrupt.
    NVIC_enable_interrupt(USART_DESCRIPTOR[instance].nvic_interrupt);
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_disable_rx(USART_instance_t instance) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check state.
    if (usart_ctx.init_count[instance] == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Disable interrupt.
    NVIC_disable_interrupt(USART_DESCRIPTOR[instance].nvic_interrupt);
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_write(USART_instance_t instance, uint8_t* data, uint32_t data_size_bytes) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    uint8_t idx = 0;
    uint32_t loop_count = 0;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check state.
    if (usart_ctx.init_count[instance] == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Check parameters.
    if ((data == NULL) || (data_size_bytes == 0)) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Byte loop.
    for (idx = 0; idx < data_size_bytes; idx++) {
#ifdef STM32L0XX_DRIVERS_USART_DISABLE_TX_0
        // Do not transmit null byte.
        if (data[idx] == 0) continue;
#endif
        // Fill transmit register.
        USART_DESCRIPTOR[instance].peripheral->TDR = data[idx];
        // Wait for transmission to complete.
        while (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 7)) == 0) {
            // Wait for TXE='1' or timeout.
            loop_count++;
            if (loop_count > USART_TIMEOUT_COUNT) {
                status = USART_ERROR_TX_TIMEOUT;
                goto errors;
            }
        }
    }
errors:
    return status;
}
