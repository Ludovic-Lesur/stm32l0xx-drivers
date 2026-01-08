/*
 * usart.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#include "usart.h"

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#include "exti.h"
#include "gpio.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_registers.h"
#include "types.h"
#include "usart_registers.h"

/*** USART local macros ***/

#define USART_TIMEOUT_COUNT         100000

#define USART_REGISTER_MASK_BRR     0x0000FFFF
#define USART_REGISTER_MASK_TDR     0x000000FF

#define USART_BRR_VALUE_MIN         0x0010
#define USART_BRR_VALUE_MAX         USART_REGISTER_MASK_BRR

/*** USART local structures ***/

/*******************************************************************/
typedef struct {
    USART_registers_t* peripheral;
    volatile uint32_t* rcc_enr;
    volatile uint32_t* rcc_smenr;
    uint32_t rcc_mask;
    uint32_t rcc_ccipr_shift;
    NVIC_interrupt_t nvic_interrupt;
} USART_descriptor_t;

/*******************************************************************/
typedef struct {
    uint8_t init_flag;
    USART_rx_irq_cb_t rxne_irq_callback;
} USART_context_t;

/*** USART local global variables ***/

static const USART_descriptor_t USART_DESCRIPTOR[USART_INSTANCE_LAST] = {
    { USART2, &(RCC->APB1ENR), &(RCC->APB1SMENR), (0b1 << 17), 2, NVIC_INTERRUPT_USART2 },
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    { USART1, &(RCC->APB2ENR), &(RCC->APB2SMENR), (0b1 << 14), 0, NVIC_INTERRUPT_USART1 },
#endif
};

static USART_context_t usart_ctx[USART_INSTANCE_LAST] = {
    [0 ... (USART_INSTANCE_LAST - 1)] = {
        .init_flag = 0,
        .rxne_irq_callback = NULL
    }
};

/*** USART local functions ***/

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _USART_irq_handler(USART_instance_t instance) {
    // Local variables.
    uint8_t rx_byte = 0;
    // RXNE interrupt.
    if (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 5)) != 0) {
        // Read incoming byte.
        rx_byte = (uint8_t) (USART_DESCRIPTOR[instance].peripheral->RDR);
        // Transmit byte to upper layer.
        if ((((USART_DESCRIPTOR[instance].peripheral->CR1) & (0b1 << 5)) != 0) && (usart_ctx[instance].rxne_irq_callback != NULL)) {
            usart_ctx[instance].rxne_irq_callback(rx_byte);
        }
    }
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) USART2_IRQHandler(void) {
    // Execute internal callback.
    _USART_irq_handler(USART_INSTANCE_USART2);
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_USART2);
}

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) USART1_IRQHandler(void) {
    // Execute internal callback.
    _USART_irq_handler(USART_INSTANCE_USART1);
    // Clear EXTI line.
    EXTI_clear_line_flag(EXTI_LINE_USART1);
}
#endif

/*** USART functions ***/

/*******************************************************************/
USART_status_t USART_init(USART_instance_t instance, const USART_gpio_t* pins, USART_configuration_t* configuration) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    uint32_t usart_clock_hz = 0;
    uint32_t brr = 0;
    uint32_t reg_value = 0;
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
    // Check state.
    if (usart_ctx[instance].init_flag != 0) {
        status = USART_ERROR_ALREADY_INITIALIZED;
        goto errors;
    }
    // Select peripheral clock.
    RCC->CCIPR &= ~(0b11 << USART_DESCRIPTOR[instance].rcc_ccipr_shift);
    switch (configuration->clock) {
    case RCC_CLOCK_SYSTEM:
        // Nothing to do.
        break;
    case RCC_CLOCK_HSI:
        RCC->CCIPR |= (0b10 << USART_DESCRIPTOR[instance].rcc_ccipr_shift);
        break;
    case RCC_CLOCK_LSE:
        RCC->CCIPR |= (0b11 << USART_DESCRIPTOR[instance].rcc_ccipr_shift);
        break;
    default:
        status = USART_ERROR_CLOCK;
        goto errors;
    }
    // Get clock source frequency.
    RCC_get_frequency_hz((configuration->clock), &usart_clock_hz);
    // Enable peripheral clock.
    (*USART_DESCRIPTOR[instance].rcc_enr) |= USART_DESCRIPTOR[instance].rcc_mask;
    (*USART_DESCRIPTOR[instance].rcc_smenr) |= USART_DESCRIPTOR[instance].rcc_mask;
    // Disable overrun detection (OVRDIS='1') and enable clock in stop mode (UCESM='1').
    USART_DESCRIPTOR[instance].peripheral->CR3 |= (0b1 << 12) | (0b1 << 23);
    // Baud rate.
    brr = ((usart_clock_hz) / (configuration->baud_rate));
    // Check value.
    if ((brr < USART_BRR_VALUE_MIN) || (brr > USART_BRR_VALUE_MAX)) {
        status = USART_ERROR_BAUD_RATE;
        goto errors;
    }
    reg_value = ((USART_DESCRIPTOR[instance].peripheral->BRR) & (~USART_REGISTER_MASK_BRR));
    reg_value |= (brr & USART_REGISTER_MASK_BRR);
    USART_DESCRIPTOR[instance].peripheral->BRR = reg_value;
    // Configure peripheral.
    if (configuration->rxne_irq_callback != NULL) {
        USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 5); // RXNEIE='1'.
    }
    else {
        USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 5); // RXNEIE='0'.
    }
    // Set interrupt priority.
    NVIC_set_priority(USART_DESCRIPTOR[instance].nvic_interrupt, (configuration->nvic_priority));
    // Configure GPIOs.
    GPIO_configure((pins->tx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->rx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Enable transmitter and receiver.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b11 << 2); // TE='1' and RE='1'.
    // Enable peripheral.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b11 << 0); // UE='1' and UESM='1'.
    // Register callback.
    usart_ctx[instance].rxne_irq_callback = (configuration->rxne_irq_callback);
    // Update initialization flag.
    usart_ctx[instance].init_flag = 1;
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
    // Check parameters.
    if (pins == NULL) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check state.
    if (usart_ctx[instance].init_flag == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Disable USART alternate function.
    GPIO_configure((pins->tx), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->rx), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Disable peripheral.
    USART_DESCRIPTOR[instance].peripheral->CR3 &= ~(0b1 << 23); // UCESM='0'.
    USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b11 << 0); // UE='0' and UESM='0'.
    // Disable peripheral clock.
    (*USART_DESCRIPTOR[instance].rcc_enr) &= ~(USART_DESCRIPTOR[instance].rcc_mask);
    // Update initialization flag.
    usart_ctx[instance].init_flag = 0;
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
    if (usart_ctx[instance].init_flag == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Clear RXNE flag if needed.
    if (((USART_DESCRIPTOR[instance].peripheral->ISR) & (0b1 << 5)) != 0) {
        USART_DESCRIPTOR[instance].peripheral->RQR |= (0b1 << 3);
    }
    // Enable interrupt.
    NVIC_enable_interrupt(USART_DESCRIPTOR[instance].nvic_interrupt);
    // Enable receiver.
    USART_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 2); // RE='1'.
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
    if (usart_ctx[instance].init_flag == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Disable receiver.
    USART_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 2); // RE='0'.
    // Disable interrupt.
    NVIC_disable_interrupt(USART_DESCRIPTOR[instance].nvic_interrupt);
errors:
    return status;
}

/*******************************************************************/
USART_status_t USART_write(USART_instance_t instance, uint8_t* data, uint32_t data_size_bytes) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    uint32_t reg_value = 0;
    uint8_t idx = 0;
    uint32_t loop_count = 0;
    // Check instance.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    // Check parameters.
    if ((data == NULL) || (data_size_bytes == 0)) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check state.
    if (usart_ctx[instance].init_flag == 0) {
        status = USART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Byte loop.
    for (idx = 0; idx < data_size_bytes; idx++) {
#ifdef STM32L0XX_DRIVERS_USART_DISABLE_TX_0
        // Do not transmit null byte.
        if (data[idx] == 0) continue;
#endif
        // Fill transmit register.
        reg_value = ((USART_DESCRIPTOR[instance].peripheral->TDR) & (~USART_REGISTER_MASK_TDR));
        reg_value |= (uint32_t) (data[idx] & USART_REGISTER_MASK_TDR);
        USART_DESCRIPTOR[instance].peripheral->TDR = reg_value;
        // Wait for transmission to complete.
        loop_count = 0;
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

/*******************************************************************/
USART_status_t USART_get_rdr_register_address(USART_instance_t instance, uint32_t* rdr_register_address) {
    // Local variables.
    USART_status_t status = USART_SUCCESS;
    // Check parameters.
    if (instance >= USART_INSTANCE_LAST) {
        status = USART_ERROR_INSTANCE;
        goto errors;
    }
    if (rdr_register_address == NULL) {
        status = USART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Update address.
    (*rdr_register_address) = ((uint32_t) &(USART_DESCRIPTOR[instance].peripheral->RDR));
errors:
    return status;
}

#endif /* STM32L0XX_DRIVERS_DISABLE */
