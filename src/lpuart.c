/*
 * lpuart.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#include "lpuart.h"

#include "exti.h"
#include "gpio.h"
#include "lpuart_registers.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_registers.h"
#include "types.h"

/*** LPUART local macros ***/

#define LPUART_TIMEOUT_COUNT                1000000

#define LPUART_BAUD_RATE_CLOCK_THRESHOLD    4000

#define LPUART_REGISTER_MASK_BRR            0x000FFFFF
#define LPUART_REGISTER_MASK_TDR            0x000000FF

#define LPUART_BRR_VALUE_MIN                0x00300
#define LPUART_BRR_VALUE_MAX                LPUART_REGISTER_MASK_BRR

/*** LPUART local structures ***/

/*******************************************************************/
typedef struct {
    uint8_t init_flag;
    LPUART_rx_irq_cb_t rxne_callback;
} LPUART_context_t;

/*** LPUART local global variables ***/

static LPUART_context_t lpuart_ctx = {
    .init_flag = 0,
    .rxne_callback = NULL
};

/*** LPUART local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) LPUART1_IRQHandler(void) {
    // Local variables.
    uint8_t rx_byte = 0;
    // RXNE interrupt.
    if (((LPUART1->ISR) & (0b1 << 5)) != 0) {
        // Read incoming byte.
        rx_byte = (LPUART1->RDR);
        // Transmit byte to upper layer.
        if ((((LPUART1->CR1) & (0b1 << 5)) != 0) && (lpuart_ctx.rxne_callback != NULL)) {
            lpuart_ctx.rxne_callback(rx_byte);
        }
    }
    // Overrun error interrupt.
    if (((LPUART1->ISR) & (0b1 << 3)) != 0) {
        // Clear ORE flag.
        LPUART1->ICR = (0b1 << 3);
    }
    EXTI_clear_line_flag(EXTI_LINE_LPUART1);
}

/*******************************************************************/
static LPUART_status_t _LPUART_set_baud_rate(uint32_t baud_rate) {
    // Local variables.
    LPUART_status_t status = LPUART_SUCCESS;
    RCC_clock_t lpuart_clock;
    uint32_t lpuart_clock_hz = 0;
    uint64_t brr = 0;
    uint32_t reg_value = 0;
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE != 0)
    uint8_t lse_status = 0;
#endif
    // Ensure peripheral is disabled.
    LPUART1->CR1 &= ~(0b1 << 0); // UE='0'.
    // Select peripheral clock.
    RCC->CCIPR &= ~(0b11 << 10); // Reset bits 10-11.
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
    // Use HSI.
    RCC->CCIPR |= (0b10 << 10); // LPUART1SEL='10'.
    lpuart_clock = RCC_CLOCK_HSI;
#else
    // Get LSE status.
    RCC_get_status(RCC_CLOCK_LSE, &lse_status);
    // Check LSE status and baud rate.
    if ((baud_rate < LPUART_BAUD_RATE_CLOCK_THRESHOLD) && (lse_status != 0)) {
        // Use LSE.
        RCC->CCIPR |= (0b11 << 10); // LPUART1SEL='11'.
        lpuart_clock = RCC_CLOCK_LSE;
    }
    else {
        // Use HSI.
        RCC->CCIPR |= (0b10 << 10); // LPUART1SEL='10'.
        lpuart_clock = RCC_CLOCK_HSI;
    }
#endif
    // Get clock source frequency.
    RCC_get_frequency_hz(lpuart_clock, &lpuart_clock_hz);
    // Compute register value.
    brr = ((uint64_t) lpuart_clock_hz) << 8;
    brr /= (uint64_t) baud_rate;
    // Check value.
    if ((brr < LPUART_BRR_VALUE_MIN) || (brr > LPUART_BRR_VALUE_MAX)) {
        status = LPUART_ERROR_BAUD_RATE;
        goto errors;
    }
    // Set baud rate.
    reg_value = ((LPUART1->BRR) & (~LPUART_REGISTER_MASK_BRR));
    reg_value |= (uint32_t) (brr & LPUART_REGISTER_MASK_BRR);
    LPUART1->BRR = reg_value;
errors:
    return status;
}

#ifdef STM32L0XX_DRIVERS_LPUART_RS485
/*******************************************************************/
static LPUART_status_t _LPUART_set_rs485_mode(LPUART_rs485_mode_t rs485_mode) {
    // Local variables.
    LPUART_status_t status = LPUART_SUCCESS;
    // Set RX mode.
    switch (rs485_mode) {
    case LPUART_RS485_MODE_ADDRESSED:
        // Enable mute mode, address detection and wake up on address match.
        LPUART1->CR1 |= 0x00002800; // MME='1' and WAKE='1'.
        LPUART1->CR3 &= 0xFFCFFFFF; // WUS='00'.
        break;
    case LPUART_RS485_MODE_DIRECT:
        // Disable mute mode, address detection and wake-up on RXNE.
        LPUART1->CR1 &= 0xFFFFD7FF; // MME='0' and WAKE='0'.
        LPUART1->CR3 |= 0x00030000; // WUS='11'.
        break;
    default:
        status = LPUART_ERROR_RS485_MODE;
        goto errors;
    }
errors:
    return status;
}
#endif

/*** LPUART functions ***/

/*******************************************************************/
LPUART_status_t LPUART_init(const LPUART_gpio_t* pins, LPUART_configuration_t* configuration) {
    // Local variables.
    LPUART_status_t status = LPUART_SUCCESS;
    // Check parameters.
    if ((pins == NULL) || (configuration == NULL)) {
        status = LPUART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check state.
    if (lpuart_ctx.init_flag != 0) {
        status = LPUART_ERROR_ALREADY_INITIALIZED;
        goto errors;
    }
    // Enable peripheral clock.
    RCC->APB1ENR |= (0b1 << 18);
    RCC->APB1SMENR |= (0b1 << 18);
    // Disable overrun detection (OVRDIS='1') and enable clock in stop mode (UCESM='1').
    LPUART1->CR3 |= (0b1 << 12) | (0b1 << 23);
    // Baud rate.
    status = _LPUART_set_baud_rate(configuration->baud_rate);
    if (status != LPUART_SUCCESS) goto errors;
    // Configure peripheral.
    if ((configuration->rxne_irq_callback) != NULL) {
        LPUART1->CR1 |= (0b1 << 5); // RXNEIE='1'.
    }
    else {
        LPUART1->CR1 &= ~(0b1 << 5); // RXNEIE='0'.
    }
#ifdef STM32L0XX_DRIVERS_LPUART_RS485
    LPUART1->CR2 |= ((configuration->self_address) << 24) | (0b1 << 4);
    LPUART1->CR3 |= 0x00004000;
    status = _LPUART_set_rs485_mode(configuration->rs485_mode);
    if (status != LPUART_SUCCESS) goto errors;
#endif
    // Configure interrupt.
    EXTI_enable_line(EXTI_LINE_LPUART1, EXTI_TRIGGER_RISING_EDGE);
    NVIC_set_priority(NVIC_INTERRUPT_LPUART1, (configuration->nvic_priority));
    // Configure GPIOs.
    GPIO_configure((pins->tx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->rx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef STM32L0XX_DRIVERS_LPUART_RS485
    // Put NRE pin in high impedance since it is directly connected to the DE pin.
    GPIO_configure((pins->de), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->nre), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
    // Enable transmitter and receiver.
    LPUART1->CR1 |= (0b11 << 2); // TE='1' and RE='1'.
    // Enable peripheral.
    LPUART1->CR1 |= (0b11 << 0); // UE='1' and UESM='1'
    // Register callback.
    lpuart_ctx.rxne_callback = (configuration->rxne_irq_callback);
    // Update initialization flag.
    lpuart_ctx.init_flag = 1;
errors:
    return status;
}

/*******************************************************************/
LPUART_status_t LPUART_de_init(const LPUART_gpio_t* pins) {
    // Local variables.
    LPUART_status_t status = LPUART_SUCCESS;
    // Check parameter.
    if (pins == NULL) {
        status = LPUART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check state.
    if (lpuart_ctx.init_flag == 0) {
        status = LPUART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Disable line.
    EXTI_disable_line(EXTI_LINE_LPUART1);
    // Disable LPUART alternate function.
    GPIO_configure((pins->tx), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->rx), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#ifdef STM32L0XX_DRIVERS_LPUART_RS485
    // Put NRE pin in high impedance since it is directly connected to the DE pin.
    GPIO_configure((pins->de), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure((pins->nre), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
    // Disable peripheral.
    LPUART1->CR3 &= ~(0b1 << 23); // UCESM='0'.
    LPUART1->CR1 &= ~(0b11 << 0); // UE='0' and and UESM='0'.
    // Disable peripheral clock.
    RCC->APB1ENR &= ~(0b1 << 18); // LPUARTEN='0'.
    // Update initialization flag.
    lpuart_ctx.init_flag = 0;
errors:
    return status;
}

/*******************************************************************/
LPUART_status_t LPUART_enable_rx(void) {
    // Local variables.
    LPUART_status_t status = LPUART_SUCCESS;
    // Check state.
    if (lpuart_ctx.init_flag == 0) {
        status = LPUART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Clear RXNE flag if needed.
    if (((LPUART1->ISR) & (0b1 << 5)) != 0) {
        LPUART1->RQR |= (0b1 << 3);
    }
    // Enable interrupt.
    NVIC_enable_interrupt(NVIC_INTERRUPT_LPUART1);
    // Enable receiver.
    LPUART1->CR1 |= (0b1 << 2); // RE='1'.
#ifdef STM32L0XX_DRIVERS_LPUART_RS485
    // Check mode.
    if (((LPUART1->CR1) & (0b1 << 13)) != 0) {
        // Mute mode request.
        LPUART1->RQR |= (0b1 << 2); // MMRQ='1'.
    }
#endif
errors:
    return status;
}

/*******************************************************************/
LPUART_status_t LPUART_disable_rx(void) {
    // Local variables.
    LPUART_status_t status = LPUART_SUCCESS;
    // Check state.
    if (lpuart_ctx.init_flag == 0) {
        status = LPUART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Disable receiver.
    LPUART1->CR1 &= ~(0b1 << 2); // RE='0'.
    // Disable interrupt.
    NVIC_disable_interrupt(NVIC_INTERRUPT_LPUART1);
errors:
    return status;
}

/*******************************************************************/
LPUART_status_t LPUART_write(uint8_t* data, uint32_t data_size_bytes) {
    // Local variables.
    LPUART_status_t status = LPUART_SUCCESS;
    uint32_t reg_value = 0;
    uint32_t idx = 0;
    uint32_t loop_count = 0;
    // Check parameter.
    if (data == NULL) {
        status = LPUART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check state.
    if (lpuart_ctx.init_flag == 0) {
        status = LPUART_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Fill TX buffer with new bytes.
    for (idx = 0; idx < data_size_bytes; idx++) {
#ifdef STM32L0XX_DRIVERS_LPUART_DISABLE_TX_0
        // Do not transmit null byte.
        if (data[idx] == 0) continue;
#endif
        // Fill transmit register.
        reg_value = ((LPUART1->TDR) & (~LPUART_REGISTER_MASK_TDR));
        reg_value |= (uint32_t) (data[idx] & LPUART_REGISTER_MASK_TDR);
        LPUART1->TDR = reg_value;
        // Wait for transmission to complete.
        while (((LPUART1->ISR) & (0b1 << 7)) == 0) {
            // Wait for TXE='1' or timeout.
            loop_count++;
            if (loop_count > LPUART_TIMEOUT_COUNT) {
                status = LPUART_ERROR_TX_TIMEOUT;
                goto errors;
            }
        }
    }
#ifdef STM32L0XX_DRIVERS_LPUART_RS485
    // Wait for TC flag.
    while (((LPUART1->ISR) & (0b1 << 6)) == 0) {
        // Exit if timeout.
        loop_count++;
        if (loop_count > LPUART_TIMEOUT_COUNT) {
            status = LPUART_ERROR_TC_TIMEOUT;
            goto errors;
        }
    }
#endif
errors:
    return status;
}

/*******************************************************************/
LPUART_status_t LPUART_get_rdr_register_address(uint32_t* rdr_register_address) {
    // Local variables.
    LPUART_status_t status = LPUART_SUCCESS;
    // Check parameter.
    if (rdr_register_address == NULL) {
        status = LPUART_ERROR_NULL_PARAMETER;
        goto errors;
    }
    (*rdr_register_address) = ((uint32_t) &(LPUART1->RDR));
errors:
    return status;
}

#endif /* STM32L0XX_DRIVERS_DISABLE */
