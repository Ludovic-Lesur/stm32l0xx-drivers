/*
 * lpuart.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "lpuart.h"

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "exti.h"
#include "gpio.h"
#include "lpuart_reg.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "types.h"

/*** LPUART local macros ***/

#define LPUART_TIMEOUT_COUNT				100000

#define LPUART_BAUD_RATE_CLOCK_THRESHOLD	4000

#define LPUART_BRR_VALUE_MIN				0x00300
#define LPUART_BRR_VALUE_MAX				0xFFFFF

/*** LPUART local structures ***/

/*******************************************************************/
typedef struct {
	uint8_t init_count;
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 0) || (STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	LPUART_rx_irq_cb_t rxne_callback;
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 1)
	LPUART_character_match_irq_cb_t cmf_callback;
#endif
} LPUART_context_t;

/*** LPUART local global variables ***/

static LPUART_context_t lpuart_ctx = {
	.init_count = 0,
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 0) || (STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	.rxne_callback = NULL
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 1)
	.cmf_callback = NULL
#endif
};

/*** LPUART local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) LPUART1_IRQHandler(void) {
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 0) || (STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	// Local variables.
	uint8_t rx_byte = 0;
	// RXNE interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 5)) != 0) {
		// Read incoming byte.
		rx_byte = (LPUART1 -> RDR);
		// Transmit byte to upper layer.
		if ((((LPUART1 -> CR1) & (0b1 << 5)) != 0) && (lpuart_ctx.rxne_callback != NULL)) {
			lpuart_ctx.rxne_callback(rx_byte);
		}
	}
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 1)
	// CMF interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 17)) != 0) {
		// Notify upper layer.
		if ((((LPUART1 -> CR1) & (0b1 << 14)) != 0) && (lpuart_ctx.cmf_callback != NULL)) {
			lpuart_ctx.cmf_callback();
		}
		// Clear CM flag.
		LPUART1 -> ICR |= (0b1 << 17);
	}
#endif
	// Overrun error interrupt.
	if (((LPUART1 -> ISR) & (0b1 << 3)) != 0) {
		// Clear ORE flag.
		LPUART1 -> ICR |= (0b1 << 3);
	}
	EXTI_clear_line_flag(EXTI_LINE_LPUART1);
}

/*******************************************************************/
static LPUART_status_t _LPUART1_set_baud_rate(uint32_t baud_rate) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	RCC_clock_t lpuart_clock;
	uint32_t lpuart_clock_hz = 0;
	uint64_t brr = 0;
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE != 0)
	uint8_t lse_status = 0;
#endif
	// Ensure peripheral is disabled.
	LPUART1 -> CR1 &= ~(0b1 << 0); // UE='0'.
	// Select peripheral clock.
	RCC -> CCIPR &= ~(0b11 << 10); // Reset bits 10-11.
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
	// Use HSI.
	RCC -> CCIPR |= (0b10 << 10); // LPUART1SEL='10'.
	lpuart_clock = RCC_CLOCK_HSI;
#else
	// Get LSE status.
	RCC_get_status(RCC_CLOCK_LSE, &lse_status);
	// Check LSE status and baud rate.
	if ((baud_rate < LPUART_BAUD_RATE_CLOCK_THRESHOLD) && (lse_status != 0)) {
		// Use LSE.
		RCC -> CCIPR |= (0b1 << 10); // LPUART1SEL='11'.
		lpuart_clock = RCC_CLOCK_LSE;
	}
	else {
		// Use HSI.
		RCC -> CCIPR &= ~(0b1 << 10); // LPUART1SEL='10'.
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
	LPUART1 -> BRR = (uint32_t) (brr & 0x000FFFFF); // BRR = (256*fCK)/(baud rate). See p.730 of RM0377 datasheet.
errors:
	return status;
}

#if (STM32L0XX_DRIVERS_LPUART_MODE == 3)
/*******************************************************************/
static LPUART_status_t _LPUART_set_rx_mode(LPUART_rx_mode_t rx_mode) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	// Set RX mode.
	switch (rx_mode) {
	case LPUART_RX_MODE_ADDRESSED:
		// Enable mute mode, address detection and wake up on address match.
		LPUART1 -> CR1 |= 0x00002800; // MME='1' and WAKE='1'.
		LPUART1 -> CR3 &= 0xFFCFFFFF; // WUS='00'.
		break;
	case LPUART_RX_MODE_DIRECT:
		// Disable mute mode, address detection and wake-up on RXNE.
		LPUART1 -> CR1 &= 0xFFFFD7FF; // MME='0' and WAKE='0'.
		LPUART1 -> CR3 |= 0x00030000; // WUS='11'.
		break;
	default:
		status = LPUART_ERROR_RX_MODE;
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
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 18);
	RCC -> APB1SMENR |= (0b1 << 18);
	// Disable overrun detection (OVRDIS='1') and enable clock in stop mode (UCESM='1').
	LPUART1 -> CR3 |= (0b1 << 12) | (0b1 << 23);
	// Baud rate.
	status = _LPUART1_set_baud_rate(configuration -> baud_rate);
	if (status != LPUART_SUCCESS) goto errors;
	// Configure peripheral.
#if (STM32L0XX_DRIVERS_LPUART_MODE == 0)
	LPUART1 -> CR1 |= (0b1 << 5); // Enable RXNE interrupt (RXNEIE='1').
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 1)
	// Configure character match interrupt and DMA.
	LPUART1 -> CR2 |= ((configuration -> match_character) << 24);
	LPUART1 -> CR3 |= (0b1 << 6); // Transfer is performed after each RXNE event (see p.738 of RM0377 datasheet).
	LPUART1 -> CR1 |= (0b1 << 14); // Enable CM interrupt (CMIE='1').
#endif
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	LPUART1 -> CR1 |= 0x00002822;
	LPUART1 -> CR2 |= ((configuration -> self_address) << 24) | (0b1 << 4);
	LPUART1 -> CR3 |= 0x00805000;
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 3)
	status = _LPUART_set_rx_mode(configuration -> rx_mode);
	if (status != LPUART_SUCCESS) goto errors;
#endif
	// Configure interrupt.
	EXTI_configure_line(EXTI_LINE_LPUART1, EXTI_TRIGGER_RISING_EDGE);
	NVIC_set_priority(NVIC_INTERRUPT_LPUART1, (configuration -> nvic_priority));
	// Enable transmitter and receiver.
	LPUART1 -> CR1 |= (0b11 << 2); // TE='1' and RE='1'.
	// Enable peripheral.
	LPUART1 -> CR1 |= (0b11 << 0); // UE='1' and UESM='1'
	// Configure GPIOs.
	GPIO_configure((pins -> tx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure((pins -> rx), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	// Put NRE pin in high impedance since it is directly connected to the DE pin.
	GPIO_configure((pins -> de), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure((pins -> nre), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Register callback.
#if (STM32L0XX_DRIVERS_LPUART_MODE == 0)
	lpuart_ctx.rxne_callback = (configuration -> rxne_callback);
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 1)
	lpuart_ctx.cmf_callback = (configuration -> cmf_callback);
#endif
	// Update initialization count.
	lpuart_ctx.init_count++;
errors:
	return status;
}

/*******************************************************************/
LPUART_status_t LPUART_de_init(const LPUART_gpio_t* pins) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	// Update initialization count.
	if (lpuart_ctx.init_count > 0) {
		lpuart_ctx.init_count--;
	}
	// Check initialization count.
	if (lpuart_ctx.init_count > 0) goto errors;
	// Check parameter.
	if (pins == NULL) {
		status = LPUART_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Disable LPUART alternate function.
	GPIO_configure((pins -> tx), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure((pins -> rx), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	// Put NRE pin in high impedance since it is directly connected to the DE pin.
	GPIO_configure((pins -> de), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure((pins -> nre), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
#endif
	// Disable peripheral.
	LPUART1 -> CR1 &= ~(0b1 << 0); // UE='0'.
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 18); // LPUARTEN='0'.
errors:
	return status;
}

/*******************************************************************/
LPUART_status_t LPUART_enable_rx(void) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	// Check state.
	if (lpuart_ctx.init_count == 0) {
		status = LPUART_ERROR_UNINITIALIZED;
		goto errors;
	}
	// Clear RXNE flag if needed.
	if (((LPUART1 -> ISR) & (0b1 << 5)) != 0) {
		LPUART1 -> RQR |= (0b1 << 3);
	}
	// Enable interrupt.
	NVIC_enable_interrupt(NVIC_INTERRUPT_LPUART1);
	// Enable receiver.
	LPUART1 -> CR1 |= (0b1 << 2); // RE='1'.
#if (STM32L0XX_DRIVERS_LPUART_MODE == 2)
	// Mute mode request.
	LPUART1 -> RQR |= (0b1 << 2); // MMRQ='1'.
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 3)
	// Check mode.
	if (((LPUART1 -> CR1) & (0b1 << 13)) != 0) {
		// Mute mode request.
		LPUART1 -> RQR |= (0b1 << 2); // MMRQ='1'.
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
	if (lpuart_ctx.init_count == 0) {
		status = LPUART_ERROR_UNINITIALIZED;
		goto errors;
	}
	// Disable receiver.
	LPUART1 -> CR1 &= ~(0b1 << 2); // RE='0'.
	// Disable interrupt.
	NVIC_disable_interrupt(NVIC_INTERRUPT_LPUART1);
errors:
	return status;
}

/*******************************************************************/
LPUART_status_t LPUART_write(uint8_t* data, uint32_t data_size_bytes) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	uint32_t idx = 0;
	uint32_t loop_count = 0;
	// Check state.
	if (lpuart_ctx.init_count == 0) {
		status = LPUART_ERROR_UNINITIALIZED;
		goto errors;
	}
	// Check parameter.
	if (data == NULL) {
		status = LPUART_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Fill TX buffer with new bytes.
	for (idx=0 ; idx<data_size_bytes ; idx++) {
#ifdef STM32L0XX_DRIVERS_LPUART_DISABLE_TX_0
		// Do not transmit null byte.
		if (data[idx] == 0) continue;
#endif
		// Fill transmit register.
		LPUART1 -> TDR = data[idx];
		// Wait for transmission to complete.
		while (((LPUART1 -> ISR) & (0b1 << 7)) == 0) {
			// Wait for TXE='1' or timeout.
			loop_count++;
			if (loop_count > LPUART_TIMEOUT_COUNT) {
				status = LPUART_ERROR_TX_TIMEOUT;
				goto errors;
			}
		}
	}
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	// Wait for TC flag.
	while (((LPUART1 -> ISR) & (0b1 << 6)) == 0) {
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

#if (STM32L0XX_DRIVERS_LPUART_MODE == 3)
/*******************************************************************/
LPUART_status_t LPUART_set_configuration(LPUART_configuration_t* configuration) {
	// Local variables.
	LPUART_status_t status = LPUART_SUCCESS;
	// Check state.
	if (lpuart_ctx.init_count == 0) {
		status = LPUART_ERROR_UNINITIALIZED;
		goto errors;
	}
	// Check parameters.
	if (configuration == NULL) {
		status = LPUART_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Temporary disable peripheral while configuring.
	LPUART1 -> CR1 &= ~(0b1 << 0); // UE='0'.
	// Set baud rate.
	status = _LPUART1_set_baud_rate(configuration -> baud_rate);
	if (status != LPUART_SUCCESS) goto errors;
	// Set RX mode.
	status = _LPUART_set_rx_mode(configuration -> rx_mode);
	if (status != LPUART_SUCCESS) goto errors;
	// Register callback.
	lpuart_ctx.rxne_callback = (configuration -> rxne_callback);
errors:
	// Enable peripheral.
	LPUART1 -> CR1 |= (0b1 << 0); // UE='1'.
	return status;
}
#endif
