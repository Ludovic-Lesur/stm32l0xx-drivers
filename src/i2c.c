/*
 * i2c.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "i2c.h"

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#include "gpio.h"
#include "i2c_reg.h"
#include "lptim.h"
#include "rcc_reg.h"
#include "types.h"

/*** I2C local macros ***/

#define I2C_ACCESS_TIMEOUT_COUNT	1000000

/*** I2C local structures ***/

/*******************************************************************/
typedef struct {
	I2C_registers_t* peripheral;
	volatile uint32_t* rcc_enr;
	uint32_t rcc_mask;
} I2C_descriptor_t;

/*******************************************************************/
typedef struct {
	uint8_t init_count[I2C_INSTANCE_LAST];
} I2C_context_t;

/*** I2C local global variables ***/

static const I2C_descriptor_t I2C_DESCRIPTOR[I2C_INSTANCE_LAST] = {
	{I2C1, &(RCC -> APB1ENR), (0b1 << 21)},
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	{I2C2, &(RCC -> APB1ENR), (0b1 << 22)},
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	{I2C3, &(RCC -> APB1ENR), (0b1 << 30)},
#endif
};
static I2C_context_t i2c_ctx = {.init_count = {0}};

/*** I2C local functions ***/

/*******************************************************************/
static I2C_status_t _I2C_clear(I2C_instance_t instance) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	// Disable peripheral.
	I2C_DESCRIPTOR[instance].peripheral -> CR1 &= ~(0b1 << 0); // PE='0'.
	// Clear delay.
	lptim_status = LPTIM_delay_milliseconds(2, LPTIM_DELAY_MODE_ACTIVE);
	LPTIM_exit_error(I2C_ERROR_BASE_LPTIM);
	// Enable peripheral and clear all flags.
	I2C_DESCRIPTOR[instance].peripheral -> CR1 |= (0b1 << 0); // PE='1'.
	I2C_DESCRIPTOR[instance].peripheral -> ICR |= 0x00003F38;
errors:
	return status;
}

/*** I2C functions ***/

/*******************************************************************/
I2C_status_t I2C_init(I2C_instance_t instance, const I2C_gpio_t* pins) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	// Check instance.
	if (instance >= I2C_INSTANCE_LAST) {
		status = I2C_ERROR_INSTANCE;
		goto errors;
	}
	// Check parameters.
	if (pins == NULL) {
		status = I2C_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Enable peripheral clock.
	(*I2C_DESCRIPTOR[instance].rcc_enr) |= I2C_DESCRIPTOR[instance].rcc_mask;
	// Disable peripheral during configuration.
	I2C_DESCRIPTOR[instance].peripheral -> CR1 &= ~(0b1 << 0); // PE='0'.
	// I2CCLK = PCLK1/(PRESC+1) = SYSCLK/(PRESC+1) = 2MHz (HSI) (PRESC='1000').
	// SCL frequency to 10kHz. See p.641 of RM0377 datasheet.
	I2C_DESCRIPTOR[instance].peripheral -> TIMINGR |= (7 << 28) | (99 << 8) | (99 << 0);
	// Enable peripheral.
	I2C_DESCRIPTOR[instance].peripheral -> CR1 |= (0b1 << 0); // PE='1'.
	// Configure GPIOs.
	GPIO_configure((pins -> scl), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure((pins -> sda), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Update initialization count.
	i2c_ctx.init_count[instance]++;
errors:
	return status;
}

/*******************************************************************/
I2C_status_t I2C_de_init(I2C_instance_t instance, const I2C_gpio_t* pins) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	// Check instance.
	if (instance >= I2C_INSTANCE_LAST) {
		status = I2C_ERROR_INSTANCE;
		goto errors;
	}
	// Check parameters.
	if (pins == NULL) {
		status = I2C_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Update initialization count.
	if (i2c_ctx.init_count[instance] > 0) {
		i2c_ctx.init_count[instance]--;
	}
	// Check initialization count.
	if (i2c_ctx.init_count[instance] > 0) goto errors;
	// Disable I2C alternate function.
	GPIO_configure((pins -> scl), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_configure((pins -> sda), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable peripheral.
	I2C_DESCRIPTOR[instance].peripheral -> CR1 &= ~(0b1 << 0); // PE='0'.
	// Disable peripheral clock.
	(*I2C_DESCRIPTOR[instance].rcc_enr) &= ~(I2C_DESCRIPTOR[instance].rcc_mask);
errors:
	return status;
}

/*******************************************************************/
I2C_status_t I2C_write(I2C_instance_t instance, uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	uint32_t loop_count = 0;
	uint8_t idx = 0;
	// Check instance.
	if (instance >= I2C_INSTANCE_LAST) {
		status = I2C_ERROR_INSTANCE;
		goto errors;
	}
	// Check parameters.
	if ((data == NULL) || (data_size_bytes == 0)) {
		status = I2C_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Clear peripheral.
	status = _I2C_clear(instance);
	if (status != I2C_SUCCESS) goto errors;
	// Wait for I2C bus to be ready.
	while (((I2C_DESCRIPTOR[instance].peripheral -> ISR) & (0b1 << 15)) != 0) {
		// Wait for BUSY='0' or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_BUSY;
			goto errors;
		}
	}
	// Configure number of bytes to send and 7-bits slave address with write request.
	I2C_DESCRIPTOR[instance].peripheral -> CR2 &= 0xFF00F800;
	I2C_DESCRIPTOR[instance].peripheral -> CR2 |= (data_size_bytes << 16) | ((slave_address & 0x7F) << 1);
	// Generate start condition.
	I2C_DESCRIPTOR[instance].peripheral -> CR2 |= (0b1 << 13); // START='1'.
	loop_count = 0;
	while (((I2C_DESCRIPTOR[instance].peripheral -> CR2) & (0b1 << 13)) != 0) {
		// Wait for START bit to be cleared by hardware or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_START_BIT_CLEAR;
			goto errors;
		}
	}
	// Send bytes.
	loop_count = 0;
	while ((idx < data_size_bytes) && (((I2C_DESCRIPTOR[instance].peripheral -> ISR) & (0b1 << 4)) == 0)) {
		// Wait for transmit buffer to be empty (TXIS='1').
		if (((I2C_DESCRIPTOR[instance].peripheral -> ISR) & (0b1 << 1)) != 0) {
			// Send next byte.
			I2C_DESCRIPTOR[instance].peripheral -> TXDR = data[idx];
			idx++;
		}
		// Exit if timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_TX_BUFFER_EMPTY;
			goto errors;
		}
	}
	// Wait for last byte to be sent.
	loop_count = 0;
	while (((I2C_DESCRIPTOR[instance].peripheral -> ISR) & (0b1 << 6)) == 0) {
		// Wait for TC='1' or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_TRANSFER_COMPLETE;
			goto errors;
		}
	}
	if (stop_flag != 0) {
		// Generate stop condition.
		I2C_DESCRIPTOR[instance].peripheral -> CR2 |= (0b1 << 14);
		loop_count = 0;
		while (((I2C_DESCRIPTOR[instance].peripheral -> ISR) & (0b1 << 5)) == 0) {
			// Wait for STOPF='1' or timeout.
			loop_count++;
			if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
				status = I2C_ERROR_STOP_DETECTION_FLAG;
				goto errors;
			}
		}
		// Clear flag.
		I2C_DESCRIPTOR[instance].peripheral -> ICR |= (0b1 << 5); // STOPCF='1'.
	}
errors:
	return status;
}

/*******************************************************************/
I2C_status_t I2C_read(I2C_instance_t instance, uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes) {
	// Local variables.
	I2C_status_t status = I2C_SUCCESS;
	uint32_t loop_count = 0;
	uint8_t idx = 0;
	// Check instance.
	if (instance >= I2C_INSTANCE_LAST) {
		status = I2C_ERROR_INSTANCE;
		goto errors;
	}
	// Check parameters.
	if ((data == NULL) || (data_size_bytes == 0)) {
		status = I2C_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Clear peripheral.
	status = _I2C_clear(instance);
	if (status != I2C_SUCCESS) goto errors;
	// Wait for I2C bus to be ready.
	while (((I2C_DESCRIPTOR[instance].peripheral -> ISR) & (0b1 << 15)) != 0) {
		// Wait for BUSY='0' or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_BUSY;
			goto errors;
		}
	}
	// Configure number of bytes to receive and 7-bits slave address with read request.
	I2C_DESCRIPTOR[instance].peripheral -> CR2 &= 0xFF00FC00;
	I2C_DESCRIPTOR[instance].peripheral -> CR2 |= (data_size_bytes << 16) | (0b1 << 12) | (0b1 << 10) | ((slave_address & 0x7F) << 1);
	// Generate start condition.
	I2C_DESCRIPTOR[instance].peripheral -> CR2 |= (0b1 << 13); // START='1'.
	loop_count = 0;
	while (((I2C_DESCRIPTOR[instance].peripheral -> CR2) & (0b1 << 13)) != 0) {
		// Wait for START bit to be cleared by hardware or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_START_BIT_CLEAR;
			goto errors;
		}
	}
	// Get bytes.
	loop_count = 0;
	while (idx < data_size_bytes) {
		// Wait for incoming data (RXNE='1').
		if (((I2C_DESCRIPTOR[instance].peripheral -> ISR) & (0b1 << 2)) != 0) {
			// Fill RX buffer with new byte */
			data[idx] = (I2C_DESCRIPTOR[instance].peripheral -> RXDR);
			idx++;
		}
		// Exit if timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_RX_TIMEOUT;
			goto errors;
		}
	}
	// Send a NACK and STOP condition after last byte.
	I2C_DESCRIPTOR[instance].peripheral -> CR2 |= (0b1 << 15);
	I2C_DESCRIPTOR[instance].peripheral -> CR2 |= (0b1 << 14);
	loop_count = 0;
	while (((I2C_DESCRIPTOR[instance].peripheral -> ISR) & (0b1 << 5)) == 0) {
		// Wait for STOPF='1' or timeout.
		loop_count++;
		if (loop_count > I2C_ACCESS_TIMEOUT_COUNT) {
			status = I2C_ERROR_STOP_DETECTION_FLAG;
			goto errors;
		}
	}
	// Clear flag.
	I2C_DESCRIPTOR[instance].peripheral -> ICR |= (0b1 << 5); // STOPCF='1'.
errors:
	return status;
}
