/*
 * rcc.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "rcc.h"

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "error.h"
#include "flash.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "tim.h"
#include "types.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT				1000000

#define RCC_LSI_FREQUENCY_DEFAULT_HZ	38000
#define RCC_LSI_FREQUENCY_MIN_HZ		26000
#define RCC_LSI_FREQUENCY_MAX_HZ		56000

#define RCC_HSI_FREQUENCY_DEFAULT_HZ	16000000
#define RCC_HSI_FREQUENCY_MIN_HZ		15040000
#define RCC_HSI_FREQUENCY_MAX_HZ		16960000

/*** RCC local structures ***/

/*******************************************************************/
typedef struct {
	RCC_clock_t sysclk_source;
	uint32_t clock_frequency[RCC_CLOCK_LAST];
} RCC_context_t;

/*** RCC local global variables ***/

static const uint32_t RCC_MSI_CLOCK_FREQUENCY[RCC_MSI_RANGE_LAST] = {65536, 131072, 262144, 524288, 1048000, 2097000, 4194000};
static RCC_context_t rcc_ctx;

/*** RCC local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RCC_IRQHandler(void) {
	// Clear flag.
	RCC -> CICR |= (0b1 << 0);
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _RCC_reset_backup_domain(void) {
	// Local variables.
	uint8_t count = 0;
	// Perform manual reset and delay.
	RCC -> CSR |= (0b1 << 19); // RTCRST='1'.
	for (count=0 ; count<100 ; count++);
	RCC -> CSR &= ~(0b1 << 19); // RTCRST='0'.
}

/*******************************************************************/
void _RCC_enable_lsi(void) {
	// Enable LSI.
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	// Enable interrupt.
	RCC -> CIER |= (0b1 << 0);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC_CRS);
	// Wait for LSI to be stable.
	while (((RCC -> CSR) & (0b1 << 1)) == 0) {
		PWR_enter_sleep_mode();
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC_CRS);
}

#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
/*******************************************************************/
RCC_status_t _RCC_enable_lse(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	uint32_t loop_count = 0;
	// Enable LSE (32.768kHz crystal).
	RCC -> CSR |= (0b1 << 8); // LSEON='1'.
	// Wait for LSE to be stable.
	while (((RCC -> CSR) & (0b1 << 9)) == 0) {
		// Wait for LSERDY='1' ready flag or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			// Switch LSE off.
			RCC -> CSR &= ~(0b1 << 8); // LSEON='0'.
			// Exit loop.
			status = RCC_ERROR_LSE_READY;
			goto errors;
		}
	}
errors:
	return status;
}
#endif

#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 2)
/*******************************************************************/
void _RCC_enable_lse(void) {
	// Enable LSE (32.768kHz crystal).
	RCC -> CSR |= (0b1 << 8); // LSEON='1'.
	// Enable interrupt.
	RCC -> CIER |= (0b1 << 1);
	NVIC_enable_interrupt(NVIC_INTERRUPT_RCC_CRS);
	// Wait for LSE to be stable.
	while (((RCC -> CSR) & (0b1 << 9)) == 0) {
		PWR_enter_sleep_mode();
	}
	NVIC_disable_interrupt(NVIC_INTERRUPT_RCC_CRS);
}
#endif

/*** RCC functions ***/

/*******************************************************************/
RCC_status_t RCC_init(uint8_t nvic_priority) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	// Init context.
	rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = RCC_LSI_FREQUENCY_DEFAULT_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_LSE] = STM32L0XX_DRIVERS_RCC_LSE_FREQUENCY_HZ;
	rcc_ctx.clock_frequency[RCC_CLOCK_MSI] = RCC_MSI_CLOCK_FREQUENCY[RCC_MSI_RANGE_5_2MHZ];
	rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = RCC_HSI_FREQUENCY_DEFAULT_HZ;
	// Update system clock.
	rcc_ctx.sysclk_source = RCC_CLOCK_MSI;
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	// Reset backup domain.
	_RCC_reset_backup_domain();
	// Set interrupt priority.
	NVIC_set_priority(NVIC_INTERRUPT_RCC_CRS, nvic_priority);
	// Start low speed oscillators.
	_RCC_enable_lsi();
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
	status = _RCC_enable_lse();
#endif
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 2)
	_RCC_enable_lse();
#endif
	return status;
}

/*******************************************************************/
RCC_status_t RCC_switch_to_hsi(void) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	uint32_t reg_cfgr = 0;
	uint32_t loop_count = 0;
	// Set flash latency.
	flash_status = FLASH_set_latency(1);
	FLASH_exit_error(RCC_ERROR_BASE_FLASH);
	// Enable HSI.
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').
	// Wait for HSI to be stable.
	while (((RCC -> CR) & (0b1 << 2)) == 0) {
		// Wait for HSIRDYF='1' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_READY;
			goto errors;
		}
	}
	// Switch SYSCLK.
	reg_cfgr = (RCC -> CFGR);
	reg_cfgr &= ~(0b11 << 0); // Reset bits 0-1.
	reg_cfgr |= (0b01 << 0); // Use HSI as system clock (SW='01').
	RCC -> CFGR = reg_cfgr;
	// Wait for clock switch.
	loop_count = 0;
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)) {
		// Wait for SWS='01' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_HSI_SWITCH;
			goto errors;
		}
	}
	// Update clocks context.
	rcc_ctx.sysclk_source = RCC_CLOCK_HSI;
errors:
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}

/*******************************************************************/
RCC_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	FLASH_status_t flash_status = FLASH_SUCCESS;
	uint32_t loop_count = 0;
	// Check parameter.
	if (msi_range >= RCC_MSI_RANGE_LAST) {
		status = RCC_ERROR_MSI_RANGE;
		goto errors;
	}
	// Set frequency.
	RCC -> ICSCR &= ~(0b111 << 13);
	RCC -> ICSCR |= (msi_range << 13);
	// Enable MSI.
	RCC -> CR |= (0b1 << 8); // MSION='1'.
	// Wait for MSI to be stable.
	while (((RCC -> CR) & (0b1 << 9)) == 0) {
		// Wait for MSIRDYF='1' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_MSI_READY;
			goto errors;
		}
	}
	// Switch SYSCLK.
	RCC -> CFGR &= ~(0b11 << 0); // Use MSI as system clock (SW='00').
	// Wait for clock switch.
	while (((RCC -> CFGR) & (0b11 << 2)) != (0b00 << 2)) {
		// Wait for SWS='00' or timeout.
		loop_count++;
		if (loop_count > RCC_TIMEOUT_COUNT) {
			status = RCC_ERROR_MSI_SWITCH;
			goto errors;
		}
	}
	// Set flash latency.
	flash_status = FLASH_set_latency(0);
	FLASH_exit_error(RCC_ERROR_BASE_FLASH);
	// Update clocks context.
	rcc_ctx.sysclk_source = RCC_CLOCK_MSI;
	rcc_ctx.clock_frequency[RCC_CLOCK_MSI] = RCC_MSI_CLOCK_FREQUENCY[msi_range];
errors:
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*******************************************************************/
RCC_status_t RCC_calibrate(uint8_t nvic_priority) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	TIM_status_t tim_status = TIM_SUCCESS;
	int32_t ref_clock_pulse_count = 0;
	int32_t mco_pulse_count = 0;
	uint64_t temp_u64 = 0;
	uint32_t clock_frequency_hz = 0;
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE > 0)
	uint8_t lse_status = 0;
#endif
	// Init measurement timer.
	TIM_CAL_init(TIM_INSTANCE_TIM21, nvic_priority);
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE > 0)
	// Check LSE status.
	status = RCC_get_status(RCC_CLOCK_LSE, &lse_status);
	if (status != RCC_SUCCESS) goto errors;
	// HSI calibration is not possible without LSE.
	if (lse_status == 0) goto lsi_calibration;
	// Connect MCO to LSE clock.
	RCC -> CFGR &= ~(0x7F << 24);
	RCC -> CFGR |= (0b0111 << 24);
	// Perform measurement.
	tim_status = TIM_CAL_mco_capture(TIM_INSTANCE_TIM21, &ref_clock_pulse_count, &mco_pulse_count);
	// Compute HSI frequency.
	temp_u64 = ((uint64_t) STM32L0XX_DRIVERS_RCC_LSE_FREQUENCY_HZ * (uint64_t) ref_clock_pulse_count);
	clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) mco_pulse_count));
	// Check value.
	if ((tim_status != TIM_SUCCESS) || (clock_frequency_hz < RCC_HSI_FREQUENCY_MIN_HZ) || (clock_frequency_hz > RCC_HSI_FREQUENCY_MAX_HZ)) {
		status = RCC_ERROR_HSI_CALIBRATION;
		goto errors;
	}
	// Update local data.
	rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = clock_frequency_hz;
lsi_calibration:
#endif
	// Connect MCO to LSI clock.
	RCC -> CFGR &= ~(0x7F << 24);
	RCC -> CFGR |= (0b0110 << 24);
	// Perform measurement.
	tim_status = TIM_CAL_mco_capture(TIM_INSTANCE_TIM21, &ref_clock_pulse_count, &mco_pulse_count);
	// Compute LSI frequency.
	temp_u64 = ((uint64_t) rcc_ctx.clock_frequency[RCC_CLOCK_HSI] * (uint64_t) mco_pulse_count);
	clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) ref_clock_pulse_count));
	// Check value.
	if ((tim_status != TIM_SUCCESS) || (clock_frequency_hz < RCC_LSI_FREQUENCY_MIN_HZ) || (clock_frequency_hz > RCC_LSI_FREQUENCY_MAX_HZ)) {
		status = RCC_ERROR_LSI_CALIBRATION;
		goto errors;
	}
	// Update local data.
	rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = clock_frequency_hz;
errors:
	// Release timer.
	TIM_CAL_de_init(TIM_INSTANCE_TIM21);
	// Update system clock.
	rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.sysclk_source];
	return status;
}
#endif

/*******************************************************************/
RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	// Check parameters.
	if (clock >= RCC_CLOCK_LAST) {
		status = RCC_ERROR_CLOCK;
		goto errors;
	}
	if (frequency_hz == NULL) {
		status = RCC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Read frequency.
	(*frequency_hz) = rcc_ctx.clock_frequency[clock];
errors:
	return status;
}

/*******************************************************************/
RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready) {
	// Local variables.
	RCC_status_t status = RCC_SUCCESS;
	// Check parameters.
	if (clock_is_ready == NULL) {
		status = RCC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Check clock.
	switch (clock) {
	case RCC_CLOCK_LSI:
		(*clock_is_ready) = (((RCC -> CSR) >> 1) & 0b1);
		break;
	case RCC_CLOCK_LSE:
		(*clock_is_ready) = (((RCC -> CSR) >> 9) & 0b1);
		break;
	case RCC_CLOCK_MSI:
		(*clock_is_ready) = (((RCC -> CR) >> 9) & 0b1);
		break;
	case RCC_CLOCK_HSI:
		(*clock_is_ready) = (((RCC -> CR) >> 2) & 0b1);
		break;
	case RCC_CLOCK_SYSTEM:
		(*clock_is_ready) = 1;
		break;
	default:
		status = RCC_ERROR_CLOCK;
		goto errors;
	}
errors:
	return status;
}
