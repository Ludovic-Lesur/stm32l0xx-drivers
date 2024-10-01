/*
 * adc.c
 *
 *  Created on: 12 aug. 2024
 *      Author: Ludo
 */

#include "adc.h"

#include "adc_reg.h"
#include "error.h"
#include "gpio.h"
#include "lptim.h"
#include "math.h"
#include "pwr_reg.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "types.h"

/*** ADC local macros ***/

#define ADC_MEDIAN_FILTER_SIZE				9
#define ADC_CENTER_AVERAGE_SIZE				3

#define ADC_VREFINT_VOLTAGE_MV				((VREFINT_CAL * VREFINT_VCC_CALIB_MV) / (ADC_FULL_SCALE_12BITS))
#define ADC_VREFINT_DEFAULT_12BITS			((VREFINT_CAL * VREFINT_VCC_CALIB_MV) / (ADC_VMCU_DEFAULT_MV))

#define ADC_LOW_FREQUENCY_MODE_THRESHOLD_HZ	3500000

#define ADC_TIMEOUT_COUNT					1000000

/*** ADC local structures ***/

/*******************************************************************/
typedef struct {
	uint8_t init_count;
} ADC_context_t;

/*** ADC local global variables ***/

static ADC_context_t adc_ctx = {.init_count = 0};

/*** ADC local functions ***/

/*******************************************************************/
static ADC_status_t _ADC_single_conversion(ADC_channel_t channel, int32_t* adc_data_12bits) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t loop_count = 0;
	// Select input channel.
	ADC1 -> CHSELR &= 0xFFF80000; // Reset all bits.
	ADC1 -> CHSELR |= (0b1 << channel);
	// Clear conversion flags.
	ADC1 -> ISR |= (0b1111 << 1);
	// Start conversion.
	ADC1 -> CR |= (0b1 << 2); // ADSTART='1'.
	// Wait for the conversion to complete.
	while (((ADC1 -> ISR) & (0b1 << 2)) == 0) {
		// Exit if timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_CONVERSION_TIMEOUT;
			goto errors;
		}
	}
	(*adc_data_12bits) = (int32_t) ((ADC1 -> DR) & ADC_FULL_SCALE);
errors:
	return status;
}

/*******************************************************************/
static ADC_status_t _ADC_disable(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	uint32_t loop_count = 0;
	// Clear all flags.
	ADC1 -> ISR |= 0x0000089F;
	// Check ADC state.
	if (((ADC1 -> CR) & (0b1 << 0)) == 0) goto errors; // Not an error but to exit directly.
	// Disable ADC.
	ADC1 -> CR |= (0b1 << 1); // ADDIS='1'.
	// Wait for ADC to be disabled.
	while (((ADC1 -> CR) & (0b1 << 0)) != 0) {
		// Exit if timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_DISABLE_TIMEOUT;
			break;
		}
	}
errors:
	return status;
}

/*** ADC functions ***/

/*******************************************************************/
ADC_status_t ADC_init(const ADC_gpio_t* pins) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	LPTIM_status_t lptim_status = LPTIM_SUCCESS;
	uint32_t adcclk_hz = 0;
	uint8_t idx = 0;
	uint32_t loop_count = 0;
	// Init GPIOs.
	if (pins != NULL) {
		for (idx=0 ; idx<(pins -> list_size) ; idx++) {
			GPIO_configure(&((pins -> list)[idx]), GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
		}
	}
	// Get system clock frequency.
	RCC_get_frequency_hz(RCC_CLOCK_SYSTEM, &adcclk_hz);
	adcclk_hz >>= 2;
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 9); // ADCEN='1'.
	// Ensure ADC is disabled.
	status = _ADC_disable();
	if (status != ADC_SUCCESS) goto errors;
	// Enable ADC voltage regulator.
	ADC1 -> CR |= (0b1 << 28);
	lptim_status = LPTIM_delay_milliseconds(ADC_INIT_DELAY_MS_REGULATOR, LPTIM_DELAY_MODE_ACTIVE);
	LPTIM_exit_error(ADC_ERROR_BASE_LPTIM);
	// ADC configuration.
	ADC1 -> CFGR2 |= (0b10 << 30); // Use (PCLK2/4) as source.
	ADC1 -> SMPR |= (0b111 << 0); // Maximum sampling time.
	// ADC clock.
	if (adcclk_hz < ADC_LOW_FREQUENCY_MODE_THRESHOLD_HZ) {
		// Enable low frequency mode.
		ADC1 -> CCR |= (0b1 << 25); // LFMEN='1'.
		ADC1 -> CCR &= ~(0b1111 << 18); // PRESC='0000'.
	}
	else {
		// Add prescaler.
		ADC1 -> CCR &= ~(0b1 << 25); // LFMEN='0'.
		ADC1 -> CCR |= (0b0010 << 18); // PRESC='0010'.
	}
	// ADC calibration.
	ADC1 -> CR |= (0b1 << 31); // ADCAL='1'.
	while ((((ADC1 -> CR) & (0b1 << 31)) != 0) && (((ADC1 -> ISR) & (0b1 << 11)) == 0)) {
		// Wait until calibration is done or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_CALIBRATION;
			break;
		}
	}
	// Enable ADC peripheral.
	ADC1 -> CR |= (0b1 << 0); // ADEN='1'.
	loop_count = 0;
	while (((ADC1 -> ISR) & (0b1 << 0)) == 0) {
		// Wait for ADC to be ready (ADRDY='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = ADC_ERROR_READY_TIMEOUT;
			goto errors;
		}
	}
	// Wake-up VREFINT and temperature sensor.
	ADC1 -> CCR |= (0b11 << 22); // TSEN='1' and VREFEN='1'.
	// Wait for startup.
	lptim_status = LPTIM_delay_milliseconds(ADC_INIT_DELAY_MS_VREF_TS, LPTIM_DELAY_MODE_ACTIVE);
	LPTIM_exit_error(ADC_ERROR_BASE_LPTIM);
	// Update initialization count.
	adc_ctx.init_count++;
errors:
	return status;
}

/*******************************************************************/
ADC_status_t ADC_de_init(void) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	// Update initialization count.
	if (adc_ctx.init_count > 0) {
		adc_ctx.init_count--;
	}
	// Check initialization count.
	if (adc_ctx.init_count > 0) goto errors;
	// Switch internal voltage reference off.
	ADC1 -> CCR &= ~(0b11 << 22); // TSEN='0' and VREFEF='0'.
	// Disable ADC peripheral.
	status = _ADC_disable();
	// Disable ADC voltage regulator.
	ADC1 -> CR &= ~(0b1 << 28);
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 9); // ADCEN='0'.
errors:
	return status;
}

/*******************************************************************/
ADC_status_t ADC_convert_channel(ADC_channel_t channel, int32_t* adc_data_12bits) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	MATH_status_t math_status = MATH_SUCCESS;
	int32_t adc_sample_buf[ADC_MEDIAN_FILTER_SIZE] = {0x00};
	uint8_t idx = 0;
	uint32_t loop_count = 0;
	// Check state.
	if (adc_ctx.init_count == 0) {
		status = ADC_ERROR_UNINITIALIZED;
		goto errors;
	}
	// Check parameters.
	if (channel >= ADC_CHANNEL_LAST) {
		status = ADC_ERROR_CHANNEL;
		goto errors;
	}
	if (adc_data_12bits == NULL) {
		status = ADC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	if (channel == ADC_CHANNEL_VREFINT) {
		// Wait for internal reference to be ready.
		while (((PWR -> CSR) & (0b1 << 3)) == 0) {
			// Wait for VREFINTRDYF or timeout.
			loop_count++;
			if (loop_count > ADC_TIMEOUT_COUNT) {
				status = ADC_ERROR_VREFINT_READY_TIMEOUT;
				goto errors;
			}
		}
	}
	// Perform all conversions.
	for (idx=0 ; idx<ADC_MEDIAN_FILTER_SIZE ; idx++) {
		status = _ADC_single_conversion(channel, &(adc_sample_buf[idx]));
		if (status != ADC_SUCCESS) goto errors;
	}
	// Apply median filter.
	math_status = MATH_median_filter(adc_sample_buf, ADC_MEDIAN_FILTER_SIZE, ADC_CENTER_AVERAGE_SIZE, adc_data_12bits);
	MATH_exit_error(ADC_ERROR_BASE_MATH);
errors:
	return status;
}

/*******************************************************************/
ADC_status_t ADC_compute_vmcu(int32_t ref_voltage_12bits, int32_t ref_voltage_mv, int32_t* vmcu_mv) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	// Check parameters.
	if (ref_voltage_12bits > ADC_FULL_SCALE) {
		status = ADC_ERROR_DATA;
		goto errors;
	}
	if (vmcu_mv == NULL) {
		status = ADC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	(*vmcu_mv) = (ref_voltage_mv * ADC_FULL_SCALE) / (ref_voltage_12bits);
errors:
	return status;
}

/*******************************************************************/
ADC_status_t ADC_compute_tmcu(int32_t vmcu_mv, int32_t tmcu_12bits, int32_t* tmcu_degrees) {
	// Local variables.
	ADC_status_t status = ADC_SUCCESS;
	int32_t raw_temp_calib_mv = 0;
	int32_t temp_calib_degrees = 0;
	// Check parameters.
	if (tmcu_12bits > ADC_FULL_SCALE) {
		status = ADC_ERROR_DATA;
		goto errors;
	}
	if (tmcu_degrees == NULL) {
		status = ADC_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Compute temperature according to MCU factory calibration.
	raw_temp_calib_mv = ((tmcu_12bits * vmcu_mv) / (ADC_TS_VCC_CALIB_MV)) - ADC_TS_CAL1;
	temp_calib_degrees = raw_temp_calib_mv * (ADC_TS_CAL2_TEMP - ADC_TS_CAL1_TEMP);
	temp_calib_degrees = (temp_calib_degrees) / (ADC_TS_CAL2 - ADC_TS_CAL1);
	(*tmcu_degrees) = temp_calib_degrees + ADC_TS_CAL1_TEMP;
errors:
	return status;
}

/*******************************************************************/
int32_t ADC_get_vrefint_voltage_mv(void) {
	// Local variables.
	int32_t vrefint_mv = ((ADC_VREFINT_CAL * ADC_VREFINT_VCC_CALIB_MV) / (ADC_FULL_SCALE));
	return vrefint_mv;
}
