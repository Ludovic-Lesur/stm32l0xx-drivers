/*
 * adc.h
 *
 *  Created on: 12 aug. 2024
 *      Author: Ludo
 */

#ifndef __ADC_H__
#define __ADC_H__

#include "gpio.h"
#include "lptim.h"
#include "math.h"
#include "types.h"

/*** ADC macros ***/

#define ADC_FULL_SCALE					4095

#define ADC_INIT_DELAY_MS_REGULATOR		5
#define ADC_INIT_DELAY_MS_VREF_TS		10

#define ADC_INIT_DELAY_MS				(ADC_INIT_DELAY_MS_REGULATOR + ADC_INIT_DELAY_MS_VREF_TS)

/*** ADC structures ***/

/*!******************************************************************
 * \enum ADC_status_t
 * \brief ADC driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	ADC_SUCCESS = 0,
	ADC_ERROR_NULL_PARAMETER,
	ADC_ERROR_DISABLE_TIMEOUT,
	ADC_ERROR_CALIBRATION,
	ADC_ERROR_READY_TIMEOUT,
	ADC_ERROR_CHANNEL,
	ADC_ERROR_DATA,
	ADC_ERROR_CONVERSION_TIMEOUT,
	// Low level drivers errors.
	ADC_ERROR_BASE_LPTIM = 0x0100,
	ADC_ERROR_BASE_MATH = (ADC_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
	// Last base value.
	ADC_ERROR_BASE_LAST = (ADC_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} ADC_status_t;

/*!******************************************************************
 * \enum ADC_channel_t
 * \brief ADC channels list.
 *******************************************************************/
typedef enum {
	ADC_CHANNEL_IN0 = 0,
	ADC_CHANNEL_IN1,
	ADC_CHANNEL_IN2,
	ADC_CHANNEL_IN3,
	ADC_CHANNEL_IN4,
	ADC_CHANNEL_IN5,
	ADC_CHANNEL_IN6,
	ADC_CHANNEL_IN7,
	ADC_CHANNEL_IN8,
	ADC_CHANNEL_IN9,
	ADC_CHANNEL_IN10,
	ADC_CHANNEL_IN11,
	ADC_CHANNEL_IN12,
	ADC_CHANNEL_IN13,
	ADC_CHANNEL_IN14,
	ADC_CHANNEL_IN15,
	ADC_CHANNEL_VLCD,
	ADC_CHANNEL_VREFINT,
	ADC_CHANNEL_TEMPERATURE_SENSOR,
	ADC_CHANNEL_LAST
} ADC_channel_t;

/*!******************************************************************
 * \struct ADC_gpio_t
 * \brief ADC GPIO pins list.
 *******************************************************************/
typedef struct {
	const GPIO_pin_t* list;
	uint8_t list_size;
} ADC_gpio_t;

/*** ADC functions ***/

/*!******************************************************************
 * \fn ADC_status_t ADC_init(const ADC_gpio_t* pins)
 * \brief Init ADC peripheral.
 * \param[in]  	pins: List of ADC pins to use.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_init(const ADC_gpio_t* pins);

/*!******************************************************************
 * \fn ADC_status_t ADC_de_init(void)
 * \brief Release ADC peripheral.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_de_init(void);

/*!******************************************************************
 * \fn ADC_status_t ADC_convert_channel(ADC_channel_t channel, uint16_t* adc_data_12bits)
 * \brief Perform a channel conversion.
 * \param[in]  	channel: Channel to convert.
 * \param[out] 	adc_data_12bits: Pointer to short that will contain the 12-bits ADC data.
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_convert_channel(ADC_channel_t channel, uint16_t* adc_data_12bits);

/*!******************************************************************
 * \fn ADC_status_t ADC_compute_vmcu(uint16_t ref_voltage_12bits, uint16_t ref_voltage_mv, int32_t* vmcu_mv)
 * \brief Compute MCU voltage.
 * \param[in]  	ref_voltage_12bits: Reference voltage 12-bits raw data from ADC.
 * \param[in]	ref_voltage_mv: Reference voltage in mV.
 * \param[out] 	vmcu_mv: Pointer to integer that will contain the MCU voltage in mV.
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_compute_vmcu(uint16_t ref_voltage_12bits, uint16_t ref_voltage_mv, int32_t* vmcu_mv);

/*!******************************************************************
 * \fn ADC_status_t ADC_compute_tmcu(int32_t vmcu_mv, uint16_t tmcu_12bits, int32_t* tmcu_degrees)
 * \brief Compute MCU temperature.
 * \param[in]	vmcu_mv: MCU supply voltage in mV.
 * \param[in]  	tmcu_12bits: Temperature sensor 12-bits raw data from ADC.
 * \param[out] 	tmcu_degrees: Pointer to integer that will contain MCU temperature in 2's complement format.
 * \retval		Function execution status.
 *******************************************************************/
ADC_status_t ADC_compute_tmcu(int32_t vmcu_mv, uint16_t tmcu_12bits, int32_t* tmcu_degrees);

/*!******************************************************************
 * \fn int32_t ADC_get_vrefint_voltage_mv(void)
 * \brief Get internal reference voltage.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Internal reference voltage in mV.
 *******************************************************************/
int32_t ADC_get_vrefint_voltage_mv(void);

/*******************************************************************/
#define ADC_exit_error(base) { if (adc_status != ADC_SUCCESS) { status = (base + adc_status); goto errors; } }

/*******************************************************************/
#define ADC_stack_error(base) { if (adc_status != ADC_SUCCESS) { ERROR_stack_add(base + adc_status); } }

/*******************************************************************/
#define ADC_stack_exit_error(base, code) { if (adc_status != ADC_SUCCESS) { ERROR_stack_add(base + adc_status); status = code; goto errors; } }

#endif /* __ADC_H__ */
