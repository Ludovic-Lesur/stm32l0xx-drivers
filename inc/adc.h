/*
 * adc.h
 *
 *  Created on: 12 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#ifndef __ADC_H__
#define __ADC_H__

#include "error.h"
#include "gpio.h"
#include "lptim.h"
#include "maths.h"
#include "types.h"

/*** ADC macros ***/

#define ADC_RESOLUTION_BITS             12
#define ADC_FULL_SCALE                  ((1 << ADC_RESOLUTION_BITS) - 1)

#define ADC_INIT_DELAY_MS_REGULATOR     5
#define ADC_INIT_DELAY_MS_VREF_TS       10

#define ADC_INIT_DELAY_MS               (ADC_INIT_DELAY_MS_REGULATOR + ADC_INIT_DELAY_MS_VREF_TS)

/*** ADC structures ***/

/*!******************************************************************
 * \enum ADC_status_t
 * \brief ADC driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    ADC_SUCCESS = 0,
    ADC_ERROR_NULL_PARAMETER,
    ADC_ERROR_UNINITIALIZED,
    ADC_ERROR_DISABLE_TIMEOUT,
    ADC_ERROR_CALIBRATION,
    ADC_ERROR_READY_TIMEOUT,
    ADC_ERROR_VREFINT_READY_TIMEOUT,
    ADC_ERROR_CHANNEL,
    ADC_ERROR_DATA,
    ADC_ERROR_CONVERSION_TIMEOUT,
    // Low level drivers errors.
    ADC_ERROR_BASE_LPTIM = ERROR_BASE_STEP,
    ADC_ERROR_BASE_MATH = (ADC_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST),
    // Last base value.
    ADC_ERROR_BASE_LAST = (ADC_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} ADC_status_t;

/*!******************************************************************
 * \enum ADC_channel_t
 * \brief ADC channels list.
 *******************************************************************/
typedef enum {
    // External channels.
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
    // Internal channels.
    ADC_CHANNEL_VLCD,
    ADC_CHANNEL_VREFINT,
    ADC_CHANNEL_TEMPERATURE_SENSOR,
    // Last index.
    ADC_CHANNEL_LAST
} ADC_channel_t;

/*!******************************************************************
 * \struct ADC_gpio_t
 * \brief ADC GPIO pins list.
 *******************************************************************/
typedef struct {
    const GPIO_pin_t** list;
    uint8_t list_size;
} ADC_gpio_t;

/*** ADC functions ***/

/*!******************************************************************
 * \fn ADC_status_t ADC_init(const ADC_gpio_t* pins)
 * \brief Init ADC peripheral.
 * \param[in]   pins: List of ADC pins to use.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_init(const ADC_gpio_t* pins);

/*!******************************************************************
 * \fn ADC_status_t ADC_de_init(void)
 * \brief Release ADC peripheral.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_de_init(void);

/*!******************************************************************
 * \fn ADC_status_t ADC_convert_channel(ADC_channel_t channel, int32_t_t* adc_data_12bits)
 * \brief Perform a channel conversion.
 * \param[in]   channel: Channel to convert.
 * \param[out]  adc_data_12bits: Pointer to integer that will contain the 12-bits ADC data.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_convert_channel(ADC_channel_t channel, int32_t* adc_data_12bits);

/*!******************************************************************
 * \fn ADC_status_t ADC_compute_mcu_voltage(int32_t_t reference_voltage_12bits, int32_t_t reference_voltage_mv, int32_t* mcu_voltage_mv)
 * \brief Compute MCU voltage.
 * \param[in]   reference_voltage_12bits: Reference voltage 12-bits raw data from ADC.
 * \param[in]   reference_voltage_mv: Reference voltage in mV.
 * \param[out]  mcu_voltage_mv: Pointer to integer that will contain the MCU voltage in mV.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_compute_mcu_voltage(int32_t reference_voltage_12bits, int32_t reference_voltage_mv, int32_t* mcu_voltage_mv);

/*!******************************************************************
 * \fn ADC_status_t ADC_compute_mcu_temperature(int32_t mcu_voltage_mv, int32_t_t mcu_temperature_12bits, int32_t* mcu_temperature_degrees)
 * \brief Compute MCU temperature.
 * \param[in]   mcu_voltage_mv: MCU supply voltage in mV.
 * \param[in]   mcu_temperature_12bits: Temperature sensor 12-bits raw data from ADC.
 * \param[out]  mcu_temperature_degrees: Pointer to integer that will contain MCU temperature in 2's complement format.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_compute_mcu_temperature(int32_t mcu_voltage_mv, int32_t mcu_temperature_12bits, int32_t* mcu_temperature_degrees);

/*!******************************************************************
 * \fn int32_t ADC_get_vrefint_voltage_mv(void)
 * \brief Get internal reference voltage.
 * \param[in]   none
 * \param[out]  none
 * \retval      Internal reference voltage in mV.
 *******************************************************************/
int32_t ADC_get_vrefint_voltage_mv(void);

/*!******************************************************************
 * \fn ADC_status_t ADC_get_master_dr_register_address(uint32_t* dr_register_address)
 * \brief Get master ADC data register address.
 * \param[in]   none
 * \param[out]  dr_register_address: Pointer to integer that will contain the ADC DR register address.
 * \retval      Function execution status.
 *******************************************************************/
ADC_status_t ADC_get_dr_register_address(uint32_t* dr_register_address);

/*******************************************************************/
#define ADC_exit_error(base) { ERROR_check_exit(adc_status, ADC_SUCCESS, base) }

/*******************************************************************/
#define ADC_stack_error(base) { ERROR_check_stack(adc_status, ADC_SUCCESS, base) }

/*******************************************************************/
#define ADC_stack_exit_error(base, code) { ERROR_check_stack_exit(adc_status, ADC_SUCCESS, base, code) }

#endif /* __ADC_H__ */

#endif /* STM32L0XX_DRIVERS_DISABLE */
