/*
 * rcc.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __RCC_H__
#define __RCC_H__

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "gpio.h"
#include "flash.h"
#include "types.h"

/*** RCC structures ***/

/*!******************************************************************
 * \enum RCC_status_t
 * \brief RCC driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    RCC_SUCCESS = 0,
    RCC_ERROR_NULL_PARAMETER,
    RCC_ERROR_CLOCK,
    RCC_ERROR_HSI_READY,
    RCC_ERROR_HSI_SWITCH,
    RCC_ERROR_HSE_READY,
    RCC_ERROR_HSE_SWITCH,
    RCC_ERROR_MSI_RANGE,
    RCC_ERROR_MSI_READY,
    RCC_ERROR_MSI_SWITCH,
    RCC_ERROR_LSE_READY,
    RCC_ERROR_CALIBRATION_TIMER,
    RCC_ERROR_CALIBRATION_HSI,
    RCC_ERROR_CALIBRATION_LSI,
    RCC_ERROR_MCO_PRESCALER,
    // Low level drivers errors.
    RCC_ERROR_BASE_FLASH = 0x0100,
    // Last base value.
    RCC_ERROR_BASE_LAST = (RCC_ERROR_BASE_FLASH + FLASH_ERROR_BASE_LAST)
} RCC_status_t;

/*!******************************************************************
 * \enum RCC_clock_t
 * \brief RCC clocks list.
 *******************************************************************/
typedef enum {
    RCC_CLOCK_NONE = 0,
    RCC_CLOCK_SYSTEM,
    RCC_CLOCK_HSI,
    RCC_CLOCK_MSI,
    RCC_CLOCK_HSE,
    RCC_CLOCK_PLL,
    RCC_CLOCK_LSI,
    RCC_CLOCK_LSE,
    RCC_CLOCK_LAST
} RCC_clock_t;

#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
/*!******************************************************************
 * \enum RCC_hse_mode_t
 * \brief RCC external oscillator modes.
 *******************************************************************/
typedef enum {
    RCC_HSE_MODE_OSCILLATOR = 0,
    RCC_HSE_MODE_BYPASS,
    RCC_HSE_MODE_LAST
} RCC_hse_mode_t;
#endif

/*!******************************************************************
 * \enum RCC_msi_range_t
 * \brief RCC MSI oscillator frequency ranges.
 *******************************************************************/
typedef enum {
    RCC_MSI_RANGE_0_65KHZ = 0,
    RCC_MSI_RANGE_1_131KHZ,
    RCC_MSI_RANGE_2_262KHZ,
    RCC_MSI_RANGE_3_524KHZ,
    RCC_MSI_RANGE_4_1MHZ,
    RCC_MSI_RANGE_5_2MHZ,
    RCC_MSI_RANGE_6_4MHZ,
    RCC_MSI_RANGE_LAST
} RCC_msi_range_t;

/*!******************************************************************
 * \enum RCC_mco_prescaler_t
 * \brief RCC MCO clock output prescaler.
 *******************************************************************/
typedef enum {
    RCC_MCO_PRESCALER_1 = 0,
    RCC_MCO_PRESCALER_2,
    RCC_MCO_PRESCALER_4,
    RCC_MCO_PRESCALER_8,
    RCC_MCO_PRESCALER_16,
    RCC_MCO_PRESCALER_LAST
} RCC_mco_prescaler_t;

/*** RCC functions ***/

/*!******************************************************************
 * \fn RCC_status_t RCC_init(uint8_t nvic_priority)
 * \brief Init MCU default clock tree.
 * \param[in]   nvic_priority: Interrupt priority.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_init(uint8_t nvic_priority);

/*!******************************************************************
 * \fn RCC_status_t RCC_switch_to_hsi(void)
 * \brief Switch system clock to 16MHz HSI.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_switch_to_hsi(void);

#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
/*!******************************************************************
 * \fn RCC_status_t RCC_switch_to_hse(RCC_hse_mode_t hse_mode)
 * \brief Switch system clock to external oscillator.
 * \param[in]   hse_mode: External oscillator mode.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_switch_to_hse(RCC_hse_mode_t hse_mode);
#endif

/*!******************************************************************
 * \fn RCC_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range)
 * \brief Switch system clock to MSI.
 * \param[in]   msi_range: MSI frequency to set.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range);

/*!******************************************************************
 * \fn RCC_status_t RCC_restore_previous_system_clock(void)
 * \brief Restore the system clock configuration used before the last clock switch.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_restore_previous_system_clock(void);

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*!******************************************************************
 * \fn RCC_status_t RCC_calibrate_internal_clocks(uint8_t nvic_priority)
 * \brief Measure internal oscillators frequency with external reference clock. Warning: this function temporarily switches the system clock to HSI.
 * \param[in]   nvic_priority: Calibration timer interrupt priority.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_calibrate_internal_clocks(uint8_t nvic_priority);
#endif

/*!******************************************************************
 * \fn RCC_clock_t RCC_get_system_clock(void)
 * \brief Get current system clock source.
 * \param[in]   none
 * \param[out]  none
 * \retval      Current system clock source.
 *******************************************************************/
RCC_clock_t RCC_get_system_clock(void);

/*!******************************************************************
 * \fn RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz)
 * \brief Get clock frequency.
 * \param[in]   clock: Clock to read.
 * \param[out]  frequency_hz: Pointer to the clock frequency in Hz.
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz);

/*!******************************************************************
 * \fn RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready)
 * \brief Get clock status.
 * \param[in]   clock: Clock to read.
 * \param[out]  clock_is_ready: Pointer to the clock status (1 if the clock is running correctly, 0 otherwise).
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready);

/*!******************************************************************
 * \fn RCC_status_t RCC_set_mco(RCC_clock_t mco_clock, RCC_mco_prescaler_t mco_prescaler, const GPIO_pin_t* mco_gpio)
 * \brief Set MCO clock output signal.
 * \param[in]   clock: Clock to select.
 * \param[in]   prescaler: Clock output prescaler.
 * \param[in]   gpio: Optional GPIO to link to MCO.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RCC_status_t RCC_set_mco(RCC_clock_t mco_clock, RCC_mco_prescaler_t mco_prescaler, const GPIO_pin_t* mco_gpio);

/*******************************************************************/
#define RCC_exit_error(base) { ERROR_check_exit(rcc_status, RCC_SUCCESS, base) }

/*******************************************************************/
#define RCC_stack_error(base) { ERROR_check_stack(rcc_status, RCC_SUCCESS, base) }

/*******************************************************************/
#define RCC_stack_exit_error(base, code) { ERROR_check_stack_exit(rcc_status, RCC_SUCCESS, base, code) }

#endif /* __RCC_H__ */
