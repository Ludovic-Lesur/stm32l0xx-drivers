/*
 * pwr.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#ifndef __PWR_H__
#define __PWR_H__

#include "types.h"

/*** PWR structures ***/

/*!******************************************************************
 * \enum PWR_sleep_mode_t
 * \brief Sleep modes list.
 *******************************************************************/
typedef enum {
    PWR_SLEEP_MODE_NORMAL = 0,
    PWR_SLEEP_MODE_LOW_POWER,
    PWR_SLEEP_MODE_LAST
} PWR_sleep_mode_t;

/*!******************************************************************
 * \enum PWR_deepsleep_mode_t
 * \brief Deep sleep modes list.
 *******************************************************************/
typedef enum {
    PWR_DEEPSLEEP_MODE_STOP = 0,
    PWR_DEEPSLEEP_MODE_STANDBY,
    PWR_DEEPSLEEP_MODE_LAST
} PWR_deepsleep_mode_t;

/*** PWR functions ***/

/*!******************************************************************
 * \fn void PWR_init(void)
 * \brief Init power interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_init(void);

/*!******************************************************************
 * \fn void PWR_de_init(void)
 * \brief Release power interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_de_init(void);

/*!******************************************************************
 * \fn void PWR_enter_sleep_mode(PWR_sleep_mode_t sleep_mode)
 * \brief Enter sleep mode.
 * \param[in]   mode: Sleep mode to enter.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_enter_sleep_mode(PWR_sleep_mode_t sleep_mode);

/*!******************************************************************
 * \fn void PWR_enter_deepsleep_mode(PWR_deepsleep_mode_t deepsleep_mode)
 * \brief Enter deep sleep mode.
 * \param[in]   mode: Deep sleep mode to enter.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_enter_deepsleep_mode(PWR_deepsleep_mode_t deepsleep_mode);

/*!******************************************************************
 * \fn void PWR_software_reset(void)
 * \brief Reset the MCU.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_software_reset(void);

/*!******************************************************************
 * \fn uint8_t PWR_get_reset_flags(void)
 * \brief Read the MCU reset flags.
 * \param[in]   none
 * \param[out]  none
 * \retval      MCU reset flags.
 *******************************************************************/
uint8_t PWR_get_reset_flags(void);

/*!******************************************************************
 * \fn void PWR_clear_reset_flags(void)
 * \brief Clear the MCU reset flags.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_clear_reset_flags(void);

#endif /* __PWR_H__ */

#endif /* STM32L0XX_DRIVERS_DISABLE */
