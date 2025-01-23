/*
 * pwr.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __PWR_H__
#define __PWR_H__

#include "types.h"

/*** PWR functions ***/

/*!******************************************************************
 * \fn void PWR_init(void)
 * \brief Init PWR interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_init(void);

/*!******************************************************************
 * \fn void PWR_enter_sleep_mode(void)
 * \brief Enter sleep mode.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_enter_sleep_mode(void);

/*!******************************************************************
 * \fn void PWR_enter_low_power_sleep_mode(void)
 * \brief Enter low power sleep mode.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_enter_low_power_sleep_mode(void);

/*!******************************************************************
 * \fn void PWR_enter_stop_mode(void)
 * \brief Enter stop mode.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void PWR_enter_stop_mode(void);

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
