/*
 * lpuart.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __LPUART_H__
#define __LPUART_H__

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "gpio.h"
#include "types.h"

/*** LPUART structures ***/

/*!******************************************************************
 * \enum LPUART_status_t
 * \brief LPUART driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	LPUART_SUCCESS = 0,
	LPUART_ERROR_NULL_PARAMETER,
	LPUART_ERROR_BAUD_RATE,
	LPUART_ERROR_RX_MODE,
	LPUART_ERROR_TX_TIMEOUT,
	LPUART_ERROR_TC_TIMEOUT,
	// Last base value.
	LPUART_ERROR_BASE_LAST = 0x0100
} LPUART_status_t;

/*!******************************************************************
 * \enum LPUART_gpio_t
 * \brief LPUART GPIO pins list.
 *******************************************************************/
typedef struct {
	const GPIO_pin_t* tx;
	const GPIO_pin_t* rx;
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	const GPIO_pin_t* de;
	const GPIO_pin_t* nre;
#endif
} LPUART_gpio_t;

#if ((STM32L0XX_DRIVERS_LPUART_MODE == 0) || (STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
/*!******************************************************************
 * \fn LPUART_rx_irq_cb_t
 * \brief LPUART RX interrupt callback.
 *******************************************************************/
typedef void (*LPUART_rx_irq_cb_t)(uint8_t data);
#endif

#if (STM32L0XX_DRIVERS_LPUART_MODE == 1)
/*!******************************************************************
 * \fn LPUART_character_match_irq_cb_t
 * \brief LPUART character match interrupt callback.
 *******************************************************************/
typedef void (*LPUART_character_match_irq_cb_t)(void);
#endif

#if (STM32L0XX_DRIVERS_LPUART_MODE == 3)
/*!******************************************************************
 * \enum LPUART_rx_mode_t
 * \brief LPUART RX modes list.
 *******************************************************************/
typedef enum {
	LPUART_RX_MODE_ADDRESSED = 0,
	LPUART_RX_MODE_DIRECT,
	LPUART_RX_MODE_LAST
} LPUART_rx_mode_t;
#endif

/*!******************************************************************
 * \struct LPUART_configuration_t
 * \brief LPUART configuration structure.
 *******************************************************************/
typedef struct {
	uint32_t baud_rate;
	uint8_t nvic_priority;
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 0) || (STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	LPUART_rx_irq_cb_t rxne_callback;
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 1)
	char_t match_character;
	LPUART_character_match_irq_cb_t cmf_callback;
#endif
#if ((STM32L0XX_DRIVERS_LPUART_MODE == 2) || (STM32L0XX_DRIVERS_LPUART_MODE == 3))
	uint8_t self_address;
#endif
#if (STM32L0XX_DRIVERS_LPUART_MODE == 3)
	LPUART_rx_mode_t rx_mode;
#endif
} LPUART_configuration_t;

/*** LPUART functions ***/

/*!******************************************************************
 * \fn LPUART_status_t LPUART_init(const LPUART_gpio_t* pins, LPUART_configuration_t* configuration)
 * \brief Init LPUART peripheral.
 * \param[in]	pins: Pointer to the LPUART pins to use.
 * \param[in]  	configuration: Pointer to the LPUART configuration structure.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_init(const LPUART_gpio_t* pins, LPUART_configuration_t* configuration);

/*!******************************************************************
 * \fn LPUART_status_t LPUART_de_init(LPUART_gpio_t* pins)
 * \brief Release LPUART peripheral.
 * param[in]	pins: Pointer to the LPUART pins to release.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_de_init(const LPUART_gpio_t* pins);

/*!******************************************************************
 * \fn LPUART_status_t LPUART_enable_rx(void)
 * \brief Enable LPUART RX operation.
 * \param[in]	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LPUART_enable_rx(void);

/*!******************************************************************
 * \fn LPUART_status_t LPUART_disable_rx(void)
 * \brief Disable LPUART RX operation.
 * \param[in]	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void LPUART_disable_rx(void);

/*!******************************************************************
 * \fn LPUART_status_t LPUART_write(uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over LPUART.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_write(uint8_t* data, uint32_t data_size_bytes);

#if (STM32L0XX_DRIVERS_LPUART_MODE == 3)
/*!******************************************************************
 * \fn LPUART_status_t LPUART_set_configuration(LPUART_configuration_t* configuration)
 * \brief Switch LPUART configuration.
 * \param[in]  	configuration: Pointer to the LPUART configuration structure.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_set_configuration(LPUART_configuration_t* configuration);
#endif

/*******************************************************************/
#define LPUART_exit_error(base) { if (lpuart_status != LPUART_SUCCESS) { status = (base + lpuart_status); goto errors; } }

/*******************************************************************/
#define LPUART_stack_error(base) { if (lpuart_status != LPUART_SUCCESS) { ERROR_stack_add(base + lpuart_status); } }

/*******************************************************************/
#define LPUART_stack_exit_error(base, code) { if (lpuart_status != LPUART_SUCCESS) { ERROR_stack_add(base + lpuart_status); status = code; goto errors; } }

#endif /* __LPUART_H__ */
