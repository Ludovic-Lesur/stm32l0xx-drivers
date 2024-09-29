/*
 * usart.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __USART_H__
#define __USART_H__

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "gpio.h"
#include "types.h"

/*** USART structures ***/

/*!******************************************************************
 * \enum USART_status_t
 * \brief USART driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	USART_SUCCESS = 0,
	USART_ERROR_UNINITIALIZED,
	USART_ERROR_NULL_PARAMETER,
	USART_ERROR_INSTANCE,
	USART_ERROR_TX_TIMEOUT,
	// Last base value.
	USART_ERROR_BASE_LAST = 0x0100
} USART_status_t;

/*!******************************************************************
 * \enum USART_instance_t
 * \brief USART instances list.
 *******************************************************************/
typedef enum {
	USART_INSTANCE_USART2 = 0,
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	USART_INSTANCE_USART1,
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	USART_INSTANCE_USART4,
	USART_INSTANCE_USART5,
#endif
	USART_INSTANCE_LAST
} USART_instance_t;

/*!******************************************************************
 * \struct USART_gpio_t
 * \brief USART GPIO pins list.
 *******************************************************************/
typedef struct {
	const GPIO_pin_t* tx;
	const GPIO_pin_t* rx;
} USART_gpio_t;

#if (STM32L0XX_DRIVERS_USART_MODE == 0)
/*!******************************************************************
 * \fn USART_rx_irq_cb_t
 * \brief USART RX interrupt callback.
 *******************************************************************/
typedef void (*USART_rx_irq_cb_t)(uint8_t data);
#endif

#if (STM32L0XX_DRIVERS_USART_MODE == 1)
/*!******************************************************************
 * \fn USART_character_match_irq_cb_t
 * \brief USART character match interrupt callback.
 *******************************************************************/
typedef void (*USART_character_match_irq_cb_t)(void);
#endif

/*!******************************************************************
 * \struct USART_configuration_t
 * \brief USART configuration structure.
 *******************************************************************/
typedef struct {
	uint32_t baud_rate;
	uint8_t nvic_priority;
#if (STM32L0XX_DRIVERS_USART_MODE == 0)
	USART_rx_irq_cb_t rxne_callback;
#endif
#if (STM32L0XX_DRIVERS_USART_MODE == 1)
	char_t match_character;
	USART_character_match_irq_cb_t cmf_callback;
#endif
} USART_configuration_t;

/*** USART functions ***/

/*!******************************************************************
 * \fn USART_status_t USART_init(USART_instance_t instance, const USART_gpio_t* pins, USART_configuration_t* configuration)
 * \brief Init USART peripheral.
 * \param[in]	instance: Peripheral instance to initialize.
 * \param[in]	pins: Pointer to the USART pins to use.
 * \param[in]  	configuration: Pointer to the USART configuration structure.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART_init(USART_instance_t instance, const USART_gpio_t* pins, USART_configuration_t* configuration);

/*!******************************************************************
 * \fn USART_status_t USART_de_init(USART_instance_t instance, const USART_gpio_t* pins)
 * \brief Release USART peripheral.
 * \param[in]	instance: Peripheral instance to release.
 * \param[in]	pins: Pointer to the USART pins to release.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART_de_init(USART_instance_t instance, const USART_gpio_t* pins);

/*!******************************************************************
 * \fn USART_status_t USART_enable_rx(USART_instance_t instance)
 * \brief Enable USART RX operation.
 * \param[in]	instance: Peripheral instance to control.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART_enable_rx(USART_instance_t instance);

/*!******************************************************************
 * \fn USART_status_t USART_disable_rx(USART_instance_t instance)
 * \brief Disable USART RX operation.
 * \param[in]	instance: Peripheral instance to control.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART_disable_rx(USART_instance_t instance);

/*!******************************************************************
 * \fn USART_status_t USART_write(USART_instance_t instance, uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over USART.
 * \param[in]	instance: Peripheral instance to use.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
USART_status_t USART_write(USART_instance_t instance, uint8_t* data, uint32_t data_size_bytes);

/*******************************************************************/
#define USART_exit_error(base) { ERROR_check_exit(usart_status, USART_SUCCESS, base) }

/*******************************************************************/
#define USART_stack_error(base) { ERROR_check_stack(usart_status, USART_SUCCESS, base) }

/*******************************************************************/
#define USART_stack_exit_error(base, code) { ERROR_check_stack_exit(usart_status, USART_SUCCESS, base, code) }

#endif /* __USART_H__ */
