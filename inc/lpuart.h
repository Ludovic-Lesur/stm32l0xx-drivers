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
    LPUART_ERROR_ALREADY_INITIALIZED,
    LPUART_ERROR_UNINITIALIZED,
    LPUART_ERROR_BAUD_RATE,
    LPUART_ERROR_TX_TIMEOUT,
    LPUART_ERROR_RS485_MODE,
    LPUART_ERROR_TC_TIMEOUT,
    // Last base value.
    LPUART_ERROR_BASE_LAST = 0x0100
} LPUART_status_t;

/*!******************************************************************
 * \struct LPUART_gpio_t
 * \brief LPUART GPIO pins list.
 *******************************************************************/
typedef struct {
    const GPIO_pin_t* tx;
    const GPIO_pin_t* rx;
#ifdef STM32L0XX_DRIVERS_LPUART_RS485
    const GPIO_pin_t* de;
    const GPIO_pin_t* nre;
#endif
} LPUART_gpio_t;

/*!******************************************************************
 * \fn LPUART_rx_irq_cb_t
 * \brief LPUART RX interrupt callback.
 *******************************************************************/
typedef void (*LPUART_rx_irq_cb_t)(uint8_t data);

#ifdef STM32L0XX_DRIVERS_LPUART_RS485
/*!******************************************************************
 * \enum LPUART_rs485_mode_t
 * \brief LPUART RS485 modes list.
 *******************************************************************/
typedef enum {
    LPUART_RS485_MODE_ADDRESSED = 0,
    LPUART_RS485_MODE_DIRECT,
    LPUART_RS485_MODE_LAST
} LPUART_rs485_mode_t;
#endif

/*!******************************************************************
 * \struct LPUART_configuration_t
 * \brief LPUART configuration structure.
 *******************************************************************/
typedef struct {
    uint32_t baud_rate;
    uint8_t nvic_priority;
    LPUART_rx_irq_cb_t rxne_callback;
#ifdef STM32L0XX_DRIVERS_LPUART_RS485
    LPUART_rs485_mode_t rs485_mode;
    uint8_t self_address;
#endif
} LPUART_configuration_t;

/*** LPUART functions ***/

/*!******************************************************************
 * \fn LPUART_status_t LPUART_init(const LPUART_gpio_t* pins, LPUART_configuration_t* configuration)
 * \brief Init LPUART peripheral.
 * \param[in]   pins: Pointer to the LPUART pins to use.
 * \param[in]   configuration: Pointer to the LPUART configuration structure.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_init(const LPUART_gpio_t* pins, LPUART_configuration_t* configuration);

/*!******************************************************************
 * \fn LPUART_status_t LPUART_de_init(const LPUART_gpio_t* pins)
 * \brief Release LPUART peripheral.
 * param[in]    pins: Pointer to the LPUART pins to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_de_init(const LPUART_gpio_t* pins);

/*!******************************************************************
 * \fn LPUART_status_t LPUART_enable_rx(void)
 * \brief Enable LPUART RX operation.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_enable_rx(void);

/*!******************************************************************
 * \fn LPUART_status_t LPUART_disable_rx(void)
 * \brief Disable LPUART RX operation.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_disable_rx(void);

/*!******************************************************************
 * \fn LPUART_status_t LPUART_write(uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over LPUART.
 * \param[in]   data: Byte array to send.
 * \param[in]   data_size_bytes: Number of bytes to send.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
LPUART_status_t LPUART_write(uint8_t* data, uint32_t data_size_bytes);

/*******************************************************************/
#define LPUART_exit_error(base) { ERROR_check_exit(lpuart_status, LPUART_SUCCESS, base) }

/*******************************************************************/
#define LPUART_stack_error(base) { ERROR_check_stack(lpuart_status, LPUART_SUCCESS, base) }

/*******************************************************************/
#define LPUART_stack_exit_error(base, code) { ERROR_check_stack_exit(lpuart_status, LPUART_SUCCESS, base, code) }

#endif /* __LPUART_H__ */
