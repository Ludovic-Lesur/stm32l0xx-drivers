/*
 * i2c.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __I2C_H__
#define __I2C_H__

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#include "gpio.h"
#include "lptim.h"
#include "types.h"

/*** I2C structures ***/

/*!******************************************************************
 * \enum I2C_status_t
 * \brief I2C driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	I2C_SUCCESS = 0,
	I2C_ERROR_UNINITIALIZED,
	I2C_ERROR_NULL_PARAMETER,
	I2C_ERROR_INSTANCE,
	I2C_ERROR_TIMEOUT,
	I2C_ERROR_BUSY,
	I2C_ERROR_START_BIT_CLEAR,
	I2C_ERROR_TX_BUFFER_EMPTY,
	I2C_ERROR_RX_TIMEOUT,
	I2C_ERROR_TRANSFER_COMPLETE,
	I2C_ERROR_STOP_DETECTION_FLAG,
	// Low level drivers errors.
	I2C_ERROR_BASE_LPTIM = 0x0100,
	// Last base value.
	I2C_ERROR_BASE_LAST = (I2C_ERROR_BASE_LPTIM + LPTIM_ERROR_BASE_LAST)
} I2C_status_t;

/*!******************************************************************
 * \enum I2C_instance_t
 * \brief I2C instances list.
 *******************************************************************/
typedef enum {
	I2C_INSTANCE_I2C1 = 0,
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	I2C_INSTANCE_I2C2,
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	I2C_INSTANCE_I2C3,
#endif
	I2C_INSTANCE_LAST
} I2C_instance_t;

/*!******************************************************************
 * \enum I2C_gpio_t
 * \brief I2C GPIO pins list.
 *******************************************************************/
typedef struct {
	const GPIO_pin_t* scl;
	const GPIO_pin_t* sda;
} I2C_gpio_t;

/*** I2C functions ***/

/*!******************************************************************
 * \fn I2C_status_t II2C_init(I2C_instance_t instance, const I2C_gpio_t* pins)
 * \brief Init I2C peripheral.
 * \param[in]  	instance: Peripheral instance to initialize.
 * \param[in]	pins: Pointer to the I2C pins to use.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
I2C_status_t I2C_init(I2C_instance_t instance, const I2C_gpio_t* pins);

/*!******************************************************************
 * \fn I2C_status_t I2C_de_init(I2C_instance_t instance, const I2C_gpio_t* pins)
 * \brief Release I2C peripheral.
 * \param[in]  	instance: Peripheral instance to release.
 * \param[in]	pins: Pointer to the I2C pins to release.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
I2C_status_t I2C_de_init(I2C_instance_t instance, const I2C_gpio_t* pins);

/*!******************************************************************
 * \fn I2C_status_t I2C_write(I2C_instance_t instance, uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag)
 * \brief Write data on I2C bus.
 * \param[in]  	instance: Peripheral instance to use.
 * \param[in]  	slave_address: 7-bits destination slave address.
 * \param[in]	data: Byte array to send.
 * \param[in]	data_size_bytes: Number of bytes to send.
 * \param[in]	stop_flag: Generate stop condition at the end of the transfer if non zero.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
I2C_status_t I2C_write(I2C_instance_t instance, uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes, uint8_t stop_flag);

/*!******************************************************************
 * \fn I2C_status_t I2C_read(I2C_instance_t instance, uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes)
 * \brief Read data on I2C bus.
 * \param[in]  	instance: Peripheral instance to use.
 * \param[in]  	slave_address: 7-bits destination slave address.
 * \param[in]	data_size_bytes: Number of bytes to read.
 * \param[out]	data: Byte array that will contain the read data.
 * \retval		Function execution status.
 *******************************************************************/
I2C_status_t I2C_read(I2C_instance_t instance, uint8_t slave_address, uint8_t* data, uint8_t data_size_bytes);

/*******************************************************************/
#define I2C_exit_error(base) { ERROR_check_exit(i2c_status, I2C_SUCCESS, base) }

/*******************************************************************/
#define I2C_stack_error(base) { ERROR_check_stack(i2c_status, I2C_SUCCESS, base) }

/*******************************************************************/
#define I2C_stack_exit_error(base, code) { ERROR_check_stack_exit(i2c_status, I2C_SUCCESS, base, code) }

#endif /* __I2C_H__ */
