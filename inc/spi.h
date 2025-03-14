/*
 * spi.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __SPI_H__
#define __SPI_H__

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#include "error.h"
#include "gpio.h"
#include "types.h"

/*** SPI structures ***/

/*!******************************************************************
 * \enum SPI_status_t
 * \brief SPI driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    SPI_SUCCESS = 0,
    SPI_ERROR_UNINITIALIZED,
    SPI_ERROR_NULL_PARAMETER,
    SPI_ERROR_INSTANCE,
    SPI_ERROR_BAUD_RATE_PRESCALER,
    SPI_ERROR_DATA_FORMAT,
    SPI_ERROR_CLOCK_POLARITY,
    SPI_ERROR_TX_BUFFER_EMPTY,
    SPI_ERROR_RX_TIMEOUT,
    // Last base value.
    SPI_ERROR_BASE_LAST = ERROR_BASE_STEP
} SPI_status_t;

/*!******************************************************************
 * \enum SPI_instance_t
 * \brief SPI instances list.
 *******************************************************************/
typedef enum {
    SPI_INSTANCE_SPI1 = 0,
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    SPI_INSTANCE_SPI2,
#endif
    SPI_INSTANCE_LAST
} SPI_instance_t;

/*!******************************************************************
 * \enum SPI_baud_rate_prescaler_t
 * \brief SPI baud rate prescalers list.
 *******************************************************************/
typedef enum {
    SPI_BAUD_RATE_PRESCALER_2 = 0,
    SPI_BAUD_RATE_PRESCALER_4,
    SPI_BAUD_RATE_PRESCALER_8,
    SPI_BAUD_RATE_PRESCALER_16,
    SPI_BAUD_RATE_PRESCALER_32,
    SPI_BAUD_RATE_PRESCALER_64,
    SPI_BAUD_RATE_PRESCALER_128,
    SPI_BAUD_RATE_PRESCALER_256,
    SPI_BAUD_RATE_PRESCALER_LAST
} SPI_baud_rate_prescaler_t;

/*!******************************************************************
 * \enum SPI_data_format_t
 * \brief SPI data formats list.
 *******************************************************************/
typedef enum {
    SPI_DATA_FORMAT_8_BITS = 0,
    SPI_DATA_FORMAT_16_BITS,
    SPI_DATA_FORMAT_LAST
} SPI_data_format_t;

/*!******************************************************************
 * \enum SPI_clock_polarity_t
 * \brief SPI clock polarities list.
 *******************************************************************/
typedef enum {
    SPI_CLOCK_POLARITY_LOW = 0,
    SPI_CLOCK_POLARITY_HIGH,
    SPI_CLOCK_POLARITY_LAST
} SPI_clock_polarity_t;

/*!******************************************************************
 * \struct SPI_gpio_t
 * \brief SPI GPIO pins list.
 *******************************************************************/
typedef struct {
    const GPIO_pin_t* sclk;
    const GPIO_pin_t* mosi;
    const GPIO_pin_t* miso;
} SPI_gpio_t;

/*!******************************************************************
 * \struct SPI_configuration_t
 * \brief SPI configuration structure.
 *******************************************************************/
typedef struct {
    SPI_baud_rate_prescaler_t baud_rate_prescaler;
    SPI_data_format_t data_format;
    SPI_clock_polarity_t clock_polarity;
} SPI_configuration_t;

/*** SPI functions ***/

/*!******************************************************************
 * \fn SPI_status_t SPI_init(SPI_instance_t instance, const SPI_gpio_t pins, SPI_configuration_t* configuration)
 * \brief Init SPI peripheral.
 * \param[in]   instance: Peripheral instance to initialize.
 * \param[in]   pins: Pointer to the SPI pins to use.
 * \param[in]   configuration: Pointer to the SPI configuration.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
SPI_status_t SPI_init(SPI_instance_t instance, const SPI_gpio_t* pins, SPI_configuration_t* configuration);

/*!******************************************************************
 * \fn SPI_status_t SPI_de_init(SPI_instance_t instance, const SPI_gpio_t* pins)
 * \brief Release SPI peripheral.
 * \param[in]   instance: Peripheral instance to release.
 * \param[in]   pins: Pointer to the SPI pins to release.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
SPI_status_t SPI_de_init(SPI_instance_t instance, const SPI_gpio_t* pins);

/*!******************************************************************
 * \fn SPI_status_t SPI_write_read_8(SPI_instance_t instance, uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size)
 * \brief SPI 8-bits data transfer function.
 * \param[in]   instance: Peripheral instance to use.
 * \param[in]   tx_data: Bytes array to send.
 * \param[in]   transfer_size: Number of bytes to send and receive.
 * \param[out]  rx_data: Pointer to the received bytes.
 * \retval      Function execution status.
 *******************************************************************/
SPI_status_t SPI_write_read_8(SPI_instance_t instance, uint8_t* tx_data, uint8_t* rx_data, uint8_t transfer_size);

/*!******************************************************************
 * \fn SPI_status_t SPI_write_read_16(SPI_instance_t instance, uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size)
 * \brief SPI 16-bits data transfer function.
 * \param[in]   instance: Peripheral instance to use.
 * \param[in]   tx_data: Shorts array to send.
 * \param[in]   transfer_size: Number of shorts to send and receive.
 * \param[out]  rx_data: Pointer to the received shorts.
 * \retval      Function execution status.
 *******************************************************************/
SPI_status_t SPI_write_read_16(SPI_instance_t instance, uint16_t* tx_data, uint16_t* rx_data, uint8_t transfer_size);

/*!******************************************************************
 * \fn void SPI_write_8(SPI_instance_t instance, uint8_t tx_data)
 * \brief Optimized SPI single byte transfer function.
 * \param[in]   instance: Peripheral instance to use.
 * \param[in]   tx_data: Short to send.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void SPI_write_8(SPI_instance_t instance, uint8_t tx_data);

/*!******************************************************************
 * \fn void SPI_write_16(SPI_instance_t instance, uint16_t tx_data)
 * \brief Optimized SPI single short transfer function.
 * \param[in]   instance: Peripheral instance to use.
 * \param[in]   tx_data: Short to send.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void SPI_write_16(SPI_instance_t instance, uint16_t tx_data);

/*******************************************************************/
#define SPI_exit_error(base) { ERROR_check_exit(spi_status, SPI_SUCCESS, base) }

/*******************************************************************/
#define SPI_stack_error(base) { ERROR_check_stack(spi_status, SPI_SUCCESS, base) }

/*******************************************************************/
#define SPI_stack_exit_error(base, code) { ERROR_check_stack_exit(spi_status, SPI_SUCCESS, base, code) }

#endif /* __SPI_H__ */
