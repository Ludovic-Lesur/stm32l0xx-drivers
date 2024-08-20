/*
 * dma.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __DMA_H__
#define __DMA_H__

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#include "types.h"

/*** DMA structures ***/

/*!******************************************************************
 * \enum DMA_status_t
 * \brief DMA driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	DMA_SUCCESS = 0,
	DMA_ERROR_NULL_PARAMETER,
	DMA_ERROR_CHANNEL,
	DMA_ERROR_DIRECTION,
	DMA_ERROR_TRANSFER_SIZE,
	DMA_ERROR_PRIORITY,
	DMA_ERROR_REQUEST_NUMBER,
	// Last base value.
	DMA_ERROR_BASE_LAST = 0x0100
} DMA_status_t;

/*!******************************************************************
 * \enum DMA_channel_t
 * \brief DMA channels list.
 *******************************************************************/
typedef enum {
	DMA_CHANNEL_1 = 0,
	DMA_CHANNEL_2,
	DMA_CHANNEL_3,
	DMA_CHANNEL_4,
	DMA_CHANNEL_5,
#if (STM32L0XX_REGISTERS_MCU_CATEGORY > 1)
	DMA_CHANNEL_6,
	DMA_CHANNEL_7,
#endif
	DMA_CHANNEL_LAST,
} DMA_channel_t;

/*!******************************************************************
 * \enum DMA_direction_t
 * \brief DMA transfer directions list.
 *******************************************************************/
typedef enum {
	DMA_DIRECTION_PERIPHERAL_TO_MEMORY = 0,
	DMA_DIRECTION_MEMORY_TO_PERIPHERAL,
	DMA_DIRECTION_LAST
} DMA_direction_t;

/*!******************************************************************
 * \union DMA_flags_t
 * \brief DMA configuration flags.
 *******************************************************************/
typedef union {
	struct {
		unsigned circular_mode : 1;
		unsigned peripheral_increment : 1;
		unsigned memory_increment : 1;
	};
	uint8_t all;
} DMA_flags_t;

/*!******************************************************************
 * \enum DMA_transfer_size_t
 * \brief DMA transfer sizes list.
 *******************************************************************/
typedef enum {
	DMA_TRANSFER_SIZE_8_BITS = 0,
	DMA_TRANSFER_SIZE_16_BITS,
	DMA_TRANSFER_SIZE_32_BITS,
	DMA_TRANSFER_SIZE_LAST
} DMA_transfer_size_t;

/*!******************************************************************
 * \enum DMA_priority_t
 * \brief DMA transfer priorities list.
 *******************************************************************/
typedef enum {
	DMA_PRIORITY_LOW = 0,
	DMA_PRIORITY_MEDIUM,
	DMA_PRIORITY_HIGH,
	DMA_PRIORITY_VERY_HIGH,
	DMA_PRIORITY_LAST
} DMA_priority_t;

/*!******************************************************************
 * \fn DMA_transfer_complete_irq_cb
 * \brief DMA transfer complete interrupt callback.
 *******************************************************************/
typedef void (*DMA_transfer_complete_irq_cb_t)(void);

/*!******************************************************************
 * \struct DMA_configuration_t
 * \brief DMA configuration structure.
 *******************************************************************/
typedef struct {
	DMA_direction_t direction;
	DMA_flags_t flags;
	DMA_transfer_size_t transfer_size;
	DMA_priority_t priority;
	uint16_t number_of_data;
	uint8_t request_number;
	uint32_t memory_address;
	uint32_t peripheral_address;
	DMA_transfer_complete_irq_cb_t irq_callback;
	uint8_t nvic_priority;
} DMA_configuration_t;

/*** DMA functions ***/

#if (STM32L0XX_DRIVERS_DMA_CHANNEL_MASK != 0)
/*!******************************************************************
 * \fn DMA_status_t DMA_init(DMA_channel_t_t channel, DMA_configuration_t* configuration)
 * \brief Init DMA channel.
 * \param[in]:	channel: DMA channel to initialize.
 * \param[in]:	configuration: Pointer to the DMA channel configuration structure.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DMA_status_t DMA_init(DMA_channel_t channel, DMA_configuration_t* configuration);
#endif

#if (STM32L0XX_DRIVERS_DMA_CHANNEL_MASK != 0)
/*!******************************************************************
 * \fn DMA_status_t DMA_de_init(DMA_channel_t_t channel)
 * \brief Release a DMA channel.
 * \param[in]:	channel: DMA channel to release.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DMA_status_t DMA_de_init(DMA_channel_t channel);
#endif

#if (STM32L0XX_DRIVERS_DMA_CHANNEL_MASK != 0)
/*!******************************************************************
 * \fn DMA_status_t DMA_start(DMA_channel_t_t channel)
 * \brief Start a DMA channel.
 * \param[in]:	channel: DMA channel to start.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DMA_status_t DMA_start(DMA_channel_t channel);
#endif

#if (STM32L0XX_DRIVERS_DMA_CHANNEL_MASK != 0)
/*!******************************************************************
 * \fn DMA_status_t DMA_stop(DMA_channel_t_t channel)
 * \brief Stop a DMA channel.
 * \param[in]:	channel: DMA channel to stop.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DMA_status_t DMA_stop(DMA_channel_t channel);
#endif

#if (STM32L0XX_DRIVERS_DMA_CHANNEL_MASK != 0)
/*!******************************************************************
 * \fn DMA_status_t DMA_set_memory_address(DMA_channel_t_t channel, uint32_t memory_addr, uint16_t number_of_data)
 * \brief Set DMA channel memory address.
 * \param[in]:	channel: DMA channel to configure.
 * \param[in] 	memory_addr: Memory address.
 * \param[in]	number_of_data: DMA transfer size.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DMA_status_t DMA_set_memory_address(DMA_channel_t channel, uint32_t memory_addr, uint16_t number_of_data);
#endif

#if (STM32L0XX_DRIVERS_DMA_CHANNEL_MASK != 0)
/*!******************************************************************
 * \fn DMA_status_t DMA_set_peripheral_address(DMA_channel_t_t channel, uint32_t peripheral_addr, uint16_t number_of_data)
 * \brief Set DMA channel peripheral address.
 * \param[in]:	channel: DMA channel to configure.
 * \param[in] 	peripheral_addr: Peripheral address.
 * \param[in]	number_of_data: DMA transfer size.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
DMA_status_t DMA_set_peripheral_address(DMA_channel_t channel, uint32_t peripheral_addr, uint16_t number_of_data);
#endif

/*******************************************************************/
#define DMA_exit_error(base) { if (dma_status != DMA_SUCCESS) { status = (base + dma_status); goto errors; } }

/*******************************************************************/
#define DMA_stack_error(base) { if (dma_status != DMA_SUCCESS) { ERROR_stack_add(base + dma_status); } }

/*******************************************************************/
#define DMA_stack_exit_error(base, code) { if (dma_status != DMA_SUCCESS) { ERROR_stack_add(base + dma_status); status = code; goto errors; } }

#endif /* __DMA_H__ */
