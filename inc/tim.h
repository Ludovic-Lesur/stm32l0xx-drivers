/*
 * tim.h
 *
 *  Created on: 04 aug. 2024
 *      Author: Ludo
 */

#ifndef __TIM_H__
#define __TIM_H__

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "gpio.h"
#include "rcc.h"
#include "types.h"

/*** TIM structures ***/

/*!******************************************************************
 * \enum TIM_status_t
 * \brief TIM CAL driver error codes.
 *******************************************************************/
typedef enum {
	// Driver errors.
	TIM_SUCCESS = 0,
	TIM_ERROR_NULL_PARAMETER,
	TIM_ERROR_INSTANCE,
	TIM_ERROR_MODE,
	TIM_ERROR_ARR_VALUE,
	TIM_ERROR_WAITING_MODE,
	TIM_ERROR_CHANNEL,
	TIM_ERROR_DURATION_UNDERFLOW,
	TIM_ERROR_DURATION_OVERFLOW,
	TIM_ERROR_COMPLETION_WATCHDOG,
	TIM_ERROR_CAPTURE_TIMEOUT,
	TIM_ERROR_DUTY_CYCLE,
	// Low level drivers errors.
	TIM_ERROR_BASE_RCC = 0x0100,
	// Last base value.
	TIM_ERROR_BASE_LAST = (TIM_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST)
} TIM_status_t;

/*!******************************************************************
 * \enum TIM_instance_t
 * \brief TIM instances list.
 *******************************************************************/
typedef enum {
	TIM_INSTANCE_TIM2 = 0,
	TIM_INSTANCE_TIM21,
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 2) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	TIM_INSTANCE_TIM22,
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	TIM_INSTANCE_TIM6,
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	TIM_INSTANCE_TIM3,
	TIM_INSTANCE_TIM7,
#endif
	TIM_INSTANCE_LAST
} TIM_instance_t;

/*!******************************************************************
 * \enum TIM_channel_t
 * \brief Timer channels list.
 *******************************************************************/
typedef enum {
	TIM_CHANNEL_1 = 0,
	TIM_CHANNEL_2,
	TIM_CHANNEL_3,
	TIM_CHANNEL_4,
	TIM_CHANNEL_LAST
} TIM_channel_t;

/*!******************************************************************
 * \enum TIM_waiting_mode_t
 * \brief Timer completion waiting modes.
 *******************************************************************/
typedef enum {
	TIM_WAITING_MODE_ACTIVE = 0,
	TIM_WAITING_MODE_SLEEP,
	TIM_WAITING_MODE_LOW_POWER_SLEEP,
	TIM_WAITING_MODE_LAST
} TIM_waiting_mode_t;

/*!******************************************************************
 * \fn TIM_completion_irq_cb_t
 * \brief TIM completion callback.
 *******************************************************************/
typedef void (*TIM_completion_irq_cb_t)(void);

/*!******************************************************************
 * \struct TIM_gpio_t
 * \brief TIM GPIO pins list.
 *******************************************************************/
typedef struct {
	const GPIO_pin_t* ch1;
	const GPIO_pin_t* ch2;
	const GPIO_pin_t* ch3;
	const GPIO_pin_t* ch4;
} TIM_gpio_t;

/*** TIM functions ***/

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_STD_init(TIM_instance_t instance, uint32_t period_ns, uint8_t nvic_priority, TIM_completion_irq_cb_t irq_callback)
 * \brief Init a timer peripheral in standard mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]	period_ns: Timer period in ns.
 * \param[in]	nvic_priority: Interrupt priority.
 * \param[in]	irq_callback: Function to call on timer completion interrupt.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_STD_init(TIM_instance_t instance, uint32_t period_ns, uint8_t nvic_priority, TIM_completion_irq_cb_t irq_callback);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_STD_de_init(TIM_instance_t instance)
 * \brief Release a timer peripheral.
 * \param[in]  	instance: Timer instance to release.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_STD_de_init(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_STD_start(TIM_instance_t instance)
 * \brief Start a timer in standard mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
TIM_status_t TIM_STD_start(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_STD_stop(TIM_instance_t instance)
 * \brief Stop a timer in standard mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
TIM_status_t TIM_STD_stop(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_MCH_init(TIM_instance_t instance, uint8_t nvic_priority)
 * \brief Init a timer peripheral in multi-channel mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]	nvic_priority: Interrupt priority.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_init(TIM_instance_t instance, uint8_t nvic_priority);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_MCH_de_init(TIM_instance_t instance)
 * \brief Release a timer peripheral.
 * \param[in]  	instance: Timer instance to release.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_de_init(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_MCH_start_channel(TIM_instance_t instance, TIM_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode)
 * \brief Start a timer in multi-channel mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]  	channel: Channel to start.
 * \param[in]	duration_ms: Timer duration in ms.
 * \param[in]	waiting_mode: Completion waiting mode.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
TIM_status_t TIM_MCH_start_channel(TIM_instance_t instance, TIM_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_stop_channel(TIM_instance_t instance, TIM_channel_t channel)
 * \brief Stop a timer in multi-channel mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]  	channel: Channel to stop.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_stop_channel(TIM_instance_t instance, TIM_channel_t channel);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_get_channel_status(TIM_instance_t instance, TIM_channel_t channel, uint8_t* timer_has_elapsed)
 * \brief Get the status of a timer channel.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]  	channel: Channel to read.
 * \param[out]	timer_has_elapsed: Pointer to bit that will contain the timer status (0 for running, 1 for complete).
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_get_channel_status(TIM_instance_t instance, TIM_channel_t channel, uint8_t* timer_has_elapsed);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_wait_channel_completion(TIM_instance_t instance, TIM_channel_t channel)
 * \brief Blocking function waiting for a timer channel completion.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]  	channel: Channel to wait for.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_wait_channel_completion(TIM_instance_t instance, TIM_channel_t channel);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_CAL_init(TIM_instance_t instance, uint8_t nvic_priority)
 * \brief Init a timer peripheral in calibration mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]	nvic_priority: Interrupt priority.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_CAL_init(TIM_instance_t instance, uint8_t nvic_priority);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_CAL_de_init(TIM_instance_t instance)
 * \brief Release a timer peripheral.
 * \param[in]  	instance: Timer instance to release.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_CAL_de_init(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_mco_capture(TIM_instance_t instance, uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count)
 * \brief Perform MCO clock capture.
 * \param[in]  	instance: Timer instance to use.
 * \param[out] 	ref_clock_pulse_count: Pointer to the number of pulses of the timer reference clock during the capture.
 * \param[out]	mco_pulse_count: Pointer to the number of pulses of the MCO clock during the capture.
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_CAL_mco_capture(TIM_instance_t instance, uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_PWM_init(TIM_instance_t instance, TIM_gpio_t* pins, uint32_t pwm_frequency_hz)
 * \brief Init a timer peripheral in PWM mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]	pins: Pointer to the timer pins to use.
 * \param[in]	pwm_frequency_hz: PWM signal frequency in Hz.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_PWM_init(TIM_instance_t instance, TIM_gpio_t* pins, uint32_t pwm_frequency_hz);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_de_init(TIM_instance_t instance, TIM_gpio_t* pins)
 * \brief Release a timer peripheral.
 * \param[in]  	instance: Timer instance to release.
 * \param[in]	pins: Pointer to the timer pins to release.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_PWM_de_init(TIM_instance_t instance, TIM_gpio_t* pins);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_PWM_set_duty_cycle(TIM_instance_t instance, TIM_channel_t channel, uint8_t duty_cycle_percent)
 * \brief Set channel duty cycle of a timer configured in PWM mode.
 * \param[in]  	instance: Timer instance to use.
 * \param[in]  	channel: Channel to configure.
 * \param[in]  	duty_cycle_percent: PWM duty cycle in percent.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
TIM_status_t TIM_PWM_set_duty_cycle(TIM_instance_t instance, TIM_channel_t channel, uint8_t duty_cycle_percent);
#endif

/*******************************************************************/
#define TIM_exit_error(base) { ERROR_check_exit(tim_status, TIM_SUCCESS, base) }

/*******************************************************************/
#define TIM_stack_error(base) { ERROR_check_stack(tim_status, TIM_SUCCESS, base) }

/*******************************************************************/
#define TIM_stack_exit_error(base, code) { ERROR_check_stack_exit(tim_status, TIM_SUCCESS, base, code) }

#endif /* __TIM_H__ */
