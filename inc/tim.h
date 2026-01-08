/*
 * tim.h
 *
 *  Created on: 04 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#ifndef __TIM_H__
#define __TIM_H__

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#include "error.h"
#include "gpio.h"
#include "maths.h"
#include "rcc.h"
#include "types.h"

/*** TIM macros ***/

#define TIM_MODE_MASK_STANDARD      0x01
#define TIM_MODE_MASK_MULTI_CHANNEL 0x02
#define TIM_MODE_MASK_CALIBRATION   0x04
#define TIM_MODE_MASK_PWM           0x08
#define TIM_MODE_MASK_OPM           0x10

#define TIM_MODE_MASK_ALL           0x1F

/*** TIM structures ***/

/*!******************************************************************
 * \enum TIM_status_t
 * \brief Timer driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    TIM_SUCCESS = 0,
    TIM_ERROR_NULL_PARAMETER,
    TIM_ERROR_INSTANCE,
    TIM_ERROR_INSTANCE_NOT_SUPPORTED,
    TIM_ERROR_MODE,
    TIM_ERROR_UNIT,
    TIM_ERROR_ARR_VALUE,
    TIM_ERROR_WAITING_MODE,
    TIM_ERROR_CHANNEL,
    TIM_ERROR_CHANNEL_NOT_SUPPORTED,
    TIM_ERROR_NUMBER_OF_PINS,
    TIM_ERROR_DURATION_UNDERFLOW,
    TIM_ERROR_DURATION_OVERFLOW,
    TIM_ERROR_COMPLETION_WATCHDOG,
    TIM_ERROR_CAPTURE_TIMEOUT,
    TIM_ERROR_FREQUENCY,
    TIM_ERROR_DUTY_CYCLE,
    TIM_ERROR_PULSE,
    // Low level drivers errors.
    TIM_ERROR_BASE_RCC = ERROR_BASE_STEP,
    TIM_ERROR_BASE_MATH = (TIM_ERROR_BASE_RCC + RCC_ERROR_BASE_LAST),
    // Last base value.
    TIM_ERROR_BASE_LAST = (TIM_ERROR_BASE_MATH + MATH_ERROR_BASE_LAST)
} TIM_status_t;

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_ALL) != 0)

/*!******************************************************************
 * \enum TIM_instance_t
 * \brief Timer instances list.
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
 * \enum TIM_polarity_t
 * \brief Timer channel output polarities list.
 *******************************************************************/
typedef enum {
    TIM_POLARITY_ACTIVE_HIGH = 0,
    TIM_POLARITY_ACTIVE_LOW,
    TIM_POLARITY_LAST
} TIM_polarity_t;

/*!******************************************************************
 * \enum TIM_unit_t
 * \brief Timer unit list
 *******************************************************************/
typedef enum {
    TIM_UNIT_NS = 0,
    TIM_UNIT_US,
    TIM_UNIT_MS,
    TIM_UNIT_LAST
} TIM_unit_t;

/*!******************************************************************
 * \struct TIM_channel_gpio_t
 * \brief Timer channel GPIO settings structure.
 *******************************************************************/
typedef struct {
    TIM_channel_t channel;
    const GPIO_pin_t* gpio;
    TIM_polarity_t polarity;
} TIM_channel_gpio_t;

/*!******************************************************************
 * \struct TIM_gpio_t
 * \brief Timer GPIOs list.
 *******************************************************************/
typedef struct {
    const TIM_channel_gpio_t** list;
    uint8_t list_size;
} TIM_gpio_t;

/*!******************************************************************
 * \fn TIM_completion_irq_cb_t
 * \brief TIM completion callback.
 *******************************************************************/
typedef void (*TIM_completion_irq_cb_t)(void);

/*** TIM functions ***/

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_STD_init(TIM_instance_t instance, uint8_t nvic_priority)
 * \brief Init a timer peripheral in standard mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   nvic_priority: Interrupt priority.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_STD_init(TIM_instance_t instance, uint8_t nvic_priority);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_STD_de_init(TIM_instance_t instance)
 * \brief Release a timer peripheral.
 * \param[in]   instance: Timer instance to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_STD_de_init(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_STD_start(TIM_instance_t instance, uint32_t period_value, TIM_unit_t period_unit, TIM_completion_irq_cb_t irq_callback)
 * \brief Start a timer in standard mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   period_value: Timer period in the given unit.
 * \param[in]   period_unit: Unit of the period value.
 * \param[in]   irq_callback: Function to call on timer completion interrupt.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
TIM_status_t TIM_STD_start(TIM_instance_t instance, uint32_t period_value, TIM_unit_t period_unit, TIM_completion_irq_cb_t irq_callback);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_STD_stop(TIM_instance_t instance)
 * \brief Stop a timer in standard mode.
 * \param[in]   instance: Timer instance to use.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
TIM_status_t TIM_STD_stop(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_MCH_init(TIM_instance_t instance, uint8_t nvic_priority)
 * \brief Init a timer peripheral in multi-channel mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   nvic_priority: Interrupt priority.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_init(TIM_instance_t instance, uint8_t nvic_priority);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_MCH_de_init(TIM_instance_t instance)
 * \brief Release a timer peripheral.
 * \param[in]   instance: Timer instance to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_de_init(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_MCH_start_channel(TIM_instance_t instance, TIM_channel_t channel, uint32_t period_ms, TIM_waiting_mode_t waiting_mode)
 * \brief Start a timer in multi-channel mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   channel: Channel to start.
 * \param[in]   period_ms: Timer channel period in ms.
 * \param[in]   waiting_mode: Completion waiting mode.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_start_channel(TIM_instance_t instance, TIM_channel_t channel, uint32_t period_ms, TIM_waiting_mode_t waiting_mode);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_stop_channel(TIM_instance_t instance, TIM_channel_t channel)
 * \brief Stop a timer in multi-channel mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   channel: Channel to stop.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_stop_channel(TIM_instance_t instance, TIM_channel_t channel);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_get_channel_status(TIM_instance_t instance, TIM_channel_t channel, uint8_t* timer_has_elapsed)
 * \brief Get the status of a timer channel.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   channel: Channel to read.
 * \param[out]  timer_has_elapsed: Pointer to bit that will contain the timer status (0 for running, 1 for complete).
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_get_channel_status(TIM_instance_t instance, TIM_channel_t channel, uint8_t* timer_has_elapsed);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_wait_channel_completion(TIM_instance_t instance, TIM_channel_t channel)
 * \brief Blocking function waiting for a timer channel completion.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   channel: Channel to wait for.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_MCH_wait_channel_completion(TIM_instance_t instance, TIM_channel_t channel);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_CAL_init(TIM_instance_t instance, uint8_t nvic_priority)
 * \brief Init a timer peripheral in calibration mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   nvic_priority: Interrupt priority.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_CAL_init(TIM_instance_t instance, uint8_t nvic_priority);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_CAL_de_init(TIM_instance_t instance)
 * \brief Release a timer peripheral.
 * \param[in]   instance: Timer instance to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_CAL_de_init(TIM_instance_t instance);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_mco_capture(TIM_instance_t instance, int32_t* ref_clock_pulse_count, int32_t* mco_pulse_count)
 * \brief Perform MCO clock capture.
 * \param[in]   instance: Timer instance to use.
 * \param[out]  ref_clock_pulse_count: Pointer to the number of pulses of the timer reference clock during the capture.
 * \param[out]  mco_pulse_count: Pointer to the number of pulses of the MCO clock during the capture.
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_CAL_mco_capture(TIM_instance_t instance, int32_t* ref_clock_pulse_count, int32_t* mco_pulse_count);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_PWM) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_PWM_init(TIM_instance_t instance, TIM_gpio_t* pins)
 * \brief Init a timer peripheral in PWM mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   pins: List of timer pins to configure.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_PWM_init(TIM_instance_t instance, TIM_gpio_t* pins);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_PWM) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_PWM_de_init(TIM_instance_t instance, TIM_gpio_t* pins)
 * \brief Release a timer peripheral.
 * \param[in]   instance: Timer instance to release.
 * \param[in]   pins: List of timer pins to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_PWM_de_init(TIM_instance_t instance, TIM_gpio_t* pins);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_PWM) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_PWM_set_waveform(TIM_instance_t instance, TIM_channel_t channel, uint32_t frequency_mhz, uint8_t duty_cycle_percent)
 * \brief Set channel duty cycle of a timer configured in PWM mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   channel: Channel to configure.
 * \param[in]   frequency_mhz: PWM frequency in mHz. Warning: this setting will be applied to all channels of the timer instance.
 * \param[in]   duty_cycle_percent: PWM duty cycle in percent. Value 0 disables the signal.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_PWM_set_waveform(TIM_instance_t instance, TIM_channel_t channel, uint32_t frequency_mhz, uint8_t duty_cycle_percent);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_OPM_init(TIM_instance_t instance, TIM_gpio_t* pins)
 * \brief Init a timer peripheral in one pulse mode.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   pins_list: List of timer pins to configure.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_OPM_init(TIM_instance_t instance, TIM_gpio_t* pins);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_OPM_de_init(TIM_instance_t instance, TIM_gpio_t* pins)
 * \brief Release a timer peripheral.
 * \param[in]   instance: Timer instance to release.
 * \param[in]   pins: List of timer pins to release.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_OPM_de_init(TIM_instance_t instance, TIM_gpio_t* pins);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_OPM_make_pulse(TIM_instance_t instance, uint8_t channels_mask, uint32_t delay_ns, uint32_t pulse_duration_ns)
 * \brief Perform a single output pulse.
 * \param[in]   instance: Timer instance to use.
 * \param[in]   channels_mask: Channels to use.
 * \param[in]   delay_ns: Delay between function call and pulse start in ns. Warning: this setting will be applied to all channels of the timer instance.
 * \param[in]   pulse_duration_ns: Pulse duration in ns. Warning: this setting will be applied to all channels of the timer instance.
 * \param[in]   internal_irq_enable: Enable or disable internal pulse completion interrupt (optionally used to wake-up the core at the end of the pulse).
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_OPM_make_pulse(TIM_instance_t instance, uint8_t channels_mask, uint32_t delay_ns, uint32_t pulse_duration_ns, uint8_t internal_irq_enable);
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*!******************************************************************
 * \fn TIM_status_t TIM_OPM_get_pulse_status(TIM_instance_t instance, uint8_t* pulse_is_done)
 * \brief Get single output pulse status
 * \param[in]   instance: Timer instance to use.
 * \param[out]  pulse_is_done: Pointer to byte that will contain the pulse status.
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_OPM_get_pulse_status(TIM_instance_t instance, uint8_t* pulse_is_done);
#endif

/*!******************************************************************
 * \fn TIM_status_t TIM_get_ccr_register_address(TIM_instance_t instance, TIM_channel_t channel, uint32_t* ccr_register_address)
 * \brief Get TIM CCRx register address.
 * \param[in]   instance: Timer instance to read.
 * \param[in]   channel: Channel to read.
 * \param[out]  ccr_register_address: Pointer to integer that will contain the TIM CCR register address.
 * \retval      Function execution status.
 *******************************************************************/
TIM_status_t TIM_get_ccr_register_address(TIM_instance_t instance, TIM_channel_t channel, uint32_t* ccr_register_address);

/*******************************************************************/
#define TIM_exit_error(base) { ERROR_check_exit(tim_status, TIM_SUCCESS, base) }

/*******************************************************************/
#define TIM_stack_error(base) { ERROR_check_stack(tim_status, TIM_SUCCESS, base) }

/*******************************************************************/
#define TIM_stack_exit_error(base, code) { ERROR_check_stack_exit(tim_status, TIM_SUCCESS, base, code) }

#endif /* STM32L0XX_DRIVERS_TIM_MODE_MASK */

#endif /* __TIM_H__ */

#endif /* STM32L0XX_DRIVERS_DISABLE */
