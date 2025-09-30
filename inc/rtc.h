/*
 * rtc.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#ifndef __RTC_H__
#define __RTC_H__

#include "error.h"
#include "nvic.h"
#include "rcc.h"
#include "types.h"

/*** RTC macros ***/

#define RTC_LOCAL_UTC_OFFSET_WINTER     1
#define RTC_LOCAL_UTC_OFFSET_SUMMER     2

#define RTC_WINTER_TIME_LAST_MONTH      3
#define RTC_WINTER_TIME_FIRST_MONTH     11

#define RTC_NUMBER_OF_HOURS_PER_DAY     24

#define RTC_AFTERNOON_HOUR_THRESHOLD    12

#define RTC_ALARM_MASK_A                0x01
#define RTC_ALARM_MASK_B                0x02
#define RTC_ALARM_MASK_ALL              0x03

/*** RTC structures ***/

/*!******************************************************************
 * \enum RTC_status_t
 * \brief RTC driver error codes.
 *******************************************************************/
typedef enum {
    // Driver errors.
    RTC_SUCCESS = 0,
    RTC_ERROR_NULL_PARAMETER,
    RTC_ERROR_INITIALIZATION_MODE,
    RTC_ERROR_WAKEUP_TIMER_REGISTER_ACCESS,
    RTC_ERROR_ALARM,
    RTC_ERROR_ALARM_MODE,
    // Last base value.
    RTC_ERROR_BASE_LAST = ERROR_BASE_STEP
} RTC_status_t;

#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & RTC_ALARM_MASK_ALL) != 0)
/*!******************************************************************
 * \enum RTC_alarm_t
 * \brief RTC alarms list.
 *******************************************************************/
typedef enum {
    RTC_ALARM_A = 0,
    RTC_ALARM_B,
    RTC_ALARM_LAST
} RTC_alarm_t;
#endif

#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & RTC_ALARM_MASK_ALL) != 0)
/*!******************************************************************
 * \enum RTC_alarm_mode_t
 * \brief RTC alarm modes list.
 *******************************************************************/
typedef enum {
    RTC_ALARM_MODE_DATE = 0,
    RTC_ALARM_MODE_WEEK_DAY,
    RTC_ALARM_MODE_LAST
} RTC_alarm_mode_t;
#endif

#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & RTC_ALARM_MASK_ALL) != 0)
/*!******************************************************************
 * \union RTC_alarm_field_t
 * \brief RTC alarm field type.
 *******************************************************************/
typedef union {
    uint8_t all;
    struct {
        unsigned mask :1;
        unsigned value :7;
    } __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed));
} RTC_alarm_field_t;
#endif

#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & RTC_ALARM_MASK_ALL) != 0)
/*!******************************************************************
 * \struct RTC_alarm_configuration_t
 * \brief RTC alarm configuration structure.
 *******************************************************************/
typedef struct {
    RTC_alarm_mode_t mode;
    RTC_alarm_field_t date;
    RTC_alarm_field_t hours;
    RTC_alarm_field_t minutes;
    RTC_alarm_field_t seconds;
} RTC_alarm_configuration_t;
#endif

/*!******************************************************************
 * \struct RTC_time_t
 * \brief RTC time structure.
 *******************************************************************/
typedef struct {
    // Date.
    uint16_t year;
    uint8_t month;
    uint8_t date;
    // Time.
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} RTC_time_t;

/*!******************************************************************
 * \fn RTC_irq_cb_t
 * \brief RTC interrupt callback.
 *******************************************************************/
typedef void (*RTC_irq_cb_t)(void);

/*** RTC functions ***/

/*!******************************************************************
 * \fn RTC_status_t RTC_init(RTC_irq_cb_t wakeup_timer_irq_callback, uint8_t nvic_priority)
 * \brief Init RTC peripheral.
 * \param[in]   wakeup_timer_irq_callback: Wakeup timer interrupt callback (can be NULL).
 * \param[in]   nvic_priority: Interrupt priority.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RTC_status_t RTC_init(RTC_irq_cb_t wakeup_timer_irq_callback, uint8_t nvic_priority);

/*!******************************************************************
 * \fn uint32_t RTC_get_uptime_seconds(void)
 * \brief Read MCU operating time in seconds.
 * \param[in]   none
 * \param[out]  none
 * \retval      Current time in seconds.
 *******************************************************************/
uint32_t RTC_get_uptime_seconds(void);

#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & RTC_ALARM_MASK_ALL) != 0)
/*!******************************************************************
 * \fn RTC_status_t RTC_start_alarm(RTC_alarm_t alarm, RTC_alarm_configuration_t* configuration, RTC_irq_cb_t irq_callback)
 * \brief Configure and start RTC alarm.
 * \param[in]   alarm: Alarm to start.
 * \param[in]   configuration: Pointer to the alarm configuration structure.
 * \param[in]   irq_callback: Function to call on alarm interrupt (can be NULL).
 * \param[out]  none
 * \retval      Current time in seconds.
 *******************************************************************/
RTC_status_t RTC_start_alarm(RTC_alarm_t alarm, RTC_alarm_configuration_t* configuration, RTC_irq_cb_t irq_callback);
#endif

#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & RTC_ALARM_MASK_ALL) != 0)
/*!******************************************************************
 * \fn RTC_status_t RTC_stop_alarm(RTC_alarm_t alarm)
 * \brief Stop RTC alarm.
 * \param[in]   alarm: Alarm to stop.
 * \param[out]  none
 * \retval      Current time in seconds.
 *******************************************************************/
RTC_status_t RTC_stop_alarm(RTC_alarm_t alarm);
#endif

#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & RTC_ALARM_MASK_ALL) != 0)
/*!******************************************************************
 * \fn RTC_status_t RTC_set_time(RTC_time_t* time)
 * \brief Calibrate RTC calendar.
 * \param[in]   time: Pointer to the absolute time to set in calendar.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RTC_status_t RTC_set_time(RTC_time_t* time);
#endif

#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & RTC_ALARM_MASK_ALL) != 0)
/*!******************************************************************
 * \fn RTC_status_t RTC_get_time(RTC_time_t* time)
 * \brief Get RTC time.
 * \param[in]   none
 * \param[out]  time: Pointer to the current absolute RTC time.
 * \retval      Function execution status.
 *******************************************************************/
RTC_status_t RTC_get_time(RTC_time_t* time);
#endif

/*******************************************************************/
#define RTC_exit_error(base) { ERROR_check_exit(rtc_status, RTC_SUCCESS, base) }

/*******************************************************************/
#define RTC_stack_error(base) { ERROR_check_stack(rtc_status, RTC_SUCCESS, base) }

/*******************************************************************/
#define RTC_stack_exit_error(base, code) { ERROR_check_stack_exit(rtc_status, RTC_SUCCESS, base, code) }

#endif /* __RTC_H__ */

#endif /* STM32L0XX_DRIVERS_DISABLE */
