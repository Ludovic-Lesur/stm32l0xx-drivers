/*
 * rtc.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "rtc.h"

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "exti.h"
#include "nvic.h"
#include "rcc_registers.h"
#include "rtc_registers.h"
#include "types.h"

/*** RTC local macros ***/

#define RTC_INIT_TIMEOUT_COUNT  1000000

/*** RTC local structures ***/

/*******************************************************************/
typedef struct {
    volatile uint32_t uptime_seconds;
    RTC_irq_cb_t wakeup_timer_irq_callback;
#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & 0x01) != 0)
    RTC_irq_cb_t alarm_a_irq_callback;
#endif
#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & 0x02) != 0)
    RTC_irq_cb_t alarm_b_irq_callback;
#endif
} RTC_context_t;

/*** RTC local global variables ***/

static RTC_context_t rtc_ctx;

/*** RTC local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RTC_IRQHandler(void) {
    // Wakeup timer interrupt.
    if (((RTC->ISR) & (0b1 << 10)) != 0) {
        // Increment update and call callback.
        if (((RTC->CR) & (0b1 << 14)) != 0) {
            rtc_ctx.uptime_seconds += STM32L0XX_DRIVERS_RTC_WAKEUP_PERIOD_SECONDS;
            if (rtc_ctx.wakeup_timer_irq_callback != NULL) {
                rtc_ctx.wakeup_timer_irq_callback();
            }
        }
        // Clear RTC and EXTI flags.
        RTC->ISR &= ~(0b1 << 10); // WUTF='0'.
        EXTI_clear_line_flag(EXTI_LINE_RTC_WAKEUP_TIMER);
    }
#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & 0x01) != 0)
    // Alarm A interrupt.
    if (((RTC->ISR) & (0b1 << 8)) != 0) {
        // Call callback.
        if (rtc_ctx.alarm_a_irq_callback != NULL) {
            rtc_ctx.alarm_a_irq_callback();
        }
        // Clear RTC and EXTI flags.
        RTC->ISR &= ~(0b1 << 8); // ALRAF='0'.
        EXTI_clear_line_flag(EXTI_LINE_RTC_ALARM);
    }
#endif
#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & 0x02) != 0)
    // Alarm B interrupt.
    if (((RTC->ISR) & (0b1 << 9)) != 0) {
        // Call callback.
        if (rtc_ctx.alarm_b_irq_callback != NULL) {
            rtc_ctx.alarm_b_irq_callback();
        }
        // Clear RTC and EXTI flags.
        RTC->ISR &= ~(0b1 << 9); // ALRBF='0'.
        EXTI_clear_line_flag(EXTI_LINE_RTC_ALARM);
    }
#endif
}

/*******************************************************************/
static RTC_status_t __attribute__((optimize("-O0"))) _RTC_enter_initialization_mode(void) {
    // Local variables.
    RTC_status_t status = RTC_SUCCESS;
    uint32_t loop_count = 0;
    // Enter key.
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    RTC->ISR |= (0b1 << 7); // INIT='1'.
    // Wait for initialization mode.
    while (((RTC->ISR) & (0b1 << 6)) == 0) {
        // Wait for INITF='1' or timeout.
        loop_count++;
        if (loop_count > RTC_INIT_TIMEOUT_COUNT) {
            status = RTC_ERROR_INITIALIZATION_MODE;
            break;
        }
    }
    return status;
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _RTC_exit_initialization_mode(void) {
    RTC->ISR &= ~(0b1 << 7); // INIT='0'.
}

/*** RTC functions ***/

/*******************************************************************/
RTC_status_t RTC_init(RTC_irq_cb_t wakeup_timer_irq_callback, uint8_t nvic_priority) {
    // Local variables.
    RTC_status_t status = RTC_SUCCESS;
    RCC_clock_t rtc_clock = RCC_CLOCK_LSE;
    uint32_t rtc_clock_hz = 0;
    uint32_t loop_count = 0;
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    uint8_t lse_status = 0;
#endif
    // Select peripheral clock.
    RCC->CSR &= ~(0b11 << 16); // Reset bits 16-17.
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
    // Use LSI.
    RCC->CSR |= (0b10 << 16); // RTCSEL='10'.
    rtc_clock = RCC_CLOCK_LSI;
#elif (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    // Get LSE status.
    RCC_get_status(RCC_CLOCK_LSE, &lse_status);
    // Check LSE status.
    if (lse_status != 0) {
        // Use LSE.
        RCC->CSR |= (0b01 << 16); // RTCSEL='01'.
        rtc_clock = RCC_CLOCK_LSE;
    }
    else {
        // Use LSI.
        RCC->CSR |= (0b10 << 16); // RTCSEL='10'.
        rtc_clock = RCC_CLOCK_LSI;
    }
#else
    // Use LSE.
    RCC->CSR |= (0b01 << 16); // RTCSEL='01'.
    rtc_clock = RCC_CLOCK_LSE;
#endif
    // Get clock source frequency.
    RCC_get_frequency_hz(rtc_clock, &rtc_clock_hz);
    // Enable RTC and register access.
    RCC->CSR |= (0b1 << 18); // RTCEN='1'.
    // Enter initialization mode.
    status = _RTC_enter_initialization_mode();
    if (status != RTC_SUCCESS) goto errors;
    // Poll WUTWF flag before accessing reload register.
    while (((RTC->ISR) & (0b1 << 2)) == 0) {
        // Wait for WUTWF='1' or timeout.
        loop_count++;
        if (loop_count > RTC_INIT_TIMEOUT_COUNT) {
            status = RTC_ERROR_WAKEUP_TIMER_REGISTER_ACCESS;
            goto errors;
        }
    }
    // Compute prescaler.
    RTC->PRER = (127 << 16) | (((rtc_clock_hz >> 7) - 1) << 0);
    // Configure wake-up timer.
    RTC->WUTR = (STM32L0XX_DRIVERS_RTC_WAKEUP_PERIOD_SECONDS - 1);
    // Register callback.
    rtc_ctx.wakeup_timer_irq_callback = wakeup_timer_irq_callback;
    // Configure interrupt.
    EXTI_enable_line(EXTI_LINE_RTC_WAKEUP_TIMER, EXTI_TRIGGER_RISING_EDGE);
    NVIC_set_priority(NVIC_INTERRUPT_RTC, nvic_priority);
    // Enable wake-up timer clocked by RTC clock (1Hz).
    RTC->CR = 0x00004424;
    // Clear all flags.
    RTC->ISR &= 0xFFFF005F;
    // Enable interrupt.
    NVIC_enable_interrupt(NVIC_INTERRUPT_RTC);
errors:
    _RTC_exit_initialization_mode();
    return status;
}

/*******************************************************************/
uint32_t RTC_get_uptime_seconds(void) {
    return (rtc_ctx.uptime_seconds);
}

#if (STM32L0XX_DRIVERS_RTC_ALARM_MASK != 0)
/*******************************************************************/
RTC_status_t RTC_start_alarm(RTC_alarm_t alarm, RTC_alarm_configuration_t* configuration, RTC_irq_cb_t irq_callback) {
    // Local variables.
    RTC_status_t status = RTC_SUCCESS;
    uint32_t alrmxr = 0;
    uint8_t tens = 0;
    uint8_t units = 0;
    // Check parameters.
    if (configuration == NULL) {
        status = RTC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    if ((configuration->mode) >= RTC_ALARM_MODE_LAST) {
        status = RTC_ERROR_ALARM_MODE;
        goto errors;
    }
    // Build register value.
    alrmxr |= (((configuration->mode)) << 30);
    // Date.
    tens = (((configuration->date).value) / 10);
    units = ((configuration->date).value) - (tens * 10);
    alrmxr |= (((configuration->date).mask) << 31) | ((tens & 0x03) << 28) | ((units & 0x0F) << 24);
    // Hours.
    tens = (((configuration->hours).value) / 10);
    units = ((configuration->hours).value) - (tens * 10);
    alrmxr |= (((configuration->hours).mask) << 23) | ((tens & 0x03) << 20) | ((units & 0x0F) << 16);
    // Minutes.
    tens = (((configuration->minutes).value) / 10);
    units = ((configuration->minutes).value) - (tens * 10);
    alrmxr |= (((configuration->minutes).mask) << 15) | ((tens & 0x07) << 12) | ((units & 0x0F) << 8);
    // Seconds.
    tens = (((configuration->seconds).value) / 10);
    units = ((configuration->seconds).value) - (tens * 10);
    alrmxr |= (((configuration->seconds).mask) << 7) | ((tens & 0x03) << 4) | ((units & 0x0F) << 0);
    // Enter initialization mode.
    status = _RTC_enter_initialization_mode();
    if (status != RTC_SUCCESS) goto errors;
    // Check alarm.
    switch (alarm) {
#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & 0x01) != 0)
    case RTC_ALARM_A:
        // Register callback.
        rtc_ctx.alarm_a_irq_callback = irq_callback;
        // Configure alarm.
        RTC->ALRMAR = alrmxr;
        // Clear flag.
        RTC->ISR &= ~(0b1 << 8);
        // Enable alarm A.
        EXTI_enable_line(EXTI_LINE_RTC_ALARM, EXTI_TRIGGER_RISING_EDGE);
        RTC->CR |= (0b1 << 12) | (0b1 << 8);
        break;
#endif
#if ((STM32L0XX_DRIVERS_RTC_ALARM_MASK & 0x02) != 0)
    case RTC_ALARM_B:
        // Register callback.
        rtc_ctx.alarm_b_irq_callback = irq_callback;
        // Clear flag.
        RTC->ISR &= ~(0b1 << 9);
        // Enable alarm B.
        EXTI_enable_line(EXTI_LINE_RTC_ALARM, EXTI_TRIGGER_RISING_EDGE);
        RTC->CR |= (0b1 << 13) | (0b1 << 9);
        break;
#endif
    default:
        status = RTC_ERROR_ALARM;
        goto errors;
    }
errors:
    _RTC_exit_initialization_mode();
    return status;
}
#endif

#if (STM32L0XX_DRIVERS_RTC_ALARM_MASK != 0)
/*******************************************************************/
RTC_status_t RTC_stop_alarm(RTC_alarm_t alarm) {
    // Local variables.
    RTC_status_t status = RTC_SUCCESS;
    // Enter initialization mode.
    status = _RTC_enter_initialization_mode();
    if (status != RTC_SUCCESS) goto errors;
    // Check alarm.
    switch (alarm) {
    case RTC_ALARM_A:
        // Stop alarm A.
        RTC->CR &= ~(0b1 << 12) & ~(0b1 << 8);
        // Clear flag.
        RTC->ISR &= ~(0b1 << 8);
        break;
    case RTC_ALARM_B:
        // Stop alarm A.
        RTC->CR &= ~(0b1 << 13) & ~(0b1 << 9);
        // Clear flag.
        RTC->ISR &= ~(0b1 << 9);
        break;
    default:
        status = RTC_ERROR_ALARM;
        goto errors;
    }
errors:
    return status;
}
#endif

#if (STM32L0XX_DRIVERS_RTC_ALARM_MASK != 0)
/*******************************************************************/
RTC_status_t RTC_set_time(RTC_time_t* time) {
    // Local variables.
    RTC_status_t status = RTC_SUCCESS;
    uint32_t tr_value = 0;
    uint32_t dr_value = 0;
    uint8_t tens = 0;
    uint8_t units = 0;
    // Check parameters.
    if (time == NULL) {
        status = RTC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Year.
    tens = ((time->year) - 2000) / 10;
    units = ((time->year) - 2000) - (tens * 10);
    dr_value |= (tens << 20) | (units << 16);
    // Month.
    tens = (time->month) / 10;
    units = (time->month) - (tens * 10);
    dr_value |= (tens << 12) | (units << 8);
    // Date.
    tens = (time->date) / 10;
    units = (time->date) - (tens * 10);
    dr_value |= (tens << 4) | (units << 0);
    // Hour.
    tens = (time->hours) / 10;
    units = (time->hours) - (tens * 10);
    tr_value |= (tens << 20) | (units << 16);
    // Minutes.
    tens = (time->minutes) / 10;
    units = (time->minutes) - (tens * 10);
    tr_value |= (tens << 12) | (units << 8);
    // Seconds.
    tens = (time->seconds) / 10;
    units = (time->seconds) - (tens * 10);
    tr_value |= (tens << 4) | (units << 0);
    // Enter initialization mode.
    status = _RTC_enter_initialization_mode();
    if (status != RTC_SUCCESS) goto errors;
    // Perform update.
    RTC->TR = tr_value;
    RTC->DR = dr_value;
    // Exit initialization mode and restart RTC.
    _RTC_exit_initialization_mode();
errors:
    return status;
}
#endif

#if (STM32L0XX_DRIVERS_RTC_ALARM_MASK != 0)
/*******************************************************************/
RTC_status_t RTC_get_time(RTC_time_t* time) {
    // Local variables.
    RTC_status_t status = RTC_SUCCESS;
    uint32_t dr_value = 0;
    uint32_t tr_value = 0;
    // Check parameters.
    if (time == NULL) {
        status = RTC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read registers.
    dr_value = (RTC->DR) & 0x00FFFF3F; // Mask reserved bits.
    tr_value = (RTC->TR) & 0x007F7F7F; // Mask reserved bits.
    // Parse registers into time structure.
    time->year = 2000 + ((dr_value & (0b1111 << 20)) >> 20) * 10 + ((dr_value & (0b1111 << 16)) >> 16);
    time->month = ((dr_value & (0b1 << 12)) >> 12) * 10 + ((dr_value & (0b1111 << 8)) >> 8);
    time->date = ((dr_value & (0b11 << 4)) >> 4) * 10 + (dr_value & 0b1111);
    time->hours = ((tr_value & (0b11 << 20)) >> 20) * 10 + ((tr_value & (0b1111 << 16)) >> 16);
    time->minutes = ((tr_value & (0b111 << 12)) >> 12) * 10 + ((tr_value & (0b1111 << 8)) >> 8);
    time->seconds = ((tr_value & (0b111 << 4)) >> 4) * 10 + (tr_value & 0b1111);
errors:
    return status;
}
#endif
