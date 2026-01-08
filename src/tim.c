/*
 * tim.c
 *
 *  Created on: 04 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#include "tim.h"

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#include "error.h"
#include "iwdg.h"
#include "maths.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_registers.h"
#include "rtc.h"
#include "tim_registers.h"
#include "types.h"

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_ALL) != 0)

/*** TIM local macros ***/

#define TIM_TIMEOUT_COUNT                   10000000

#define TIM_REGISTER_MASK_ARR_PSC_CCR       0x0000FFFF

#define TIM_ARR_VALUE_MIN                   0x0001
#define TIM_ARR_VALUE_MAX                   TIM_REGISTER_MASK_ARR_PSC_CCR

#define TIM_CNT_VALUE_MAX                   TIM_REGISTER_MASK_ARR_PSC_CCR

#define TIM_MCH_TARGET_TRIGGER_CLOCK_HZ     2048
#define TIM_MCH_PRESCALER_ETRF_LSE          1
#define TIM_MCH_PRESCALER_PSC_LSE           ((tim_clock_hz) / (TIM_MCH_TARGET_TRIGGER_CLOCK_HZ * TIM_MCH_PRESCALER_ETRF_LSE))
#define TIM_MCH_PRESCALER_ETRF_HSI          8
#define TIM_MCH_PRESCALER_PSC_HSI           ((tim_clock_hz) / (TIM_MCH_TARGET_TRIGGER_CLOCK_HZ * TIM_MCH_PRESCALER_ETRF_HSI))
#define TIM_MCH_CLOCK_SWITCH_LATENCY_MS     2
#define TIM_MCH_TIMER_PERIOD_MS_MIN         1
#define TIM_MCH_TIMER_PERIOD_MS_MAX         ((TIM_CNT_VALUE_MAX * 1000) / (tim_mch_ctx.ck_cnt_hz))
#define TIM_MCH_WATCHDOG_PERIOD_SECONDS     ((TIM_CNT_VALUE_MAX / tim_mch_ctx.ck_cnt_hz) + 5)

#define TIM_CAL_INPUT_CAPTURE_PRESCALER     2
#define TIM_CAL_MEDIAN_FILTER_SIZE          9
#define TIM_CAL_CENTER_AVERAGE_SIZE         3

/*** TIM local structures ***/

/*******************************************************************/
typedef struct {
    TIM_registers_t* peripheral;
    uint8_t number_of_channels;
    volatile uint32_t* rcc_reset;
    volatile uint32_t* rcc_enr;
    volatile uint32_t* rcc_smenr;
    uint32_t rcc_mask;
    NVIC_interrupt_t nvic_interrupt;
} TIM_descriptor_t;

/*******************************************************************/
typedef enum {
    TIM_MODE_NONE = 0,
    TIM_MODE_STANDARD,
    TIM_MODE_MULTI_CHANNEL,
    TIM_MODE_CALIBRATION,
    TIM_MODE_PWM,
    TIM_MODE_OPM,
    TIM_MODE_LAST
} TIM_mode_t;

/*******************************************************************/
typedef void (*TIM_irq_handler_cb_t)(TIM_instance_t instance);

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*******************************************************************/
typedef struct {
    TIM_completion_irq_cb_t irq_callback;
} TIM_STD_context_t;
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
typedef struct {
    uint32_t duration_ms;
    TIM_waiting_mode_t waiting_mode;
    volatile uint8_t running_flag;
    volatile uint8_t irq_flag;
} TIM_MCH_channel_context_t;
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
typedef struct {
    RCC_clock_t clock_source;
    uint32_t ck_cnt_hz;
    TIM_MCH_channel_context_t channel[TIM_CHANNEL_LAST];
} TIM_MCH_context_t;
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*******************************************************************/
typedef struct {
    volatile uint16_t ccr1_start;
    volatile uint16_t ccr1_end;
    volatile uint16_t capture_count;
    volatile uint8_t capture_done;
} TIM_CAL_context_t;
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_PWM) != 0)
/*******************************************************************/
typedef struct {
    uint32_t channels_duty_cycle;
} TIM_PWM_context_t;
#endif

/*******************************************************************/
typedef struct {
    TIM_mode_t mode;
    TIM_irq_handler_cb_t irq_handler;
} TIM_context_t;

/*** TIM local global variables ***/

static const TIM_descriptor_t TIM_DESCRIPTOR[TIM_INSTANCE_LAST] = {
    { TIM2,  4, &(RCC->APB1RSTR), &(RCC->APB1SMENR), &(RCC->APB1ENR), (0b1 << 0), NVIC_INTERRUPT_TIM2 },
    { TIM21, 2, &(RCC->APB2RSTR), &(RCC->APB2SMENR), &(RCC->APB2ENR), (0b1 << 2), NVIC_INTERRUPT_TIM21 },
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 2) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    { TIM22, 2, &(RCC->APB2RSTR), &(RCC->APB2SMENR), &(RCC->APB2ENR), (0b1 << 5), NVIC_INTERRUPT_TIM22 },
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    { TIM6,  0, &(RCC->APB1RSTR), &(RCC->APB1SMENR), &(RCC->APB1ENR), (0b1 << 4), NVIC_INTERRUPT_TIM6 },
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    { TIM3,  4, &(RCC->APB1RSTR), &(RCC->APB1SMENR), &(RCC->APB1ENR), (0b1 << 1), NVIC_INTERRUPT_TIM3 },
    { TIM7,  0, &(RCC->APB1RSTR), &(RCC->APB1SMENR), &(RCC->APB1ENR), (0b1 << 5), NVIC_INTERRUPT_TIM7 },
#endif
};

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
static TIM_STD_context_t tim_std_ctx[TIM_INSTANCE_LAST] = {
    [0 ... (TIM_INSTANCE_LAST - 1)] = {
        .irq_callback = NULL
    }
};
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
// Not defined as array because only supported by one instance (TIM2).
static TIM_MCH_context_t tim_mch_ctx = {
    .clock_source = RCC_CLOCK_NONE,
    .ck_cnt_hz = 0,
    .channel = {
        [0 ... (TIM_CHANNEL_LAST - 1)] = {
            .duration_ms = 0,
            .waiting_mode = TIM_WAITING_MODE_ACTIVE,
            .running_flag = 0,
            .irq_flag = 0
        }
    }
};
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
// Not defined as array because only supported by one instance (TIM21).
static TIM_CAL_context_t tim_cal_ctx = {
    .ccr1_start = 0,
    .ccr1_end = 0,
    .capture_count = 0,
    .capture_done = 0
};
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_PWM) != 0)
static TIM_PWM_context_t tim_pwm_ctx[TIM_INSTANCE_LAST] = {
    [0 ... (TIM_INSTANCE_LAST - 1)] = {
        .channels_duty_cycle = 0
    }
};
#endif

static TIM_context_t tim_ctx[TIM_INSTANCE_LAST] = {
    [0 ... (TIM_INSTANCE_LAST - 1)] = {
        .mode = TIM_MODE_NONE,
        .irq_handler = NULL
    }
};

/*** TIM local functions ***/

/*******************************************************************/
#define _TIM_check_instance(instance) { \
    /* Check instance */ \
    if (instance >= TIM_INSTANCE_LAST) { \
        status = TIM_ERROR_INSTANCE; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _TIM_check_mode(instance, expected_mode) { \
    /* Check timer mode */ \
    if (tim_ctx[instance].mode != expected_mode) { \
        status = TIM_ERROR_MODE; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _TIM_check_channel(instance, channel) { \
    /* Check channel */ \
    if (channel >= TIM_CHANNEL_LAST) { \
        status = TIM_ERROR_CHANNEL; \
        goto errors; \
    } \
    if (channel >= TIM_DESCRIPTOR[instance].number_of_channels) { \
        status = TIM_ERROR_CHANNEL_NOT_SUPPORTED; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _TIM_check_gpio(void) { \
    /* Check parameters */ \
    if (pins == NULL) { \
        status = TIM_ERROR_NULL_PARAMETER; \
        goto errors; \
    } \
    if ((pins->list) == NULL) { \
        status = TIM_ERROR_NULL_PARAMETER; \
        goto errors; \
    } \
    if (((pins->list_size) == 0) || ((pins->list_size) > TIM_CHANNEL_LAST)) { \
        status = TIM_ERROR_NUMBER_OF_PINS; \
        goto errors; \
    } \
}

/*******************************************************************/
#define _TIM_irq_handler(instance) { \
    /* Execute internal callback */ \
    if (tim_ctx[instance].irq_handler != NULL) { \
        tim_ctx[instance].irq_handler(instance); \
    } \
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM2_IRQHandler(void) {
    // Execute internal callback.
    _TIM_irq_handler(TIM_INSTANCE_TIM2);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM21_IRQHandler(void) {
    // Execute internal callback.
    _TIM_irq_handler(TIM_INSTANCE_TIM21);
}

#if ((STM32L0XX_REGISTERS_MCU_CATEGORY == 2) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5))
/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM22_IRQHandler(void) {
    // Execute internal callback.
    _TIM_irq_handler(TIM_INSTANCE_TIM22);
}
#endif

#if ((STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5))
/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM6_IRQHandler(void) {
    // Execute internal callback.
    _TIM_irq_handler(TIM_INSTANCE_TIM6);
}
#endif

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM3_IRQHandler(void) {
    // Execute internal callback.
    _TIM_irq_handler(TIM_INSTANCE_TIM3);
}
#endif

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM7_IRQHandler(void) {
    // Execute internal callback.
    _TIM_irq_handler(TIM_INSTANCE_TIM7);
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _TIM_STD_irq_handler(TIM_instance_t instance) {
    // Update interrupt.
    if (((TIM_DESCRIPTOR[instance].peripheral->SR) & (0b1 << 0)) != 0) {
        // Call callback.
        if (tim_std_ctx[instance].irq_callback != NULL) {
            tim_std_ctx[instance].irq_callback();
        }
        // Clear flag.
        TIM_DESCRIPTOR[instance].peripheral->SR &= ~(0b1 << 0);
    }
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _TIM_MCH_irq_handler(TIM_instance_t instance) {
    // Local variables.
    uint8_t channel_idx = 0;
    uint8_t channel_mask = 0;
    // Channels loop.
    for (channel_idx = 0; channel_idx < TIM_CHANNEL_LAST; channel_idx++) {
        // Compute mask.
        channel_mask = (0b1 << (channel_idx + 1));
        // Check flag.
        if (((TIM_DESCRIPTOR[instance].peripheral->SR) & channel_mask) != 0) {
            // Set local flag if channel is active.
            tim_mch_ctx.channel[channel_idx].irq_flag = tim_mch_ctx.channel[channel_idx].running_flag;
            // Clear flag.
            TIM_DESCRIPTOR[instance].peripheral->SR &= ~(channel_mask);
        }
    }
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
static void _TIM_MCH_compute_compare_value(TIM_instance_t instance, TIM_channel_t channel) {
    // Local variables.
    uint32_t reg_value = (TIM_DESCRIPTOR[instance].peripheral->CCRx[channel] & (~TIM_REGISTER_MASK_ARR_PSC_CCR));
    // Compute next compare value.
    reg_value |= (((TIM_DESCRIPTOR[instance].peripheral->CNT) + ((tim_mch_ctx.channel[channel].duration_ms * tim_mch_ctx.ck_cnt_hz) / (1000))) % TIM_CNT_VALUE_MAX);
    // Set register.
    TIM_DESCRIPTOR[instance].peripheral->CCRx[channel] = reg_value;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
static TIM_status_t _TIM_MCH_internal_watchdog(uint32_t time_start, uint32_t* time_reference) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint32_t time = RTC_get_uptime_seconds();
    // If the RTC is correctly clocked, it will be used as internal watchdog and the IWDG can be reloaded.
    // If the RTC is not running anymore due to a clock failure, the IWDG is not reloaded and will reset the MCU.
    if (time != (*time_reference)) {
        // Update time reference and reload IWDG.
        (*time_reference) = time;
        IWDG_reload();
    }
    // Internal watchdog.
    if (time > (time_start + TIM_MCH_WATCHDOG_PERIOD_SECONDS)) {
        status = TIM_ERROR_COMPLETION_WATCHDOG;
    }
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _TIM_CAL_irq_handler(TIM_instance_t instance) {
    // TI1 interrupt.
    if (((TIM_DESCRIPTOR[instance].peripheral->SR) & (0b1 << 1)) != 0) {
        // Update flags.
        if (tim_cal_ctx.capture_done == 0) {
            // Check count.
            if (tim_cal_ctx.capture_count == 0) {
                // Store start value.
                tim_cal_ctx.ccr1_start = (uint16_t) (TIM_DESCRIPTOR[instance].peripheral->CCR1);
                tim_cal_ctx.capture_count++;
            }
            else {
                // Check rollover.
                if ((TIM_DESCRIPTOR[instance].peripheral->CCR1) > tim_cal_ctx.ccr1_end) {
                    // Store new value.
                    tim_cal_ctx.ccr1_end = (uint16_t) (TIM_DESCRIPTOR[instance].peripheral->CCR1);
                    tim_cal_ctx.capture_count++;
                }
                else {
                    // Capture complete.
                    tim_cal_ctx.capture_done = 1;
                }
            }
        }
        TIM_DESCRIPTOR[instance].peripheral->SR &= ~(0b1 << 1);
    }
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*******************************************************************/
static TIM_status_t _TIM_CAL_single_capture(TIM_instance_t instance, int32_t* ref_clock_pulse_count, int32_t* mco_pulse_count) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint32_t loop_count = 0;
    // Reset timer context.
    tim_cal_ctx.ccr1_start = 0;
    tim_cal_ctx.ccr1_end = 0;
    tim_cal_ctx.capture_count = 0;
    tim_cal_ctx.capture_done = 0;
    // Reset timer.
    TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
    // Enable interrupt.
    TIM_DESCRIPTOR[instance].peripheral->SR &= 0xFFFFF9B8; // Clear all flags.
    NVIC_enable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    // Enable TIM peripheral.
    TIM_DESCRIPTOR[instance].peripheral->CCER |= (0b1 << 0); // CC1E='1'.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 0); // CEN='1'.
    // Wait for capture to complete.
    while (tim_cal_ctx.capture_done == 0) {
        // Manage timeout.
        loop_count++;
        if (loop_count > TIM_TIMEOUT_COUNT) {
            status = TIM_ERROR_CAPTURE_TIMEOUT;
            goto errors;
        }
    }
    // Update results.
    (*ref_clock_pulse_count) = (int32_t) (tim_cal_ctx.ccr1_end - tim_cal_ctx.ccr1_start);
    (*mco_pulse_count) = (int32_t) (TIM_CAL_INPUT_CAPTURE_PRESCALER * (tim_cal_ctx.capture_count - 1));
    // Disable interrupt.
    NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    // Stop counter.
    TIM_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 0); // CEN='0'.
    TIM_DESCRIPTOR[instance].peripheral->CCER &= ~(0b1 << 0); // CC1E='0'.
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _TIM_OPM_irq_handler(TIM_instance_t instance) {
    // Update interrupt.
    if (((TIM_DESCRIPTOR[instance].peripheral->SR) & (0b1 << 0)) != 0) {
        // Clear flag.
        TIM_DESCRIPTOR[instance].peripheral->SR &= ~(0b1 << 0);
    }
}
#endif

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _TIM_reset(TIM_instance_t instance) {
    // Local variables.
    uint8_t count = 0;
    // Perform manual reset and delay.
    (*TIM_DESCRIPTOR[instance].rcc_reset) |= (TIM_DESCRIPTOR[instance].rcc_mask);
    for (count = 0; count < 100; count++);
    (*TIM_DESCRIPTOR[instance].rcc_reset) &= ~(TIM_DESCRIPTOR[instance].rcc_mask);
}

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & (TIM_MODE_MASK_STANDARD | TIM_MODE_MASK_PWM | TIM_MODE_MASK_OPM)) != 0)
/*******************************************************************/
static TIM_status_t _TIM_compute_psc_arr(TIM_instance_t instance, uint32_t tim_clock_hz, uint64_t expected_period_ns, uint32_t* arr) {
    // Local variables.
    TIM_status_t status = TIM_ERROR_ARR_VALUE;
    uint64_t arr_u64 = 0;
    uint32_t psc = 0;
    uint32_t reg_value = 0;
    uint8_t idx = 0;
    // Check parameter.
    if ((expected_period_ns == 0) || (tim_clock_hz == 0)) goto errors;
    // Search prescaler to reach expected period.
    for (idx = 0; idx <= MATH_U16_SIZE_BITS; idx++) {
        // Try next power of 2.
        psc = (0b1 << idx);
        // Compute ARR.
        arr_u64 = (expected_period_ns * ((uint64_t) tim_clock_hz));
        arr_u64 /= (((uint64_t) MATH_POWER_10[9]) * ((uint64_t) psc));
        arr_u64 -= 1;
        // Check value.
        if ((arr_u64 > TIM_ARR_VALUE_MIN) && (arr_u64 < TIM_ARR_VALUE_MAX)) {
            // Write PSC..
            reg_value = ((TIM_DESCRIPTOR[instance].peripheral->PSC) & (~TIM_REGISTER_MASK_ARR_PSC_CCR));
            reg_value |= ((psc - 1) & TIM_REGISTER_MASK_ARR_PSC_CCR);
            TIM_DESCRIPTOR[instance].peripheral->PSC = reg_value;
            // Write ARR.
            reg_value = ((TIM_DESCRIPTOR[instance].peripheral->ARR) & (~TIM_REGISTER_MASK_ARR_PSC_CCR));
            reg_value |= (((uint16_t) arr_u64) & TIM_REGISTER_MASK_ARR_PSC_CCR);
            TIM_DESCRIPTOR[instance].peripheral->ARR = reg_value;
            // Update computed value.
            if (arr != NULL) {
                (*arr) = (uint32_t) arr_u64;
            }
            // Update status and exit.
            status = TIM_SUCCESS;
            break;
        }
    }
errors:
    return status;
}
#endif

/*******************************************************************/
static void _TIM_de_init(TIM_instance_t instance) {
    // Disable interrupt.
    NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    // Disable timer.
    TIM_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 0); // CEN='0'.
    // Disable peripheral clock.
    (*TIM_DESCRIPTOR[instance].rcc_enr) &= ~(TIM_DESCRIPTOR[instance].rcc_mask);
    // Update mode.
    tim_ctx[instance].mode = TIM_MODE_NONE;
    tim_ctx[instance].irq_handler = NULL;
}

/*** TIM functions ***/

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*******************************************************************/
TIM_status_t TIM_STD_init(TIM_instance_t instance, uint8_t nvic_priority) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance and mode.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_NONE);
    // Reset peripheral.
    _TIM_reset(instance);
    // Update local interrupt handler.
    tim_ctx[instance].irq_handler = &_TIM_STD_irq_handler;
    // Enable peripheral clock.
    (*TIM_DESCRIPTOR[instance].rcc_enr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    (*TIM_DESCRIPTOR[instance].rcc_smenr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    // Enable preload.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 7); // ARPE='1'.
    // Use update event as trigger output.
    TIM_DESCRIPTOR[instance].peripheral->CR2 |= (0b010 << 4); // MMS='010'.
    // Set trigger selection to reserved value to ensure there is no link with other timers.
    TIM_DESCRIPTOR[instance].peripheral->SMCR |= (0b011 << 4);
    // Set interrupt priority.
    NVIC_set_priority(TIM_DESCRIPTOR[instance].nvic_interrupt, nvic_priority);
    // Generate event to update registers.
    TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
    // Update mode.
    tim_ctx[instance].mode = TIM_MODE_STANDARD;
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*******************************************************************/
TIM_status_t TIM_STD_de_init(TIM_instance_t instance) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance.
    _TIM_check_instance(instance);
    // Release timer.
    _TIM_de_init(instance);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*******************************************************************/
TIM_status_t TIM_STD_start(TIM_instance_t instance, uint32_t period_value, TIM_unit_t period_unit, TIM_completion_irq_cb_t irq_callback) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint32_t tim_clock_hz = 0;
    uint64_t period_ns = 0;
    // Check instance and mode.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_STANDARD);
    // Check unit.
    switch (period_unit) {
    case TIM_UNIT_NS:
        period_ns = ((uint64_t) period_value);
        break;
    case TIM_UNIT_US:
        period_ns = ((uint64_t) period_value) * ((uint64_t) MATH_POWER_10[3]);
        break;
    case TIM_UNIT_MS:
        period_ns = ((uint64_t) period_value) * ((uint64_t) MATH_POWER_10[6]);
        break;
    default:
        status = TIM_ERROR_UNIT;
        goto errors;
    }
    // Get clock source frequency.
    RCC_get_frequency_hz(RCC_CLOCK_SYSTEM, &tim_clock_hz);
    // Compute ARR and PSC values.
    status = _TIM_compute_psc_arr(instance, tim_clock_hz, (uint64_t) period_ns, NULL);
    if (status != TIM_SUCCESS) goto errors;
    // Generate event to update registers.
    TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
    // Clear flag.
    TIM_DESCRIPTOR[instance].peripheral->SR &= ~(0b1 << 0); // UIF='0'.
    // Register callback.
    tim_std_ctx[instance].irq_callback = irq_callback;
    // Enable interrupt if callback is given.
    if (irq_callback != NULL) {
        TIM_DESCRIPTOR[instance].peripheral->DIER |= (0b1 << 0);
        NVIC_enable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    }
    else {
        TIM_DESCRIPTOR[instance].peripheral->DIER &= ~(0b1 << 0);
        NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    }
    // Start timer.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 0);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_STANDARD) != 0)
/*******************************************************************/
TIM_status_t TIM_STD_stop(TIM_instance_t instance) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance and mode.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_STANDARD);
    // Stop timer.
    TIM_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 0);
    // Disable interrupt.
    NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
TIM_status_t TIM_MCH_init(TIM_instance_t instance, uint8_t nvic_priority) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint32_t tim_clock_hz = 0;
    uint32_t psc = 0;
    uint32_t reg_value = 0;
    uint8_t idx = 0;
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    uint8_t lse_status = 0;
#endif
    // Check instance and mode.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_NONE);
    // Check supported instances.
    if (instance != TIM_INSTANCE_TIM2) {
        status = TIM_ERROR_INSTANCE_NOT_SUPPORTED;
        goto errors;
    }
    // Reset peripheral.
    _TIM_reset(instance);
    // Init context.
    for (idx = 0; idx < TIM_CHANNEL_LAST; idx++) {
        tim_mch_ctx.channel[idx].duration_ms = 0;
        tim_mch_ctx.channel[idx].waiting_mode = TIM_WAITING_MODE_ACTIVE;
        tim_mch_ctx.channel[idx].running_flag = 0;
        tim_mch_ctx.channel[idx].irq_flag = 0;
    }
    // Init common context.
    tim_ctx[instance].irq_handler = &_TIM_MCH_irq_handler;
    // Select trigger.
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
    tim_mch_ctx.clock_source = RCC_CLOCK_HSI;
#elif (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    RCC_get_status(RCC_CLOCK_LSE, &lse_status);
    tim_mch_ctx.clock_source = (lse_status != 0) ? RCC_CLOCK_LSE : RCC_CLOCK_HSI;
#else
    tim_mch_ctx.clock_source = RCC_CLOCK_LSE;
#endif
    // Get clock source frequency.
    RCC_get_frequency_hz(tim_mch_ctx.clock_source, &tim_clock_hz);
    // Enable peripheral clock.
    (*TIM_DESCRIPTOR[instance].rcc_enr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    (*TIM_DESCRIPTOR[instance].rcc_smenr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    // Configure channels.
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
    RCC->CR |= (0b1 << 5); // HSI16OUTEN='1'.
    TIM_DESCRIPTOR[instance].peripheral->SMCR |= (0b11 << 12); // ETRF prescaler = 8 (minimum 4 due to CK_INT clock ratio constraint).
    TIM_DESCRIPTOR[instance].peripheral->OR |= (0b011 << 0);
    // Update clock frequency.
    psc = TIM_MCH_PRESCALER_PSC_HSI;
    tim_mch_ctx.ck_cnt_hz = ((tim_clock_hz) / (TIM_MCH_PRESCALER_ETRF_HSI * TIM_MCH_PRESCALER_PSC_HSI));
#elif (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    // Select trigger.
    if (lse_status != 0) {
        // Use LSE as trigger.
        RCC->CR &= ~(0b1 << 5); // HSI16OUTEN='0'.
        TIM_DESCRIPTOR[instance].peripheral->SMCR &= ~(0b11 << 12);// No prescaler on ETRF.
        TIM_DESCRIPTOR[instance].peripheral->OR |= (0b101 << 0);
        // Update clock frequency.
        psc = TIM_MCH_PRESCALER_PSC_LSE;
        tim_mch_ctx.ck_cnt_hz = ((tim_clock_hz) / (TIM_MCH_PRESCALER_ETRF_LSE * TIM_MCH_PRESCALER_PSC_LSE));
    }
    else {
        // Use HSI as trigger.
        RCC->CR |= (0b1 << 5); // HSI16OUTEN='1'.
        TIM_DESCRIPTOR[instance].peripheral->SMCR |= (0b11 << 12);// ETRF prescaler = 8 (minimum 4 due to CK_INT clock ratio constraint).
        TIM_DESCRIPTOR[instance].peripheral->OR |= (0b011 << 0);
        // Update clock frequency.
        psc = TIM_MCH_PRESCALER_PSC_HSI;
        tim_mch_ctx.ck_cnt_hz = ((tim_clock_hz) / (TIM_MCH_PRESCALER_ETRF_HSI * TIM_MCH_PRESCALER_PSC_HSI));
    }
#else
    // Use LSE as trigger.
    RCC->CR &= ~(0b1 << 5); // HSI16OUTEN='0'.
    TIM_DESCRIPTOR[instance].peripheral->SMCR &= ~(0b11 << 12); // No prescaler on ETRF.
    TIM_DESCRIPTOR[instance].peripheral->OR |= (0b101 << 0);
    // Update clock frequency.
    psc = TIM_MCH_PRESCALER_PSC_LSE;
    tim_mch_ctx.ck_cnt_hz = ((tim_clock_hz) / (TIM_MCH_PRESCALER_ETRF_LSE * TIM_MCH_PRESCALER_PSC_LSE));
#endif
    // No overflow.
    reg_value = ((TIM_DESCRIPTOR[instance].peripheral->PSC) & (~TIM_REGISTER_MASK_ARR_PSC_CCR));
    reg_value |= ((psc - 1) & TIM_REGISTER_MASK_ARR_PSC_CCR);
    TIM_DESCRIPTOR[instance].peripheral->PSC = reg_value;
    TIM_DESCRIPTOR[instance].peripheral->ARR |= TIM_REGISTER_MASK_ARR_PSC_CCR;
    // Use external clock mode 2.
    TIM_DESCRIPTOR[instance].peripheral->SMCR |= (0b1 << 14) | (0b111 << 4);
    // Configure channels 1-4 in output compare mode.
    TIM_DESCRIPTOR[instance].peripheral->CCMR1 &= 0xFFFF0000;
    TIM_DESCRIPTOR[instance].peripheral->CCMR2 &= 0xFFFF0000;
    TIM_DESCRIPTOR[instance].peripheral->CCER &= 0xFFFF0000;
    // Set interrupt priority.
    NVIC_set_priority(TIM_DESCRIPTOR[instance].nvic_interrupt, nvic_priority);
    // Generate event to update registers.
    TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
    // Update mode.
    tim_ctx[instance].mode = TIM_MODE_MULTI_CHANNEL;
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
TIM_status_t TIM_MCH_de_init(TIM_instance_t instance) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance.
    _TIM_check_instance(instance);
    // Release timer.
    _TIM_de_init(instance);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
TIM_status_t TIM_MCH_start_channel(TIM_instance_t instance, TIM_channel_t channel, uint32_t period_ms, TIM_waiting_mode_t waiting_mode) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint32_t local_period_ms = period_ms;
    uint32_t period_min_ms = TIM_MCH_TIMER_PERIOD_MS_MIN;
    // Check instance, mode and channel.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_MULTI_CHANNEL);
    _TIM_check_channel(instance, channel);
    // Check parameters.
    if (waiting_mode >= TIM_WAITING_MODE_LAST) {
        status = TIM_ERROR_WAITING_MODE;
        goto errors;
    }
    if (waiting_mode == TIM_WAITING_MODE_LOW_POWER_SLEEP) {
        // Compensate clock switch latency.
        period_min_ms += TIM_MCH_CLOCK_SWITCH_LATENCY_MS;
    }
    if (period_ms < period_min_ms) {
        status = TIM_ERROR_DURATION_UNDERFLOW;
        goto errors;
    }
    if (period_ms > TIM_MCH_TIMER_PERIOD_MS_MAX) {
        status = TIM_ERROR_DURATION_OVERFLOW;
        goto errors;
    }
    if (waiting_mode == TIM_WAITING_MODE_LOW_POWER_SLEEP) {
        local_period_ms -= TIM_MCH_CLOCK_SWITCH_LATENCY_MS;
    }
    // Update channel context.
    tim_mch_ctx.channel[channel].duration_ms = local_period_ms;
    tim_mch_ctx.channel[channel].waiting_mode = waiting_mode;
    tim_mch_ctx.channel[channel].running_flag = 1;
    tim_mch_ctx.channel[channel].irq_flag = 0;
    // Compute compare value.
    _TIM_MCH_compute_compare_value(instance, channel);
    // Clear flag.
    TIM_DESCRIPTOR[instance].peripheral->SR &= ~(0b1 << (channel + 1));
    // Enable interrupt.
    TIM_DESCRIPTOR[instance].peripheral->DIER |= (0b1 << (channel + 1));
    NVIC_enable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    // Enable counter.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 0);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
TIM_status_t TIM_MCH_stop_channel(TIM_instance_t instance, TIM_channel_t channel) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance, mode and channel.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_MULTI_CHANNEL);
    _TIM_check_channel(instance, channel);
    // Disable interrupt.
    TIM_DESCRIPTOR[instance].peripheral->DIER &= ~(0b1 << (channel + 1));
    // Clear flag.
    TIM_DESCRIPTOR[instance].peripheral->SR &= ~(0b1 << (channel + 1));
    // Disable channel.
    tim_mch_ctx.channel[channel].running_flag = 0;
    tim_mch_ctx.channel[channel].irq_flag = 0;
    // Disable counter if all channels are stopped.
    if (((TIM_DESCRIPTOR[instance].peripheral->DIER) & 0x0000001E) == 0) {
        // Disable counter.
        TIM_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 0);
        // Disable interrupt.
        NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    }
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
TIM_status_t TIM_MCH_get_channel_status(TIM_instance_t instance, TIM_channel_t channel, uint8_t* timer_has_elapsed) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance, mode and channel.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_MULTI_CHANNEL);
    _TIM_check_channel(instance, channel);
    // Check parameters.
    if (timer_has_elapsed == NULL) {
        status = TIM_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Update flag.
    (*timer_has_elapsed) = ((tim_mch_ctx.channel[channel].running_flag == 0) || (tim_mch_ctx.channel[channel].irq_flag != 0)) ? 1 : 0;
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_MULTI_CHANNEL) != 0)
/*******************************************************************/
TIM_status_t TIM_MCH_wait_channel_completion(TIM_instance_t instance, TIM_channel_t channel) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    RCC_status_t rcc_status = RCC_SUCCESS;
    uint32_t time_start = RTC_get_uptime_seconds();
    uint32_t time_reference = 0;
    // Check instance, mode and channel.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_MULTI_CHANNEL);
    _TIM_check_channel(instance, channel);
    // Directly exit if the IRQ already occurred.
    if ((tim_mch_ctx.channel[channel].running_flag == 0) || (tim_mch_ctx.channel[channel].irq_flag != 0)) goto errors;
    // Sleep until channel is not running.
    switch (tim_mch_ctx.channel[channel].waiting_mode) {
    case TIM_WAITING_MODE_ACTIVE:
        // Active loop.
        while (tim_mch_ctx.channel[channel].irq_flag == 0) {
            // Internal watchdog.
            status = _TIM_MCH_internal_watchdog(time_start, &time_reference);
            if (status != TIM_SUCCESS) goto errors;
        }
        break;
    case TIM_WAITING_MODE_SLEEP:
        // Enter sleep mode.
        while (tim_mch_ctx.channel[channel].irq_flag == 0) {
            PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
            // Internal watchdog.
            status = _TIM_MCH_internal_watchdog(time_start, &time_reference);
            if (status != TIM_SUCCESS) goto errors;
        }
        break;
    case TIM_WAITING_MODE_LOW_POWER_SLEEP:
        // Check trigger source.
        if (tim_mch_ctx.clock_source == RCC_CLOCK_LSE) {
            // Switch to MSI.
            rcc_status = RCC_switch_to_msi(RCC_MSI_RANGE_1_131KHZ);
            RCC_exit_error(TIM_ERROR_BASE_RCC);
            // Enter low power sleep mode.
            while (tim_mch_ctx.channel[channel].irq_flag == 0) {
                PWR_enter_sleep_mode(PWR_SLEEP_MODE_LOW_POWER);
                // Internal watchdog.
                status = _TIM_MCH_internal_watchdog(time_start, &time_reference);
                if (status != TIM_SUCCESS) goto errors;
            }
            // Restore previous clock.
            rcc_status = RCC_restore_previous_system_clock();
            RCC_exit_error(TIM_ERROR_BASE_RCC);
        }
        else {
            // Enter sleep mode.
            while (tim_mch_ctx.channel[channel].irq_flag == 0) {
                PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
                // Internal watchdog.
                status = _TIM_MCH_internal_watchdog(time_start, &time_reference);
                if (status != TIM_SUCCESS) goto errors;
            }
        }
        break;
    default:
        status = TIM_ERROR_WAITING_MODE;
        goto errors;
    }
    // Clear flag and update compare value for next IRQ.
    tim_mch_ctx.channel[channel].irq_flag = 0;
    _TIM_MCH_compute_compare_value(instance, channel);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*******************************************************************/
TIM_status_t TIM_CAL_init(TIM_instance_t instance, uint8_t nvic_priority) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance and mode.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_NONE);
    // Check supported instances.
    if (instance != TIM_INSTANCE_TIM21) {
        status = TIM_ERROR_INSTANCE_NOT_SUPPORTED;
        goto errors;
    }
    // Reset peripheral.
    _TIM_reset(instance);
    // Update local interrupt handler.
    tim_ctx[instance].irq_handler = &_TIM_CAL_irq_handler;
    // Enable peripheral clock.
    (*TIM_DESCRIPTOR[instance].rcc_enr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    (*TIM_DESCRIPTOR[instance].rcc_smenr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    // Channel input on TI1, CH1 mapped on MCO and capture done every 2 edges.
    TIM_DESCRIPTOR[instance].peripheral->CCMR1 |= (0b01 << 2) | (0b01 << 0);
    TIM_DESCRIPTOR[instance].peripheral->OR |= (0b111 << 2);
    // Set trigger selection to reserved value to ensure there is no link with other timers.
    TIM_DESCRIPTOR[instance].peripheral->SMCR |= (0b011 << 4);
    // Enable interrupt.
    TIM_DESCRIPTOR[instance].peripheral->DIER |= (0b1 << 1); // CC1IE='1'.
    NVIC_set_priority(TIM_DESCRIPTOR[instance].nvic_interrupt, nvic_priority);
    // Generate event to update registers.
    TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
    // Update mode.
    tim_ctx[instance].mode = TIM_MODE_CALIBRATION;
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*******************************************************************/
TIM_status_t TIM_CAL_de_init(TIM_instance_t instance) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance.
    _TIM_check_instance(instance);
    // Release timer.
    _TIM_de_init(instance);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_CALIBRATION) != 0)
/*******************************************************************/
TIM_status_t TIM_CAL_mco_capture(TIM_instance_t instance, int32_t* ref_clock_pulse_count, int32_t* mco_pulse_count) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    MATH_status_t math_status = MATH_SUCCESS;
    int32_t ref_clock_pulse_count_buffer[TIM_CAL_MEDIAN_FILTER_SIZE] = { 0x00 };
    int32_t mco_pulse_count_buffer[TIM_CAL_MEDIAN_FILTER_SIZE] = { 0x00 };
    uint8_t idx = 0;
    // Check instance and mode.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_CALIBRATION);
    // Check parameters.
    if ((ref_clock_pulse_count == NULL) || (mco_pulse_count == NULL)) {
        status = TIM_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Captures loop.
    for (idx = 0; idx < TIM_CAL_MEDIAN_FILTER_SIZE; idx++) {
        status = _TIM_CAL_single_capture(instance, &(ref_clock_pulse_count_buffer[idx]), &(mco_pulse_count_buffer[idx]));
        if (status != TIM_SUCCESS) goto errors;
    }
    // Apply median filter.
    math_status = MATH_median_filter(ref_clock_pulse_count_buffer, TIM_CAL_MEDIAN_FILTER_SIZE, TIM_CAL_CENTER_AVERAGE_SIZE, ref_clock_pulse_count);
    MATH_exit_error(TIM_ERROR_BASE_MATH);
    math_status = MATH_median_filter(mco_pulse_count_buffer, TIM_CAL_MEDIAN_FILTER_SIZE, TIM_CAL_CENTER_AVERAGE_SIZE, mco_pulse_count);
    MATH_exit_error(TIM_ERROR_BASE_MATH);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_PWM) != 0)
/*******************************************************************/
TIM_status_t TIM_PWM_init(TIM_instance_t instance, TIM_gpio_t* pins) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    TIM_channel_t channel = 0;
    uint32_t reg_value = 0;
    uint8_t idx = 0;
    // Check instance, mode and GPIOs.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_NONE);
    _TIM_check_gpio();
    // Check supported instance.
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    if (instance == TIM_INSTANCE_TIM6) {
        status = TIM_ERROR_INSTANCE_NOT_SUPPORTED;
        goto errors;
    }
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    if (instance == TIM_INSTANCE_TIM7) {
        status = TIM_ERROR_INSTANCE_NOT_SUPPORTED;
        goto errors;
    }
#endif
    // Reset peripheral.
    _TIM_reset(instance);
    // Update local interrupt handler.
    tim_ctx[instance].irq_handler = NULL;
    // Init context.
    tim_pwm_ctx[instance].channels_duty_cycle = 0;
    // Enable peripheral clock.
    (*TIM_DESCRIPTOR[instance].rcc_enr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    (*TIM_DESCRIPTOR[instance].rcc_smenr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    // Enable preload.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 7); // ARPE='1'.
    // Set default ARR.
    reg_value = ((TIM_DESCRIPTOR[instance].peripheral->ARR) & (~TIM_REGISTER_MASK_ARR_PSC_CCR));
    reg_value |= (TIM_ARR_VALUE_MAX - 1);
    TIM_DESCRIPTOR[instance].peripheral->ARR = reg_value;
    // Set trigger selection to reserved value to ensure there is no link with other timers.
    TIM_DESCRIPTOR[instance].peripheral->SMCR |= (0b011 << 4);
    // Configure channels.
    for (idx = 0; idx < (pins->list_size); idx++) {
        // Check channel.
        channel = (pins->list[idx]->channel);
        _TIM_check_channel(instance, channel);
        // Use PWM mode 2 with preload (OCxM='111' and OCxPE='1').
        TIM_DESCRIPTOR[instance].peripheral->CCMRx[channel >> 1] |= (0b1111 << (((channel % 2) << 3) + 3));
        // Set polarity.
        if ((pins->list[idx]->polarity) == TIM_POLARITY_ACTIVE_LOW) {
            TIM_DESCRIPTOR[instance].peripheral->CCER |= (0b1 << ((channel << 2) + 1));
        }
        // Disable output by default.
        TIM_DESCRIPTOR[instance].peripheral->CCRx[channel] |= TIM_REGISTER_MASK_ARR_PSC_CCR;
        // Generate event to update registers.
        TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
        // Enable channel.
        TIM_DESCRIPTOR[instance].peripheral->CCER |= (0b1 << (channel << 2));
        // Init GPIO.
        GPIO_configure((pins->list[idx]->gpio), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
    }
    // Update mode.
    tim_ctx[instance].mode = TIM_MODE_PWM;
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_PWM) != 0)
/*******************************************************************/
TIM_status_t TIM_PWM_de_init(TIM_instance_t instance, TIM_gpio_t* pins) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint8_t idx = 0;
    // Check instance and GPIOs.
    _TIM_check_instance(instance);
    _TIM_check_gpio();
    // Release GPIOs.
    for (idx = 0; idx < (pins->list_size); idx++) {
        GPIO_write((pins->list[idx]->gpio), ((pins->list[idx]->polarity) == TIM_POLARITY_ACTIVE_LOW) ? 1 : 0);
        GPIO_configure((pins->list[idx]->gpio), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    }
    // Release peripheral.
    _TIM_de_init(instance);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_PWM) != 0)
/*******************************************************************/
TIM_status_t TIM_PWM_set_waveform(TIM_instance_t instance, TIM_channel_t channel, uint32_t frequency_mhz, uint8_t duty_cycle_percent) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint32_t tim_clock_hz = 0;
    uint32_t arr = 0;
    uint64_t period_ns = 0;
    uint32_t reg_value = 0;
    // Check instance, mode and channel.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_PWM);
    _TIM_check_channel(instance, channel);
    // Check parameters.
    if (frequency_mhz == 0) {
        status = TIM_ERROR_FREQUENCY;
        goto errors;
    }
    if (duty_cycle_percent > MATH_PERCENT_MAX) {
        status = TIM_ERROR_DUTY_CYCLE;
        goto errors;
    }
    // Get clock source frequency.
    RCC_get_frequency_hz(RCC_CLOCK_SYSTEM, &tim_clock_hz);
    // Disable update event during registers writing.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 1);
    // Compute PSC and ARR values.
    period_ns = ((uint64_t) 1000000000000);
    period_ns /= ((uint64_t) frequency_mhz);
    status = _TIM_compute_psc_arr(instance, tim_clock_hz, period_ns, &arr);
    if (status != TIM_SUCCESS) goto errors;
    // Set duty cycle.
    reg_value = (TIM_DESCRIPTOR[instance].peripheral->CCRx[channel] & (~TIM_REGISTER_MASK_ARR_PSC_CCR));
    reg_value |= (((arr + 1) - (((arr + 1) * duty_cycle_percent) / (MATH_PERCENT_MAX))) & TIM_REGISTER_MASK_ARR_PSC_CCR);
    TIM_DESCRIPTOR[instance].peripheral->CCRx[channel] = reg_value;
    // Re-enable update event.
    TIM_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 1);
    // Update duty cycle word.
    tim_pwm_ctx[instance].channels_duty_cycle &= ~(MATH_U8_MASK << (channel << 3));
    tim_pwm_ctx[instance].channels_duty_cycle |= (duty_cycle_percent << (channel << 3));
    // Check timer status.
    if (((TIM_DESCRIPTOR[instance].peripheral->CR1) & (0b1 << 0)) == 0) {
        // Disable one pulse mode.
        TIM_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 3); // OPM='0'.
        // Generate event to update PWM settings directly and start counter.
        TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
        TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 0); // CEN='1'.
    }
    else {
        if (tim_pwm_ctx[instance].channels_duty_cycle == 0) {
            // Enable one pulse mode to stop counter automatically at the end of the last period.
            TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 3); // OPM='1'.
        }
    }
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*******************************************************************/
TIM_status_t TIM_OPM_init(TIM_instance_t instance, TIM_gpio_t* pins) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    TIM_channel_t channel = 0;
    uint32_t reg_value = 0;
    uint8_t idx = 0;
    // Check instance, mode and GPIOs.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_NONE);
    _TIM_check_gpio();
    // Check supported instance.
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    if (instance == TIM_INSTANCE_TIM6) {
        status = TIM_ERROR_INSTANCE_NOT_SUPPORTED;
        goto errors;
    }
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
    if (instance == TIM_INSTANCE_TIM7) {
        status = TIM_ERROR_INSTANCE_NOT_SUPPORTED;
        goto errors;
    }
#endif
    // Reset peripheral.
    _TIM_reset(instance);
    // Update local interrupt handler.
    tim_ctx[instance].irq_handler = &_TIM_OPM_irq_handler;
    // Enable peripheral clock.
    (*TIM_DESCRIPTOR[instance].rcc_enr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    (*TIM_DESCRIPTOR[instance].rcc_smenr) |= TIM_DESCRIPTOR[instance].rcc_mask;
    // Enable one pulse mode and preload.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 7) | (0b1 << 3); // ARPE='1' and OPM='1'.
    // Set default ARR.
    reg_value = ((TIM_DESCRIPTOR[instance].peripheral->ARR) & (~TIM_REGISTER_MASK_ARR_PSC_CCR));
    reg_value |= (TIM_ARR_VALUE_MAX - 1);
    TIM_DESCRIPTOR[instance].peripheral->ARR = reg_value;
    // Set trigger selection to reserved value to ensure there is no link with other timers.
    TIM_DESCRIPTOR[instance].peripheral->SMCR |= (0b011 << 4);
    // Configure channels.
    for (idx = 0; idx < (pins->list_size); idx++) {
        // Check channel.
        channel = (pins->list[idx]->channel);
        _TIM_check_channel(instance, channel);
        // Use PWM mode 2 with preload (OCxM='111', OCxPE='1' and OCxFE='1').
        TIM_DESCRIPTOR[instance].peripheral->CCMRx[channel >> 1] |= (0b11111 << (((channel % 2) << 3) + 2));
        // Set polarity.
        if ((pins->list[idx]->polarity) == TIM_POLARITY_ACTIVE_LOW) {
            TIM_DESCRIPTOR[instance].peripheral->CCER |= (0b1 << ((channel << 2) + 1));
        }
        // Disable output by default.
        TIM_DESCRIPTOR[instance].peripheral->CCRx[channel] |= TIM_REGISTER_MASK_ARR_PSC_CCR;
        // Generate event to update registers.
        TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
        // Enable channel.
        TIM_DESCRIPTOR[instance].peripheral->CCER |= (0b1 << (channel << 2));
        // Init GPIO.
        GPIO_configure((pins->list[idx]->gpio), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
    }
    // Update mode.
    tim_ctx[instance].mode = TIM_MODE_OPM;
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*******************************************************************/
TIM_status_t TIM_OPM_de_init(TIM_instance_t instance, TIM_gpio_t* pins) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint8_t idx = 0;
    // Check instance and GPIOs.
    _TIM_check_instance(instance);
    _TIM_check_gpio();
    // Release GPIOs.
    for (idx = 0; idx < (pins->list_size); idx++) {
        GPIO_write((pins->list[idx]->gpio), ((pins->list[idx]->polarity) == TIM_POLARITY_ACTIVE_LOW) ? 1 : 0);
        GPIO_configure((pins->list[idx]->gpio), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    }
    // Release peripheral.
    _TIM_de_init(instance);
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*******************************************************************/
TIM_status_t TIM_OPM_make_pulse(TIM_instance_t instance, uint8_t channels_mask, uint32_t delay_ns, uint32_t pulse_duration_ns, uint8_t internal_irq_enable) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    uint32_t tim_clock_hz = 0;
    uint32_t arr = 0;
    uint64_t ccr = 0;
    uint32_t reg_value = 0;
    uint8_t idx = 0;
    // Check instance and mode.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_OPM);
    // Directly exit if there is mask is null.
    if (channels_mask == 0) goto errors;
    // Check parameters.
    if ((delay_ns + pulse_duration_ns) == 0) {
        status = TIM_ERROR_PULSE;
        goto errors;
    }
    // Get clock source frequency.
    RCC_get_frequency_hz(RCC_CLOCK_SYSTEM, &tim_clock_hz);
    // Disable update event during registers writing.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 1);
    // Configure interrupt.
    if (internal_irq_enable != 0) {
        TIM_DESCRIPTOR[instance].peripheral->DIER |= (0b1 << 0);
        NVIC_enable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    }
    else {
        TIM_DESCRIPTOR[instance].peripheral->DIER &= ~(0b1 << 0);
        NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
    }
    // Compute PSC and ARR values.
    status = _TIM_compute_psc_arr(instance, tim_clock_hz, ((uint64_t) delay_ns + (uint64_t) pulse_duration_ns), &arr);
    if (status != TIM_SUCCESS) goto errors;
    // Compute CCR value.
    ccr = (((uint64_t) delay_ns) * ((uint64_t) (arr + 1)));
    ccr /= (((uint64_t) delay_ns) + ((uint64_t) pulse_duration_ns));
    // Channels loop.
    for (idx = 0; idx < TIM_DESCRIPTOR[instance].number_of_channels; idx++) {
        // Check mask.
        if ((channels_mask & (0b1 << idx)) != 0) {
            // Write register.
            reg_value = (TIM_DESCRIPTOR[instance].peripheral->CCRx[idx] & (~TIM_REGISTER_MASK_ARR_PSC_CCR));
            reg_value |= (((ccr == 0) ? 1 : ccr) & TIM_REGISTER_MASK_ARR_PSC_CCR);
            TIM_DESCRIPTOR[instance].peripheral->CCRx[idx] = reg_value;
        }
        else {
            // Disable channel.
            TIM_DESCRIPTOR[instance].peripheral->CCRx[idx] |= TIM_REGISTER_MASK_ARR_PSC_CCR;
        }
    }
    // Re-enable update event.
    TIM_DESCRIPTOR[instance].peripheral->CR1 &= ~(0b1 << 1);
    // Generate event to update registers.
    TIM_DESCRIPTOR[instance].peripheral->EGR |= (0b1 << 0); // UG='1'.
    // Start channel and timer.
    TIM_DESCRIPTOR[instance].peripheral->CR1 |= (0b1 << 0); // CEN='1'.
errors:
    return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & TIM_MODE_MASK_OPM) != 0)
/*******************************************************************/
TIM_status_t TIM_OPM_get_pulse_status(TIM_instance_t instance, uint8_t* pulse_is_done) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance, mode and channel.
    _TIM_check_instance(instance);
    _TIM_check_mode(instance, TIM_MODE_OPM);
    // Check parameter.
    if (pulse_is_done == NULL) {
        status = TIM_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Update status.
    (*pulse_is_done) = (((TIM_DESCRIPTOR[instance].peripheral->CR1) & (0b1 << 0)) == 0) ? 1 : 0;
errors:
    return status;
}
#endif

/*******************************************************************/
TIM_status_t TIM_get_ccr_register_address(TIM_instance_t instance, TIM_channel_t channel, uint32_t* ccr_register_address) {
    // Local variables.
    TIM_status_t status = TIM_SUCCESS;
    // Check instance and channel.
    _TIM_check_instance(instance);
    _TIM_check_channel(instance, channel);
    // Update address.
    (*ccr_register_address) = ((uint32_t) &(TIM_DESCRIPTOR[instance].peripheral->CCRx[channel]));
errors:
    return status;
}

#endif /* STM32L0XX_DRIVERS_TIM_MODE_MASK */

#endif /* STM32L0XX_DRIVERS_DISABLE */
