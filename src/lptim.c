/*
 * lptim.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#include "lptim.h"

#include "exti.h"
#include "iwdg.h"
#include "lptim_registers.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc.h"
#include "rcc_registers.h"
#include "types.h"

/*** LPTIM local macros ***/

#define LPTIM_TIMEOUT_COUNT     1000000

#define LPTIM_ARR_MASK          0x0000FFFF
#define LPTIM_ARR_MAX_VALUE     LPTIM_ARR_MASK

#define LPTIM_DELAY_MS_MIN      2
#define LPTIM_DELAY_MS_MAX      ((LPTIM_ARR_MAX_VALUE * 1000) / (lptim_ctx.clock_frequency_hz))

#define LPTIM_CNT_MASK          0x0000FFFF

/*** LPTIM local structures ***/

/*******************************************************************/
typedef union {
    struct {
        unsigned wake_up :1;
        unsigned running :1;
        unsigned init : 1;
    };
    uint8_t all;
} LPTIM_flags_t;

/*******************************************************************/
typedef struct {
    RCC_clock_t clock_source;
    uint32_t clock_frequency_hz;
    volatile LPTIM_flags_t flags;
} LPTIM_context_t;

/*** LPTIM local global variables ***/

static LPTIM_context_t lptim_ctx = {
    .clock_source = RCC_CLOCK_NONE,
    .clock_frequency_hz = 0,
    .flags.all = 0
};

/*** LPTIM local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) LPTIM1_IRQHandler(void) {
    // Check flag.
    if (((LPTIM1->ISR) & (0b1 << 1)) != 0) {
        // Set local flag.
        if (((LPTIM1->IER) & (0b1 << 1)) != 0) {
            lptim_ctx.flags.wake_up = 1;
        }
        // Clear flag.
        LPTIM1->ICR = (0b1 << 1);
    }
    EXTI_clear_line_flag(EXTI_LINE_LPTIM1);
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _LPTIM_reset(void) {
    // Local variables.
    uint8_t count = 0;
    // Perform manual reset and delay.
    RCC->APB1RSTR |= (0b1 << 31);
    for (count = 0; count < 100; count++);
    RCC->APB1RSTR &= ~(0b1 << 31);
    // Disable peripheral clock.
    RCC->APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
    // Force APB clock at the end of delay.
    RCC->CCIPR &= ~(0b11 << 18); // LPTIM1SEL='00'.
    // Reset running flag.
    lptim_ctx.flags.running = 0;
}

/*******************************************************************/
static LPTIM_status_t _LPTIM_init(LPTIM_clock_prescaler_t prescaler, uint32_t arr_value) {
    // Local variables.
    LPTIM_status_t status = LPTIM_SUCCESS;
    uint32_t loop_count = 0;
    uint32_t arr = 0;
    // Force APB clock to access registers.
    RCC->CCIPR &= ~(0b11 << 18); // LPTIM1SEL='00'.
    // Enable peripheral clock.
    RCC->APB1ENR |= (0b1 << 31); // LPTIM1EN='1'.
    // Configure prescaler.
    LPTIM1->CFGR |= (prescaler << 9);
    // Reset flags.
    LPTIM1->ICR = ((0b1 << 4) | (0b1 << 1));
    // Enable peripheral.
    LPTIM1->CR |= (0b1 << 0); // ENABLE='1'.
    // Write ARR.
    arr = ((LPTIM1->ARR) & ~(LPTIM_ARR_MASK));
    arr |= (arr_value & LPTIM_ARR_MASK);
    LPTIM1->ARR = arr;
    // Wait for ARR write operation to complete.
    while (((LPTIM1->ISR) & (0b1 << 4)) == 0) {
        loop_count++;
        if (loop_count > LPTIM_TIMEOUT_COUNT) {
            status = LPTIM_ERROR_ARR_TIMEOUT;
            goto errors;
        }
    }
    // Select clock source.
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
    RCC->CCIPR |= (0b01 << 18);
#elif (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    switch (lptim_ctx.clock_source) {
    case RCC_CLOCK_LSE:
        RCC->CCIPR |= (0b11 << 18);
        break;
    case RCC_CLOCK_LSI:
        RCC->CCIPR |= (0b01 << 18);
        break;
    default:
        status = LPTIM_ERROR_CLOCK_SOURCE;
        goto errors;
    }
#else
    RCC->CCIPR |= (0b11 << 18);
#endif
errors:
    return status;
}

/*** LPTIM functions ***/

/*******************************************************************/
LPTIM_status_t LPTIM_init(uint8_t nvic_priority) {
    // Local variables.
    LPTIM_status_t status = LPTIM_SUCCESS;
    uint32_t lptim_clock_hz = 0;
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    uint8_t lse_status = 0;
#endif
    // Check state.
    if (lptim_ctx.flags.init != 0) {
        status = LPTIM_ERROR_ALREADY_INITIALIZED;
        goto errors;
    }
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
    // Use LSI.
    lptim_ctx.clock_source = RCC_CLOCK_LSI;
#elif (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    // Get LSE status.
    RCC_get_status(RCC_CLOCK_LSE, &lse_status);
    // Update clock source.
    lptim_ctx.clock_source = (lse_status != 0) ? RCC_CLOCK_LSE : RCC_CLOCK_LSI;
#else
    // Use LSE.
    lptim_ctx.clock_source = RCC_CLOCK_LSE;
#endif
    // Get clock source frequency.
    RCC_get_frequency_hz(lptim_ctx.clock_source, &lptim_clock_hz);
    lptim_ctx.clock_frequency_hz = (lptim_clock_hz >> 3);
    // Reset flags.
    lptim_ctx.flags.all = 0;
    // Set interrupt priority.
    NVIC_set_priority(NVIC_INTERRUPT_LPTIM1, nvic_priority);
    // Enable LPTIM EXTI line.
    EXTI_enable_line(EXTI_LINE_LPTIM1, EXTI_TRIGGER_RISING_EDGE);
    // Update initialization flag.
    lptim_ctx.flags.init = 1;
errors:
    return status;
}

/*******************************************************************/
LPTIM_status_t LPTIM_de_init(void) {
    // Local variables.
    LPTIM_status_t status = LPTIM_SUCCESS;
    // Check state.
    if (lptim_ctx.flags.init == 0) {
        status = LPTIM_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Disable peripheral clock.
    RCC->APB1ENR &= ~(0b1 << 31); // LPTIM1EN='0'.
    // Update initialization flag.
    lptim_ctx.flags.init = 0;
errors:
    return status;
}

/*******************************************************************/
LPTIM_status_t LPTIM_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode) {
    // Local variables.
    LPTIM_status_t status = LPTIM_SUCCESS;
    // Check state.
    if (lptim_ctx.flags.init == 0) {
        status = LPTIM_ERROR_UNINITIALIZED;
        goto errors;
    }
    // Check delay.
    if ((delay_ms > LPTIM_DELAY_MS_MAX) || (delay_ms > (IWDG_FREE_DELAY_SECONDS_MAX * 1000))) {
        status = LPTIM_ERROR_DELAY_OVERFLOW;
        goto errors;
    }
    if (delay_ms < LPTIM_DELAY_MS_MIN) {
        status = LPTIM_ERROR_DELAY_UNDERFLOW;
        goto errors;
    }
    // Check if delay is not already running.
    if (lptim_ctx.flags.running != 0) {
        status = LPTIM_ERROR_ALREADY_RUNNING;
        goto end;
    }
    // Init peripheral.
    status = _LPTIM_init(LPTIM_CLOCK_PRESCALER_8, (((delay_ms - 1) * lptim_ctx.clock_frequency_hz) / (1000)));
    if (status != LPTIM_SUCCESS) goto errors;
    // Enable interrupt.
    LPTIM1->IER |= (0b1 << 1); // ARRMIE='1'.
    // Set running flag.
    lptim_ctx.flags.running = 1;
    // Clear wake-up flag.
    lptim_ctx.flags.wake_up = 0;
    // Start timer.
    LPTIM1->CR |= (0b1 << 1); // SNGSTRT='1'.
    // Perform delay with the selected waiting mode.
    switch (delay_mode) {
    case LPTIM_DELAY_MODE_ACTIVE:
        // Active loop.
        while (((LPTIM1->ISR) & (0b1 << 1)) == 0);
        break;
    case LPTIM_DELAY_MODE_SLEEP:
        // Enable interrupt.
        NVIC_enable_interrupt(NVIC_INTERRUPT_LPTIM1);
        // Enter sleep mode.
        while (lptim_ctx.flags.wake_up == 0) {
            PWR_enter_sleep_mode(PWR_SLEEP_MODE_NORMAL);
        }
        // Disable interrupt.
        NVIC_disable_interrupt(NVIC_INTERRUPT_LPTIM1);
        break;
    case LPTIM_DELAY_MODE_STOP:
        // Enable interrupt.
        NVIC_enable_interrupt(NVIC_INTERRUPT_LPTIM1);
        // Enter stop mode.
        while (lptim_ctx.flags.wake_up == 0) {
            PWR_enter_deepsleep_mode(PWR_DEEPSLEEP_MODE_STOP);
        }
        // Disable interrupt.
        NVIC_disable_interrupt(NVIC_INTERRUPT_LPTIM1);
        break;
    default:
        status = LPTIM_ERROR_DELAY_MODE;
        goto errors;
    }
errors:
    // Reset peripheral.
    _LPTIM_reset();
end:
    return status;
}

/*******************************************************************/
LPTIM_status_t LPTIM_start(LPTIM_clock_prescaler_t prescaler) {
    // Local variables.
    LPTIM_status_t status = LPTIM_SUCCESS;
    // Check if delay is not already running.
    if (lptim_ctx.flags.running != 0) {
        status = LPTIM_ERROR_ALREADY_RUNNING;
        goto errors;
    }
    // Check parameters.
    if (prescaler >= LPTIM_CLOCK_PRESCALER_LAST) {
       status = LPTIM_ERROR_CLOCK_PRESCALER;
       goto errors;
    }
    // Init peripheral.
    status = _LPTIM_init(prescaler, LPTIM_ARR_MAX_VALUE);
    if (status != LPTIM_SUCCESS) goto errors;
    // Set running flag.
    lptim_ctx.flags.running = 1;
    // Start timer.
    LPTIM1->CR |= (0b1 << 2); // CNTSTRT='1'.
errors:
    return status;
}

/*******************************************************************/
LPTIM_status_t LPTIM_stop(void) {
    // Local variables.
    LPTIM_status_t status = LPTIM_SUCCESS;
    // Stop timer.
    LPTIM1->CR &= ~(0b1 << 0); // ENABLE='0'.
    // Reset peripheral.
    _LPTIM_reset();
    return status;
}

/*******************************************************************/
uint32_t LPTIM_get_counter(void) {
    return ((LPTIM1->CNT) & LPTIM_CNT_MASK);
}

#endif /* STM32L0XX_DRIVERS_DISABLE */
