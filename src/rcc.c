/*
 * rcc.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "rcc.h"

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "error.h"
#include "gpio.h"
#include "flash.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "tim.h"
#include "types.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT                   1000000

#define RCC_LSI_FREQUENCY_TYPICAL_HZ        38000
#define RCC_LSI_FREQUENCY_ACCURACY_PERCENT  48

#define RCC_MSI_FREQUENCY_ACCURACY_PERCENT  4

#define RCC_HSI_FREQUENCY_TYPICAL_HZ        16000000
#define RCC_HSI_FREQUENCY_ACCURACY_PERCENT  6

/*** RCC local structures ***/

/*******************************************************************/
typedef struct {
    RCC_clock_t source;
    RCC_msi_range_t msi_range;
#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
    RCC_hse_mode_t hse_mode;
#endif
} RCC_system_clock_t;

/*******************************************************************/
typedef struct {
    RCC_system_clock_t current_sysclk;
    RCC_system_clock_t previous_sysclk;
    uint32_t clock_frequency[RCC_CLOCK_LAST];
    uint8_t hsi_stop_mode_request_count;
} RCC_context_t;

/*** RCC local global variables ***/

static const uint32_t RCC_MSI_FREQUENCY_TYPICAL[RCC_MSI_RANGE_LAST] = { 65536, 131072, 262144, 524288, 1048000, 2097000, 4194000 };
static RCC_context_t rcc_ctx;

/*** RCC local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) RCC_IRQHandler(void) {
    // Clear flags.
    RCC->CICR |= (0b11 << 0);
}

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _RCC_reset_backup_domain(void) {
    // Local variables.
    uint8_t count = 0;
    // Perform manual reset and delay.
    RCC->CSR |= (0b1 << 19); // RTCRST='1'.
    for (count = 0; count < 100; count++);
    RCC->CSR &= ~(0b1 << 19); // RTCRST='0'.
}

/*******************************************************************/
void _RCC_enable_lsi(void) {
    // Enable LSI.
    RCC->CSR |= (0b1 << 0); // LSION='1'.
    // Enable interrupt.
    RCC->CIER |= (0b1 << 0);
    NVIC_enable_interrupt(NVIC_INTERRUPT_RCC_CRS);
    // Wait for LSI to be stable.
    while (((RCC->CSR) & (0b1 << 1)) == 0) {
        PWR_enter_sleep_mode();
    }
    NVIC_disable_interrupt(NVIC_INTERRUPT_RCC_CRS);
}

#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
/*******************************************************************/
RCC_status_t _RCC_enable_lse(void) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint32_t loop_count = 0;
    // Enable LSE (32.768kHz crystal).
    RCC->CSR |= (0b1 << 8); // LSEON='1'.
    // Wait for LSE to be stable.
    while (((RCC->CSR) & (0b1 << 9)) == 0) {
        // Wait for LSERDY='1' ready flag or timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            // Switch LSE off.
            RCC->CSR &= ~(0b1 << 8); // LSEON='0'.
            // Exit loop.
            status = RCC_ERROR_LSE_READY;
            goto errors;
        }
    }
errors:
    return status;
}
#endif

#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 2)
/*******************************************************************/
void _RCC_enable_lse(void) {
    // Enable LSE (32.768kHz crystal).
    RCC->CSR |= (0b1 << 8); // LSEON='1'.
    // Enable interrupt.
    RCC->CIER |= (0b1 << 1);
    NVIC_enable_interrupt(NVIC_INTERRUPT_RCC_CRS);
    // Wait for LSE to be stable.
    while (((RCC->CSR) & (0b1 << 9)) == 0) {
        PWR_enter_sleep_mode();
    }
    NVIC_disable_interrupt(NVIC_INTERRUPT_RCC_CRS);
}
#endif

/*******************************************************************/
static void _RCC_save_system_clock(void) {
    // Copy current settings in previous structure.
    rcc_ctx.previous_sysclk.source = rcc_ctx.current_sysclk.source;
    rcc_ctx.previous_sysclk.msi_range = rcc_ctx.current_sysclk.msi_range;
#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
    rcc_ctx.previous_sysclk.hse_mode = rcc_ctx.current_sysclk.hse_mode;
#endif
}

/*******************************************************************/
static RCC_status_t _RCC_wait_for_clock_ready(RCC_clock_t clock, RCC_status_t timeout_error) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint8_t clock_is_ready = 0;
    uint32_t loop_count = 0;
    // Wait for clock to be stable.
    do {
        // Read status.
        status = RCC_get_status(clock, &clock_is_ready);
        if (status != RCC_SUCCESS) goto errors;
        // Exit if timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = timeout_error;
            goto errors;
        }
    }
    while (clock_is_ready == 0);
errors:
    return status;
}

/*******************************************************************/
static RCC_status_t _RCC_switch_system_clock(RCC_clock_t system_clock, RCC_status_t switch_error) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint32_t reg_cfgr = 0;
    uint32_t switch_value = 0;
    uint32_t loop_count = 0;
    // Read register.
    reg_cfgr = (RCC->CFGR);
    reg_cfgr &= ~(0b11 << 0); // Reset bits 0-1.
    // Check system clock.
    switch (system_clock) {
    case RCC_CLOCK_MSI:
        switch_value = 0b00;
        break;
    case RCC_CLOCK_HSI:
        switch_value = 0b01;
        break;
    case RCC_CLOCK_HSE:
        switch_value = 0b10;
        break;
    case RCC_CLOCK_PLL:
        switch_value = 0b11;
        break;
    default:
        status = RCC_ERROR_CLOCK;
        goto errors;
    }
    reg_cfgr |= (switch_value << 0);
    // Perform switch.
    RCC->CFGR = reg_cfgr;
    // Wait for clock switch.
    while (((RCC->CFGR) & (0b11 << 2)) != (switch_value << 2)) {
        // Wait for SWS='switch_value' or timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = switch_error;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
RCC_status_t _RCC_check_frequency_range(uint32_t default_value, uint32_t accuracy_percent, uint32_t measured_value, RCC_status_t calibration_error) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint32_t frequency_min = (default_value - ((default_value * accuracy_percent) / (100)));
    uint32_t frequency_max = (default_value + ((default_value * accuracy_percent) / (100)));
    // Check range.
    if ((measured_value < frequency_min) || (measured_value > frequency_max)) {
        status = calibration_error;
    }
    return status;
}

/*** RCC functions ***/

/*******************************************************************/
RCC_status_t RCC_init(uint8_t nvic_priority) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Set boot configuration.
    rcc_ctx.current_sysclk.source = RCC_CLOCK_MSI;
    rcc_ctx.current_sysclk.msi_range = RCC_MSI_RANGE_5_2MHZ;
#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
    rcc_ctx.current_sysclk.hse_mode = RCC_HSE_MODE_OSCILLATOR;
#endif
    _RCC_save_system_clock();
    // Set default frequencies.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = RCC_HSI_FREQUENCY_TYPICAL_HZ;
    rcc_ctx.clock_frequency[RCC_CLOCK_MSI] = RCC_MSI_FREQUENCY_TYPICAL[rcc_ctx.current_sysclk.msi_range];
    rcc_ctx.clock_frequency[RCC_CLOCK_HSE] = STM32L0XX_DRIVERS_RCC_HSE_FREQUENCY_HZ;
    rcc_ctx.clock_frequency[RCC_CLOCK_PLL] = 0;
    rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = RCC_LSI_FREQUENCY_TYPICAL_HZ;
    rcc_ctx.clock_frequency[RCC_CLOCK_LSE] = STM32L0XX_DRIVERS_RCC_LSE_FREQUENCY_HZ;
    rcc_ctx.hsi_stop_mode_request_count = 0;
    // Reset backup domain.
    _RCC_reset_backup_domain();
    // Set interrupt priority.
    NVIC_set_priority(NVIC_INTERRUPT_RCC_CRS, nvic_priority);
    // Start low speed oscillators.
    _RCC_enable_lsi();
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
    status = _RCC_enable_lse();
#endif
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 2)
    _RCC_enable_lse();
#endif
    return status;
}

/*******************************************************************/
RCC_status_t RCC_switch_to_hsi(void) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    FLASH_status_t flash_status = FLASH_SUCCESS;
    // Save current configuration.
    _RCC_save_system_clock();
    // Set flash latency.
    flash_status = FLASH_set_latency(1);
    FLASH_exit_error(RCC_ERROR_BASE_FLASH);
    // Enable HSI.
    RCC->CR |= (0b1 << 0); // HSI16ON='1'.
    // Wait for HSI to be stable.
    status = _RCC_wait_for_clock_ready(RCC_CLOCK_HSI, RCC_ERROR_HSI_READY);
    if (status != RCC_SUCCESS) goto errors;
    // Switch system clock.
    status = _RCC_switch_system_clock(RCC_CLOCK_HSI, RCC_ERROR_HSI_SWITCH);
    if (status != RCC_SUCCESS) goto errors;
    // Update clocks context.
    rcc_ctx.current_sysclk.source = RCC_CLOCK_HSI;
errors:
    // Update system clock frequency.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    return status;
}

#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
/*******************************************************************/
RCC_status_t RCC_switch_to_hse(RCC_hse_mode_t hse_mode) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    FLASH_status_t flash_status = FLASH_SUCCESS;
    // Save current configuration.
    _RCC_save_system_clock();
    // Set flash latency.
    flash_status = FLASH_set_latency(1);
    FLASH_exit_error(RCC_ERROR_BASE_FLASH);
    // Set mode.
    if (hse_mode == RCC_HSE_MODE_BYPASS) {
        RCC->CR |= (0b1 << 18); // HSEBYP='1'.
    }
    // Enable HSE.
    RCC->CR |= (0b1 << 16); // HSEON='1'.
    // Wait for HSI to be stable.
    status = _RCC_wait_for_clock_ready(RCC_CLOCK_HSE, RCC_ERROR_HSE_READY);
    if (status != RCC_SUCCESS) goto errors;
    // Switch system clock.
    status = _RCC_switch_system_clock(RCC_CLOCK_HSE, RCC_ERROR_HSE_SWITCH);
    if (status != RCC_SUCCESS) goto errors;
    // Update clocks context.
    rcc_ctx.current_sysclk.source = RCC_CLOCK_HSE;
    rcc_ctx.current_sysclk.hse_mode = hse_mode;
errors:
    // Update system clock frequency.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    return status;
}
#endif

/*******************************************************************/
RCC_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    FLASH_status_t flash_status = FLASH_SUCCESS;
    // Check parameter.
    if (msi_range >= RCC_MSI_RANGE_LAST) {
        status = RCC_ERROR_MSI_RANGE;
        goto errors;
    }
    // Save current configuration.
    _RCC_save_system_clock();
    // Set MSI range.
    RCC->ICSCR &= ~(0b111 << 13);
    RCC->ICSCR |= (msi_range << 13);
    // Update MSI frequency.
    rcc_ctx.current_sysclk.msi_range = msi_range;
    rcc_ctx.clock_frequency[RCC_CLOCK_MSI] = RCC_MSI_FREQUENCY_TYPICAL[msi_range];
    // Enable MSI.
    RCC->CR |= (0b1 << 8); // MSION='1'.
    // Wait for MSI to be stable.
    status = _RCC_wait_for_clock_ready(RCC_CLOCK_MSI, RCC_ERROR_MSI_READY);
    if (status != RCC_SUCCESS) goto errors;
    // Switch system clock.
    status = _RCC_switch_system_clock(RCC_CLOCK_MSI, RCC_ERROR_MSI_SWITCH);
    if (status != RCC_SUCCESS) goto errors;
    // Set flash latency.
    flash_status = FLASH_set_latency(0);
    FLASH_exit_error(RCC_ERROR_BASE_FLASH);
    // Update clocks context.
    rcc_ctx.current_sysclk.source = RCC_CLOCK_MSI;
errors:
    // Update system clock frequency.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    return status;
}

/*******************************************************************/
RCC_status_t RCC_restore_previous_system_clock(void) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Check previous configuration.
    switch (rcc_ctx.previous_sysclk.source) {
    case RCC_CLOCK_MSI:
        status = RCC_switch_to_msi(rcc_ctx.previous_sysclk.msi_range);
        break;
    case RCC_CLOCK_HSI:
        status = RCC_switch_to_hsi();
        break;
#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
    case RCC_CLOCK_HSE:
        status = RCC_switch_to_hse(rcc_ctx.previous_sysclk.hse_mode);
        break;
#endif
    default:
        status = RCC_ERROR_CLOCK;
        break;
    }
    return status;
}

/*******************************************************************/
void RCC_set_hsi_in_stop_mode(uint8_t enable) {
    // Check parameter.
    if (enable == 0) {
        // Update initialization count.
        if (rcc_ctx.hsi_stop_mode_request_count > 0) {
            rcc_ctx.hsi_stop_mode_request_count--;
        }
        // Check initialization count.
        if (rcc_ctx.hsi_stop_mode_request_count == 0) {
            RCC->CR &= ~(0b1 << 1); // HSI16KERON='0'.
        }
    }
    else {
        RCC->CR |= (0b1 << 1); // HSI16KERON='1'.
        // Update initialization count.
        rcc_ctx.hsi_stop_mode_request_count++;
    }
}

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*******************************************************************/
RCC_status_t RCC_calibrate_internal_clocks(uint8_t nvic_priority) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    TIM_status_t tim_status = TIM_SUCCESS;
    int32_t ref_clock_pulse_count = 0;
    int32_t mco_pulse_count = 0;
    uint64_t temp_u64 = 0;
    uint32_t clock_frequency_hz = 0;
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE > 0)
    uint8_t lse_status = 0;
#endif
#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
    uint8_t restore_done = 0;
#endif
    // Switch to HSI.
    status = RCC_switch_to_hsi();
    if (status != RCC_SUCCESS) goto errors;
    // Init measurement timer.
    tim_status = TIM_CAL_init(TIM_INSTANCE_TIM21, nvic_priority);
    if (tim_status != TIM_SUCCESS) {
        status = RCC_ERROR_CALIBRATION_TIMER;
        goto errors;
    }
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE > 0)
    // Check LSE status.
    RCC_get_status(RCC_CLOCK_LSE, &lse_status);
    // HSI calibration is not possible without LSE.
    if (lse_status == 0) goto lsi_calibration;
    // Connect MCO to LSE clock.
    RCC_set_mco(RCC_CLOCK_LSE, RCC_MCO_PRESCALER_1, NULL);
    // HSI calibration.
    tim_status = TIM_CAL_mco_capture(TIM_INSTANCE_TIM21, &ref_clock_pulse_count, &mco_pulse_count);
    if (tim_status != TIM_SUCCESS) {
        status = RCC_ERROR_CALIBRATION_TIMER;
        goto errors;
    }
    // Compute HSI frequency.
    temp_u64 = ((uint64_t) STM32L0XX_DRIVERS_RCC_LSE_FREQUENCY_HZ * (uint64_t) ref_clock_pulse_count);
    clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) mco_pulse_count));
    // Check range.
    status = _RCC_check_frequency_range(RCC_HSI_FREQUENCY_TYPICAL_HZ, RCC_HSI_FREQUENCY_ACCURACY_PERCENT, clock_frequency_hz, RCC_ERROR_CALIBRATION_HSI);
    if (status != RCC_SUCCESS) goto errors;
    // Store calibration value.
    rcc_ctx.clock_frequency[RCC_CLOCK_HSI] = clock_frequency_hz;
lsi_calibration:
#endif
    // LSI calibration.
#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
    // Check if HSE is available for better precision.
    if (rcc_ctx.previous_sysclk.source == RCC_CLOCK_HSE) {
        // Restore HSE.
        status = RCC_restore_previous_system_clock();
        if (status != RCC_SUCCESS) goto errors;
        // Update local flag.
        restore_done = 1;
    }
#endif
    // Connect MCO to LSI clock.
    RCC_set_mco(RCC_CLOCK_LSI, RCC_MCO_PRESCALER_1, NULL);
    // Perform measurement.
    tim_status = TIM_CAL_mco_capture(TIM_INSTANCE_TIM21, &ref_clock_pulse_count, &mco_pulse_count);
    if (tim_status != TIM_SUCCESS) {
        status = RCC_ERROR_CALIBRATION_TIMER;
        goto errors;
    }
    // Compute LSI frequency.
    temp_u64 = ((uint64_t) rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] * (uint64_t) mco_pulse_count);
    clock_frequency_hz = (uint32_t) ((temp_u64) / ((uint64_t) ref_clock_pulse_count));
    // Check range.
    status = _RCC_check_frequency_range(RCC_LSI_FREQUENCY_TYPICAL_HZ, RCC_LSI_FREQUENCY_ACCURACY_PERCENT, clock_frequency_hz, RCC_ERROR_CALIBRATION_LSI);
    if (status != RCC_SUCCESS) goto errors;
    // Update local data.
    rcc_ctx.clock_frequency[RCC_CLOCK_LSI] = clock_frequency_hz;
errors:
    // Release timer and MCO.
    TIM_CAL_de_init(TIM_INSTANCE_TIM21);
    RCC_set_mco(RCC_CLOCK_NONE, RCC_MCO_PRESCALER_1, NULL);
    // Restore system clock.
#ifdef STM32L0XX_DRIVERS_RCC_HSE_ENABLE
    if (restore_done == 0) {
        status = RCC_restore_previous_system_clock();
        if (status != RCC_SUCCESS) goto errors;
    }
#else
    status = RCC_restore_previous_system_clock();
    if (status != RCC_SUCCESS) goto errors;
#endif
    // Update system clock frequency.
    rcc_ctx.clock_frequency[RCC_CLOCK_SYSTEM] = rcc_ctx.clock_frequency[rcc_ctx.current_sysclk.source];
    return status;
}
#endif

/*******************************************************************/
RCC_clock_t RCC_get_system_clock(void) {
    return (rcc_ctx.current_sysclk.source);
}

/*******************************************************************/
RCC_status_t RCC_get_frequency_hz(RCC_clock_t clock, uint32_t* frequency_hz) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Check parameters.
    if (clock >= RCC_CLOCK_LAST) {
        status = RCC_ERROR_CLOCK;
        goto errors;
    }
    if (frequency_hz == NULL) {
        status = RCC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Read frequency.
    (*frequency_hz) = rcc_ctx.clock_frequency[clock];
errors:
    return status;
}

/*******************************************************************/
RCC_status_t RCC_get_status(RCC_clock_t clock, uint8_t* clock_is_ready) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    // Check parameters.
    if (clock_is_ready == NULL) {
        status = RCC_ERROR_NULL_PARAMETER;
        goto errors;
    }
    // Check clock.
    switch (clock) {
    case RCC_CLOCK_SYSTEM:
        (*clock_is_ready) = 1;
        break;
    case RCC_CLOCK_HSI:
        (*clock_is_ready) = (((RCC->CR) >> 2) & 0b1);
        break;
    case RCC_CLOCK_MSI:
        (*clock_is_ready) = (((RCC->CR) >> 9) & 0b1);
        break;
    case RCC_CLOCK_HSE:
        (*clock_is_ready) = (((RCC->CR) >> 17) & 0b1);
        break;
    case RCC_CLOCK_PLL:
        (*clock_is_ready) = (((RCC->CR) >> 25) & 0b1);
        break;
    case RCC_CLOCK_LSI:
        (*clock_is_ready) = (((RCC->CSR) >> 1) & 0b1);
        break;
    case RCC_CLOCK_LSE:
        (*clock_is_ready) = (((RCC->CSR) >> 9) & 0b1);
        break;
    default:
        status = RCC_ERROR_CLOCK;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
RCC_status_t RCC_set_mco(RCC_clock_t mco_clock, RCC_mco_prescaler_t mco_prescaler, const GPIO_pin_t* mco_gpio) {
    // Local variables.
    RCC_status_t status = RCC_SUCCESS;
    uint32_t cfgr = 0;
    // Check parameters.
    if (mco_clock >= RCC_CLOCK_LAST) {
        status = RCC_ERROR_CLOCK;
        goto errors;
    }
    if (mco_prescaler >= RCC_MCO_PRESCALER_LAST) {
        status = RCC_ERROR_MCO_PRESCALER;
        goto errors;
    }
    // Configure clock and prescaler.
    cfgr = (RCC->CFGR);
    cfgr &= 0x80FFFFFF;
    cfgr |= (mco_prescaler << 28) | (mco_clock << 24);
    RCC->CFGR = cfgr;
    // Configure GPIO if needed.
    if (mco_gpio != NULL) {
        // Check clock selection.
        if (mco_clock == RCC_CLOCK_NONE) {
            GPIO_configure(mco_gpio, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        }
        else {
            GPIO_configure(mco_gpio, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
        }
    }
errors:
    return status;
}
