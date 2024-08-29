/*
 * tim.c
 *
 *  Created on: 04 aug. 2024
 *      Author: Ludo
 */

#include "tim.h"

#ifndef STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_registers_flags.h"
#endif
#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "error.h"
#include "iwdg.h"
#include "nvic.h"
#include "pwr.h"
#include "rcc_reg.h"
#include "rtc.h"
#include "tim_reg.h"
#include "types.h"

/*** TIM local macros ***/

#define TIM_TIMEOUT_COUNT				10000000

#define TIM_ARR_VALUE_MIN				0x0001
#define TIM_ARR_VALUE_MAX				0xFFFF

#define TIM_CNT_VALUE_MAX				0xFFFF

#define TIM_TARGET_TRIGGER_CLOCK_HZ		2048

#define TIM_PRESCALER_ETRF_LSE			1
#define TIM_PRESCALER_PSC_LSE			((tim_clock_hz) / (TIM_TARGET_TRIGGER_CLOCK_HZ * TIM_PRESCALER_ETRF_LSE))

#define TIM_PRESCALER_ETRF_HSI			8
#define TIM_PRESCAKER_PSC_HSI			((tim_clock_hz) / (TIM_TARGET_TRIGGER_CLOCK_HZ * TIM_PRESCALER_ETRF_HSI))

#define TIM_CLOCK_SWITCH_LATENCY_MS		2

#define TIM_TIMER_DURATION_MS_MIN		1
#define TIM_TIMER_DURATION_MS_MAX		((TIM_CNT_VALUE_MAX * 1000) / (tim_ctx.multi_channel.etrf_clock_hz))

#define TIM_WATCHDOG_PERIOD_SECONDS		((TIM_TIMER_DURATION_MS_MAX / 1000) + 5)

#define TIM_INPUT_CAPTURE_PRESCALER		8

#define TIM_DUTY_CYCLE_PERCENT_MAX		100

/*** TIM local structures ***/

/*******************************************************************/
typedef struct {
	TIM_registers_t* peripheral;
	volatile uint32_t* rcc_enr;
	volatile uint32_t* rcc_smenr;
	uint32_t rcc_mask;
	NVIC_interrupt_t nvic_interrupt;
} TIM_descriptor_t;

/*******************************************************************/
typedef void (*TIM_irq_handler_cb_t)(TIM_instance_t instance, TIM_registers_t* peripheral);

/*******************************************************************/
typedef struct {
	RCC_clock_t clock_source;
	TIM_mode_t mode;
	TIM_irq_handler_cb_t irq_handler;
} TIM_common_t;

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
/*******************************************************************/
typedef struct {
	TIM_completion_irq_cb_t irq_callback;
} TIM_context_standard_t;
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
typedef struct {
	uint32_t duration_ms;
	TIM_waiting_mode_t waiting_mode;
	volatile uint8_t running_flag;
	volatile uint8_t irq_flag;
} TIM_channel_context_t;
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
typedef struct {
	uint32_t etrf_clock_hz;
	TIM_channel_context_t channel[TIM_CHANNEL_LAST];
} TIM_context_multi_channel_t;
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*******************************************************************/
typedef struct {
	volatile uint16_t ccr1_start;
	volatile uint16_t ccr1_end;
	volatile uint16_t capture_count;
	volatile uint8_t capture_done;
} TIM_context_calibration_t;
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
typedef struct {
	uint32_t channels_duty_cycle;
} TIM_context_pwm_t;
#endif

/*******************************************************************/
typedef struct {
	TIM_common_t common[TIM_INSTANCE_LAST];
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
	TIM_context_standard_t standard[TIM_INSTANCE_LAST];
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
	TIM_context_multi_channel_t multi_channel; // Not defined as array because only supported by one instance (TIM2).
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
	TIM_context_calibration_t calibration; // Not defined as array because only supported by one instance (TIM21).
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
	TIM_context_pwm_t pwm[TIM_INSTANCE_LAST];
#endif
} TIM_context_t;

/*** TIM local global variables ***/

static const TIM_descriptor_t TIM_DESCRIPTOR[TIM_INSTANCE_LAST] = {
	{TIM2,  &(RCC -> APB1SMENR), &(RCC -> APB1ENR), (0b1 << 0),  NVIC_INTERRUPT_TIM2},
	{TIM21, &(RCC -> APB2SMENR), &(RCC -> APB2ENR), (0b1 << 2),  NVIC_INTERRUPT_TIM21},
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 2) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	{TIM22, &(RCC -> APB2SMENR), &(RCC -> APB2ENR), (0b1 << 5),  NVIC_INTERRUPT_TIM22},
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	{TIM6,  &(RCC -> APB1SMENR), &(RCC -> APB1ENR), (0b1 << 4),  NVIC_INTERRUPT_TIM6},
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	{TIM3,  &(RCC -> APB1SMENR), &(RCC -> APB1ENR), (0b1 << 1),  NVIC_INTERRUPT_TIM3},
	{TIM7,  &(RCC -> APB1SMENR), &(RCC -> APB1ENR), (0b1 << 5),  NVIC_INTERRUPT_TIM7},
#endif
};
static TIM_context_t tim_ctx;

/*** TIM local functions ***/

/*******************************************************************/
#define _TIM_irq_handler(instance, peripheral) { \
	/* Execute internal callback */ \
	if (tim_ctx.common[instance].irq_handler != NULL) { \
		tim_ctx.common[instance].irq_handler(instance, peripheral); \
	} \
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM2_IRQHandler(void) {
	// Execute internal callback.
	_TIM_irq_handler(TIM_INSTANCE_TIM2, TIM2);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM21_IRQHandler(void) {
	// Execute internal callback.
	_TIM_irq_handler(TIM_INSTANCE_TIM21, TIM21);
}

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 2) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM22_IRQHandler(void) {
	// Execute internal callback.
	_TIM_irq_handler(TIM_INSTANCE_TIM22, TIM22);
}
#endif

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM6_IRQHandler(void) {
	// Execute internal callback.
	_TIM_irq_handler(TIM_INSTANCE_TIM6, TIM6);
}
#endif

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM3_IRQHandler(void) {
	// Execute internal callback.
	_TIM_irq_handler(TIM_INSTANCE_TIM3, TIM3);
}
#endif

#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM7_IRQHandler(void) {
	// Execute internal callback.
	_TIM_irq_handler(TIM_INSTANCE_TIM7, TIM7);
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _TIM_irq_handler_standard(TIM_instance_t instance, TIM_registers_t* peripheral) {
	// Update interrupt.
	if (((peripheral -> SR) & (0b1 << 0)) != 0) {
		// Call callback.
		if (tim_ctx.standard[instance].irq_callback != NULL) {
			tim_ctx.standard[instance].irq_callback();
		}
		// Clear flag.
		peripheral -> SR &= ~(0b1 << 0);
	}
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _TIM_irq_handler_multi_channel(TIM_instance_t instance, TIM_registers_t* peripheral) {
	// Local variables.
	uint8_t channel_idx = 0;
	uint8_t channel_mask = 0;
	// Unused parameter.
	UNUSED(instance);
	// Channels loop.
	for (channel_idx=0 ; channel_idx<TIM_CHANNEL_LAST ; channel_idx++) {
		// Compute mask.
		channel_mask = (0b1 << (channel_idx + 1));
		// Check flag.
		if (((peripheral -> SR) & channel_mask) != 0) {
			// Set local flag if channel is active.
			tim_ctx.multi_channel.channel[channel_idx].irq_flag = tim_ctx.multi_channel.channel[channel_idx].running_flag;
			// Clear flag.
			peripheral -> SR &= ~(channel_mask);
		}
	}
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*******************************************************************/
static void __attribute__((optimize("-O0"))) _TIM_irq_handler_calibration(TIM_instance_t instance, TIM_registers_t* peripheral) {
	// Unused parameter.
	UNUSED(instance);
	// TI1 interrupt.
	if (((peripheral -> SR) & (0b1 << 1)) != 0) {
		// Update flags.
		if (((peripheral -> DIER) & (0b1 << 1)) != 0) {
			// Check count.
			if (tim_ctx.calibration.capture_count == 0) {
				// Store start value.
				tim_ctx.calibration.ccr1_start = (peripheral -> CCR1);
				tim_ctx.calibration.capture_count++;
			}
			else {
				// Check rollover.
				if ((peripheral -> CCR1) > tim_ctx.calibration.ccr1_end) {
					// Store new value.
					tim_ctx.calibration.ccr1_end = (peripheral -> CCR1);
					tim_ctx.calibration.capture_count++;
				}
				else {
					// Capture complete.
					tim_ctx.calibration.capture_done = 1;
				}
			}
		}
		peripheral -> SR &= ~(0b1 << 1);
	}
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
static void _TIM_compute_compare_value(TIM_instance_t instance, TIM_channel_t channel) {
	// Update compare value.
	TIM_DESCRIPTOR[instance].peripheral -> CCRx[channel] = ((TIM_DESCRIPTOR[instance].peripheral -> CNT) + ((tim_ctx.multi_channel.channel[channel].duration_ms * tim_ctx.multi_channel.etrf_clock_hz) / (1000))) % TIM_CNT_VALUE_MAX;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
static TIM_status_t _TIM_internal_watchdog(uint32_t time_start, uint32_t* time_reference) {
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
	if (time > (time_start + TIM_WATCHDOG_PERIOD_SECONDS)) {
		status = TIM_ERROR_COMPLETION_WATCHDOG;
	}
	return status;
}
#endif

/*******************************************************************/
#define _TIM_check_channel(channel) { \
	/* Check channel */ \
	if (channel >= TIM_CHANNEL_LAST) { \
		status = TIM_ERROR_CHANNEL; \
		goto errors; \
	} \
}

/*** TIM functions ***/

/*******************************************************************/
TIM_status_t TIM_init(TIM_instance_t instance, TIM_gpio_t* pins, TIM_configuration_t* configuration) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint32_t tim_clock_hz = 0;
	uint64_t arr  = 0;
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x0A) != 0)
	uint8_t idx = 0;
#endif
#if (((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0) && (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1))
	uint8_t lse_status = 0;
#endif
	// Check instance.
	if (instance >= TIM_INSTANCE_LAST) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
	// Check parameters.
	if (configuration == NULL) {
		status = TIM_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset IRQ handler.
	tim_ctx.common[instance].irq_handler = NULL;
	// Check mode.
	switch (configuration -> mode) {
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
	case TIM_MODE_STANDARD:
		// Select clock source.
		tim_ctx.common[instance].clock_source = RCC_CLOCK_SYSTEM;
		break;
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
	case TIM_MODE_MULTI_CHANNEL:
		// Check supported instances.
		if (instance != TIM_INSTANCE_TIM2) {
			status = TIM_ERROR_INSTANCE;
			goto errors;
		}
		// Select clock source.
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
		// Use HSI as trigger.
		tim_ctx.common[instance].clock_source = RCC_CLOCK_HSI;
#elif (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
		// Get LSE status.
		RCC_get_status(RCC_CLOCK_LSE, &lse_status);
		// Select trigger.
		tim_ctx.common[instance].clock_source = (lse_status != 0) ? RCC_CLOCK_LSE : RCC_CLOCK_HSI;
#else
		// Use LSE as trigger.
		tim_ctx.common[instance].clock_source = RCC_CLOCK_LSE;
#endif
		break;
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
	case TIM_MODE_CALIBRATION:
		// Check supported instances.
		if (instance != TIM_INSTANCE_TIM21) {
			status = TIM_ERROR_INSTANCE;
			goto errors;
		}
		// Select clock source.
		tim_ctx.common[instance].clock_source = RCC_CLOCK_SYSTEM;
		break;
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
	case TIM_MODE_PWM:
		tim_ctx.common[instance].clock_source = RCC_CLOCK_SYSTEM;
		break;
#endif
	default:
		status = TIM_ERROR_MODE;
		goto errors;
	}
	// Update mode.
	tim_ctx.common[instance].mode = (configuration -> mode);
	// Get clock source frequency.
	RCC_get_frequency_hz(tim_ctx.common[instance].clock_source, &tim_clock_hz);
	// Enable peripheral clock.
	(*TIM_DESCRIPTOR[instance].rcc_enr) |= TIM_DESCRIPTOR[instance].rcc_mask;
	(*TIM_DESCRIPTOR[instance].rcc_smenr) |= TIM_DESCRIPTOR[instance].rcc_mask;
	// Configure peripheral.
	switch (configuration -> mode) {
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
	case TIM_MODE_STANDARD:
		// Compute ARR value.
		arr = ((uint64_t) (configuration -> period_ns)) * ((uint64_t) tim_clock_hz);
		arr /= ((uint64_t) 1000000000);
		// No prescaler.
		TIM_DESCRIPTOR[instance].peripheral -> PSC = 0;
		// Enable interrupt.
		TIM_DESCRIPTOR[instance].peripheral -> DIER |= (0b1 << 0);
		// Update internal callback.
		tim_ctx.common[instance].irq_handler = &_TIM_irq_handler_standard;
		// Register external callback.
		tim_ctx.standard[instance].irq_callback = (configuration -> irq_callback);
		break;
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
	case TIM_MODE_MULTI_CHANNEL:
		// Init context.
		for (idx=0 ; idx<TIM_CHANNEL_LAST ; idx++) {
			tim_ctx.multi_channel.channel[idx].duration_ms = 0;
			tim_ctx.multi_channel.channel[idx].waiting_mode = TIM_WAITING_MODE_ACTIVE;
			tim_ctx.multi_channel.channel[idx].running_flag = 0;
			tim_ctx.multi_channel.channel[idx].irq_flag = 0;
		}
		// Configure channels.
#if (STM32L0XX_DRIVERS_RCC_LSE_MODE == 0)
		RCC -> CR |= (0b1 << 5); // HSI16OUTEN='1'.
		TIM_DESCRIPTOR[instance].peripheral -> SMCR |= (0b11 << 12); // ETRF prescaler = 8 (minimum 4 due to CK_INT clock ratio constraint).
		TIM_DESCRIPTOR[instance].peripheral -> PSC = (TIM_PRESCAKER_PSC_HSI - 1);
		TIM_DESCRIPTOR[instance].peripheral -> OR |= (0b011 << 0);
		// Update clock frequency.
		tim_ctx.multi_channel.etrf_clock_hz = ((tim_clock_hz) / (TIM_PRESCALER_ETRF_HSI * TIM_PRESCAKER_PSC_HSI));
#elif (STM32L0XX_DRIVERS_RCC_LSE_MODE == 1)
	// Select trigger.
	if (lse_status != 0) {
		// Use LSE as trigger.
		RCC -> CR &= ~(0b1 << 5); // HSI16OUTEN='0'.
		TIM_DESCRIPTOR[instance].peripheral -> SMCR &= ~(0b11 << 12); // No prescaler on ETRF.
		TIM_DESCRIPTOR[instance].peripheral -> PSC = (TIM_PRESCALER_PSC_LSE - 1);
		TIM_DESCRIPTOR[instance].peripheral -> OR |= (0b101 << 0);
		// Update clock frequency.
		tim_ctx.multi_channel.etrf_clock_hz = ((tim_clock_hz) / (TIM_PRESCALER_ETRF_LSE * TIM_PRESCALER_PSC_LSE));
	}
	else {
		// Use HSI as trigger.
		RCC -> CR |= (0b1 << 5); // HSI16OUTEN='1'.
		TIM_DESCRIPTOR[instance].peripheral -> SMCR |= (0b11 << 12); // ETRF prescaler = 8 (minimum 4 due to CK_INT clock ratio constraint).
		TIM_DESCRIPTOR[instance].peripheral -> PSC = (TIM_PRESCAKER_PSC_HSI - 1);
		TIM_DESCRIPTOR[instance].peripheral -> OR |= (0b011 << 0);
		/// Update clock frequency.
		tim_ctx.multi_channel.etrf_clock_hz = ((tim_clock_hz) / (TIM_PRESCALER_ETRF_HSI * TIM_PRESCAKER_PSC_HSI));
	}
#else
		// Use LSE as trigger.
		RCC -> CR &= ~(0b1 << 5); // HSI16OUTEN='0'.
		TIM_DESCRIPTOR[instance].peripheral -> SMCR &= ~(0b11 << 12); // No prescaler on ETRF.
		TIM_DESCRIPTOR[instance].peripheral -> PSC = (TIM_PRESCALER_PSC_LSE - 1);
		TIM_DESCRIPTOR[instance].peripheral -> OR |= (0b101 << 0);
		// Update clock frequency.
		tim_ctx.multi_channel.etrf_clock_hz = ((tim_clock_hz) / (TIM_PRESCALER_ETRF_LSE * TIM_PRESCALER_PSC_LSE));
#endif
		// No overflow.
		arr = 0xFFFF;
		// Use external clock mode 2.
		TIM_DESCRIPTOR[instance].peripheral -> SMCR |= (0b1 << 14) | (0b111 << 4);
		// Configure channels 1-4 in output compare mode.
		TIM_DESCRIPTOR[instance].peripheral -> CCMR1 &= 0xFFFF0000;
		TIM_DESCRIPTOR[instance].peripheral -> CCMR2 &= 0xFFFF0000;
		TIM_DESCRIPTOR[instance].peripheral -> CCER &= 0xFFFF0000;
		// Update internal callback.
		tim_ctx.common[instance].irq_handler = &_TIM_irq_handler_multi_channel;
		break;
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
	case TIM_MODE_CALIBRATION:
		// Channel input on TI1.
		// Capture done every 8 edges.
		// CH1 mapped on MCO.
		TIM_DESCRIPTOR[instance].peripheral -> CCMR1 |= (0b01 << 0) | (0b11 << 2);
		TIM_DESCRIPTOR[instance].peripheral -> OR |= (0b111 << 2);
		// Enable interrupt.
		TIM_DESCRIPTOR[instance].peripheral -> DIER |= (0b1 << 1); // CC1IE='1'.
		// Update internal callback.
		tim_ctx.common[instance].irq_handler = &_TIM_irq_handler_calibration;
		break;
#endif
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
	case TIM_MODE_PWM:
		// Set PWM frequency.
		TIM_DESCRIPTOR[instance].peripheral -> ARR = ((tim_clock_hz) / (configuration -> pwm_frequency_hz));
		// Reset counter.
		TIM_DESCRIPTOR[instance].peripheral -> CNT = 0;
		// Configure channels 1-4 in PWM mode 1 (OCxM='110' and OCxPE='1').
		TIM_DESCRIPTOR[instance].peripheral -> CCMR1 |= (0b110 << 12) | (0b1 << 11) | (0b110 << 4) | (0b1 << 3);
		TIM_DESCRIPTOR[instance].peripheral -> CCMR2 |= (0b110 << 12) | (0b1 << 11) | (0b110 << 4) | (0b1 << 3);
		TIM_DESCRIPTOR[instance].peripheral -> CR1 |= (0b1 << 7);
		// Disable all channels by default.
		tim_ctx.pwm[instance].channels_duty_cycle = 0;
		for (idx=0 ; idx<TIM_CHANNEL_LAST ; idx++) TIM_DESCRIPTOR[instance].peripheral -> CCRx[idx] = ((TIM_DESCRIPTOR[instance].peripheral -> ARR) + 1);
		TIM_DESCRIPTOR[instance].peripheral -> CCER |= 0x00001111;
		break;
#endif
	default:
		status = TIM_ERROR_MODE;
		goto errors;
	}
	// Write common registers.
	if ((arr < ((uint64_t) TIM_ARR_VALUE_MIN)) || (arr > ((uint64_t) TIM_ARR_VALUE_MAX))) {
		status = TIM_ERROR_ARR_VALUE;
		goto errors;
	}
	TIM_DESCRIPTOR[instance].peripheral -> ARR = ((uint16_t) arr);
	// Generate event to update registers.
	TIM_DESCRIPTOR[instance].peripheral -> EGR |= (0b1 << 0); // UG='1'.
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
	// Init pins if needed.
	if ((configuration -> mode) == TIM_MODE_PWM) {
		// Check parameter.
		if (pins == NULL) {
			status = TIM_ERROR_NULL_PARAMETER;
			goto errors;
		}
		// Configure GPIOs.
		if ((pins -> ch1) != NULL) {
			GPIO_configure((pins -> ch1), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
		}
		if ((pins -> ch2) != NULL) {
			GPIO_configure((pins -> ch1), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
		}
		if ((pins -> ch3) != NULL) {
			GPIO_configure((pins -> ch1), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
		}
		if ((pins -> ch4) != NULL) {
			GPIO_configure((pins -> ch1), GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
		}
	}
#else
	UNUSED(pins);
#endif
	// Set interrupt priority.
	NVIC_set_priority(TIM_DESCRIPTOR[instance].nvic_interrupt, (configuration -> nvic_priority));
errors:
	return status;
}

/*******************************************************************/
TIM_status_t TIM_de_init(TIM_instance_t instance, TIM_gpio_t* pins) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	// Check instance.
	if (instance >= TIM_INSTANCE_LAST) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
	if (tim_ctx.common[instance].mode == TIM_MODE_PWM) {
		// Check parameter.
		if (pins == NULL) {
			status = TIM_ERROR_NULL_PARAMETER;
			goto errors;
		}
		// Release GPIOs.
		if ((pins -> ch1) != NULL) {
			GPIO_configure((pins -> ch1), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
		}
		if ((pins -> ch2) != NULL) {
			GPIO_configure((pins -> ch1), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
		}
		if ((pins -> ch3) != NULL) {
			GPIO_configure((pins -> ch1), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
		}
		if ((pins -> ch4) != NULL) {
			GPIO_configure((pins -> ch1), GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
		}
	}
#else
	UNUSED(pins);
#endif
errors:
	// Disable interrupt.
	NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
	// Disable timer.
	TIM_DESCRIPTOR[instance].peripheral -> CR1 &= ~(0b1 << 0); // CEN='0'.
	// Disable peripheral clock.
	(*TIM_DESCRIPTOR[instance].rcc_enr) &= ~(TIM_DESCRIPTOR[instance].rcc_mask);
	return status;
}

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
/*******************************************************************/
TIM_status_t TIM_start(TIM_instance_t instance) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	// Check instance.
	if (instance >= TIM_INSTANCE_LAST) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
	// Enable interrupt.
	NVIC_enable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
	// Start timer.
	TIM_DESCRIPTOR[instance].peripheral -> CR1 |= (0b1 << 0);
errors:
	return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x01) != 0)
/*******************************************************************/
TIM_status_t TIM_stop(TIM_instance_t instance) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	// Check instance.
	if (instance >= TIM_INSTANCE_LAST) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
	// Stop timer.
	TIM_DESCRIPTOR[instance].peripheral -> CR1 &= ~(0b1 << 0);
	// Disable interrupt.
	NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
errors:
	return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
TIM_status_t TIM_start_channel(TIM_instance_t instance, TIM_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint32_t local_duration_ms = duration_ms;
	uint32_t duration_min_ms = TIM_TIMER_DURATION_MS_MIN;
	// Check instance.
	if (instance != TIM_INSTANCE_TIM2) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
	// Check channel.
	_TIM_check_channel(channel);
	// Check parameters.
	if (waiting_mode >= TIM_WAITING_MODE_LAST) {
		status = TIM_ERROR_WAITING_MODE;
		goto errors;
	}
	if (waiting_mode == TIM_WAITING_MODE_LOW_POWER_SLEEP) {
		// Compensate clock switch latency.
		duration_min_ms += TIM_CLOCK_SWITCH_LATENCY_MS;
	}
	if (duration_ms < duration_min_ms) {
		status = TIM_ERROR_DURATION_UNDERFLOW;
		goto errors;
	}
	if (duration_ms > TIM_TIMER_DURATION_MS_MAX) {
		status = TIM_ERROR_DURATION_OVERFLOW;
		goto errors;
	}
	if (waiting_mode == TIM_WAITING_MODE_LOW_POWER_SLEEP) {
		local_duration_ms -= TIM_CLOCK_SWITCH_LATENCY_MS;
	}
	// Update channel context.
	tim_ctx.multi_channel.channel[channel].duration_ms = local_duration_ms;
	tim_ctx.multi_channel.channel[channel].waiting_mode = waiting_mode;
	tim_ctx.multi_channel.channel[channel].running_flag = 1;
	tim_ctx.multi_channel.channel[channel].irq_flag = 0;
	// Compute compare value.
	_TIM_compute_compare_value(instance, channel);
	// Clear flag.
	TIM_DESCRIPTOR[instance].peripheral -> SR &= ~(0b1 << (channel + 1));
	// Enable channel.
	TIM_DESCRIPTOR[instance].peripheral -> DIER |= (0b1 << (channel + 1));
	// Enable interrupt.
	NVIC_enable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
	// Enable counter.
	TIM_DESCRIPTOR[instance].peripheral -> CR1 |= (0b1 << 0);
errors:
	return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
TIM_status_t TIM_stop_channel(TIM_instance_t instance, TIM_channel_t channel) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	// Check instance.
	if (instance != TIM_INSTANCE_TIM2) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
	// Check channel.
	_TIM_check_channel(channel);
	// Disable interrupt.
	TIM_DESCRIPTOR[instance].peripheral -> DIER &= ~(0b1 << (channel + 1));
	// Clear flag.
	TIM_DESCRIPTOR[instance].peripheral -> SR &= ~(0b1 << (channel + 1));
	// Disable channel.
	tim_ctx.multi_channel.channel[channel].running_flag = 0;
	tim_ctx.multi_channel.channel[channel].irq_flag = 0;
	// Disable counter if all channels are stopped.
	if (((TIM_DESCRIPTOR[instance].peripheral -> DIER) & 0x0000001E) == 0) {
		// Disable counter.
		TIM_DESCRIPTOR[instance].peripheral -> CR1 &= ~(0b1 << 0);
		// Disable interrupt.
		NVIC_enable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
	}
errors:
	return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
TIM_status_t TIM_get_channel_status(TIM_instance_t instance, TIM_channel_t channel, uint8_t* timer_has_elapsed) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	// Check instance.
	if (instance != TIM_INSTANCE_TIM2) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
	// Check channel.
	_TIM_check_channel(channel);
	// Check parameters.
	if (timer_has_elapsed == NULL) {
		status = TIM_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Update flag.
	(*timer_has_elapsed) = ((tim_ctx.multi_channel.channel[channel].running_flag == 0) || (tim_ctx.multi_channel.channel[channel].irq_flag != 0)) ? 1 : 0;
errors:
	return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x02) != 0)
/*******************************************************************/
TIM_status_t TIM_wait_channel_completion(TIM_instance_t instance, TIM_channel_t channel) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	RCC_status_t rcc_status = RCC_SUCCESS;
	uint32_t time_start = RTC_get_uptime_seconds();
	uint32_t time_reference = 0;
	// Check instance.
	if (instance != TIM_INSTANCE_TIM2) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
	// Check channel.
	_TIM_check_channel(channel);
	// Directly exit if the IRQ already occurred.
	if ((tim_ctx.multi_channel.channel[channel].running_flag == 0) || (tim_ctx.multi_channel.channel[channel].irq_flag != 0)) goto errors;
	// Sleep until channel is not running.
	switch (tim_ctx.multi_channel.channel[channel].waiting_mode) {
	case TIM_WAITING_MODE_ACTIVE:
		// Active loop.
		while (tim_ctx.multi_channel.channel[channel].irq_flag == 0) {
			// Internal watchdog.
			status = _TIM_internal_watchdog(time_start, &time_reference);
			if (status != TIM_SUCCESS) goto errors;
		}
		break;
	case TIM_WAITING_MODE_SLEEP:
		// Enter sleep mode.
		while (tim_ctx.multi_channel.channel[channel].irq_flag == 0) {
			PWR_enter_sleep_mode();
			// Internal watchdog.
			status = _TIM_internal_watchdog(time_start, &time_reference);
			if (status != TIM_SUCCESS) goto errors;
		}
		break;
	case TIM_WAITING_MODE_LOW_POWER_SLEEP:
		// Check trigger source.
		if (tim_ctx.common[instance].clock_source == RCC_CLOCK_LSE) {
			// Switch to MSI.
			rcc_status = RCC_switch_to_msi(RCC_MSI_RANGE_1_131KHZ);
			RCC_exit_error(TIM_ERROR_BASE_RCC);
			// Enter low power sleep mode.
			while (tim_ctx.multi_channel.channel[channel].irq_flag == 0) {
				PWR_enter_low_power_sleep_mode();
				// Internal watchdog.
				status = _TIM_internal_watchdog(time_start, &time_reference);
				if (status != TIM_SUCCESS) goto errors;
			}
			// Go back to HSI.
			rcc_status = RCC_switch_to_hsi();
			RCC_exit_error(TIM_ERROR_BASE_RCC);
		}
		else {
			// Enter sleep mode.
			while (tim_ctx.multi_channel.channel[channel].irq_flag == 0) {
				PWR_enter_sleep_mode();
				// Internal watchdog.
				status = _TIM_internal_watchdog(time_start, &time_reference);
				if (status != TIM_SUCCESS) goto errors;
			}
		}
		break;
	default:
		status = TIM_ERROR_WAITING_MODE;
		goto errors;
	}
	// Clear flag and update compare value for next IRQ.
	tim_ctx.multi_channel.channel[channel].irq_flag = 0;
	_TIM_compute_compare_value(instance, channel);
errors:
	return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x04) != 0)
/*******************************************************************/
TIM_status_t TIM_mco_capture(TIM_instance_t instance, uint16_t* ref_clock_pulse_count, uint16_t* mco_pulse_count) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint32_t loop_count = 0;
	// Check instance.
	if (instance != TIM_INSTANCE_TIM21) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
	// Check parameters.
	if ((ref_clock_pulse_count == NULL) || (mco_pulse_count == NULL)) {
		status = TIM_ERROR_NULL_PARAMETER;
		goto errors;
	}
	// Reset timer context.
	tim_ctx.calibration.ccr1_start = 0;
	tim_ctx.calibration.ccr1_end = 0;
	tim_ctx.calibration.capture_count = 0;
	tim_ctx.calibration.capture_done = 0;
	// Reset counter.
	TIM_DESCRIPTOR[instance].peripheral -> CNT = 0;
	TIM_DESCRIPTOR[instance].peripheral -> CCR1 = 0;
	// Enable interrupt.
	TIM_DESCRIPTOR[instance].peripheral -> SR &= 0xFFFFF9B8; // Clear all flags.
	NVIC_enable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
	// Enable TIM peripheral.
	TIM_DESCRIPTOR[instance].peripheral -> CR1 |= (0b1 << 0); // CEN='1'.
	TIM_DESCRIPTOR[instance].peripheral -> CCER |= (0b1 << 0); // CC1E='1'.
	// Wait for capture to complete.
	while (tim_ctx.calibration.capture_done == 0) {
		// Manage timeout.
		loop_count++;
		if (loop_count > TIM_TIMEOUT_COUNT) {
			status = TIM_ERROR_CAPTURE_TIMEOUT;
			goto errors;
		}
	}
	// Update results.
	(*ref_clock_pulse_count) = (tim_ctx.calibration.ccr1_end - tim_ctx.calibration.ccr1_start);
	(*mco_pulse_count) = (TIM_INPUT_CAPTURE_PRESCALER * (tim_ctx.calibration.capture_count - 1));
	// Disable interrupt.
	NVIC_disable_interrupt(TIM_DESCRIPTOR[instance].nvic_interrupt);
	// Stop counter.
	TIM_DESCRIPTOR[instance].peripheral -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM_DESCRIPTOR[instance].peripheral -> CCER &= ~(0b1 << 0); // CC1E='0'.
errors:
	return status;
}
#endif

#if ((STM32L0XX_DRIVERS_TIM_MODE_MASK & 0x08) != 0)
/*******************************************************************/
TIM_status_t TIM_set_pwm_duty_cycle(TIM_instance_t instance, TIM_channel_t channel, uint8_t duty_cycle_percent) {
	// Local variables.
	TIM_status_t status = TIM_SUCCESS;
	uint32_t arr_plus_one = 0;
	// Check instance.
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 3) || (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	if (instance == TIM_INSTANCE_TIM6) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
#endif
#if (STM32L0XX_REGISTERS_MCU_CATEGORY == 5)
	// Check instance.
	if (instance == TIM_INSTANCE_TIM7) {
		status = TIM_ERROR_INSTANCE;
		goto errors;
	}
#endif
	// Check channel.
	_TIM_check_channel(channel);
	// Check parameters.
	if (duty_cycle_percent > TIM_DUTY_CYCLE_PERCENT_MAX) {
		status = TIM_ERROR_DUTY_CYCLE;
		goto errors;
	}
	// Update duty cycle.
	tim_ctx.pwm[instance].channels_duty_cycle &= ~(0xFF << (channel << 3));
	tim_ctx.pwm[instance].channels_duty_cycle |= (duty_cycle_percent << (channel << 3));
	// Set duty cycle.
	arr_plus_one = ((TIM_DESCRIPTOR[instance].peripheral -> ARR) + 1);
	TIM_DESCRIPTOR[instance].peripheral -> CCRx[channel] = arr_plus_one - ((arr_plus_one * duty_cycle_percent) / (TIM_DUTY_CYCLE_PERCENT_MAX));
	// Check channels status.
	if (tim_ctx.pwm[instance].channels_duty_cycle == 0) {
		// Disable and reset counter.
		TIM_DESCRIPTOR[instance].peripheral -> CR1 &= ~(0b1 << 0); // CEN='0'.
		TIM_DESCRIPTOR[instance].peripheral -> CNT = 0;
	}
	else {
		// Enable counter.
		TIM_DESCRIPTOR[instance].peripheral -> CR1 |= (0b1 << 0); // CEN='1'.
	}
errors:
	return status;
}
#endif
