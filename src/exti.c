/*
 * exti.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "exti.h"

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif
#include "exti_reg.h"
#include "gpio.h"
#include "nvic.h"
#include "rcc_reg.h"
#include "syscfg_reg.h"
#include "types.h"

/*** EXTI local macros ***/

#define EXTI_RTSR_FTSR_RESERVED_INDEX			18
#define EXTI_RTSR_FTSR_MAX_INDEX				22

#define EXTI_NVIC_SHARED_GPIO_MASK_PIN0_PIN1	0x0003
#define EXTI_NVIC_SHARED_GPIO_MASK_PIN2_PIN3	0x000C
#define EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15	0xFFF0

/*** EXTI local structures ***/

/*******************************************************************/
typedef struct {
	NVIC_interrupt_t nvic_interrupt;
	uint16_t nvic_shared_mask;
} EXTI_descriptor_t;

/*******************************************************************/
typedef struct {
	uint8_t enabled_gpio_mask;
	EXTI_gpio_irq_cb_t gpio_irq_callbacks[GPIO_PINS_PER_PORT];
} EXTI_context_t;

/*** EXTI local global variables ***/

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
static const EXTI_descriptor_t EXTI_DESCRIPTOR[GPIO_PINS_PER_PORT] = {
	{NVIC_INTERRUPT_EXTI_0_1,  EXTI_NVIC_SHARED_GPIO_MASK_PIN0_PIN1},
	{NVIC_INTERRUPT_EXTI_0_1,  EXTI_NVIC_SHARED_GPIO_MASK_PIN0_PIN1},
	{NVIC_INTERRUPT_EXTI_2_3,  EXTI_NVIC_SHARED_GPIO_MASK_PIN2_PIN3},
	{NVIC_INTERRUPT_EXTI_2_3,  EXTI_NVIC_SHARED_GPIO_MASK_PIN2_PIN3},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
	{NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15},
};
static EXTI_context_t exti_ctx = {
	.enabled_gpio_mask = 0,
	.gpio_irq_callbacks = {NULL}
};
#endif

/*** EXTI local functions ***/

/*******************************************************************/
#define _EXTI_irq_handler(pin) { \
	/* Check flag */ \
	if (((EXTI -> PR) & (0b1 << pin)) != 0) { \
		/* Check mask and callback */ \
		if ((((EXTI -> IMR) & (0b1 << pin)) != 0) && (exti_ctx.gpio_irq_callbacks[pin] != NULL)) { \
			/* Execute callback */ \
			exti_ctx.gpio_irq_callbacks[pin](); \
		} \
		/* Clear flag */ \
		EXTI -> PR |= (0b1 << pin); \
	} \
}

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_NVIC_SHARED_GPIO_MASK_PIN0_PIN1) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI0_1_IRQHandler(void) {
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN0) != 0)
	// Px0.
	_EXTI_irq_handler(0);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN1) != 0)
	// Px1.
	_EXTI_irq_handler(1);
#endif
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_NVIC_SHARED_GPIO_MASK_PIN2_PIN3) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI2_3_IRQHandler(void) {
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN2) != 0)
	_EXTI_irq_handler(2);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN3) != 0)
	_EXTI_irq_handler(3);
#endif
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI4_15_IRQHandler(void) {
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN4) != 0)
	_EXTI_irq_handler(4);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN5) != 0)
	_EXTI_irq_handler(5);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN6) != 0)
	_EXTI_irq_handler(6);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN7) != 0)
	_EXTI_irq_handler(7);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN8) != 0)
	_EXTI_irq_handler(8);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN9) != 0)
	_EXTI_irq_handler(9);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN10) != 0)
	_EXTI_irq_handler(10);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN11) != 0)
	_EXTI_irq_handler(11);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN12) != 0)
	_EXTI_irq_handler(12);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN13) != 0)
	_EXTI_irq_handler(13);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN14) != 0)
	_EXTI_irq_handler(14);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN15) != 0)
	_EXTI_irq_handler(15);
#endif
}
#endif

/*******************************************************************/
static void _EXTI_set_trigger(EXTI_trigger_t trigger, uint8_t line_idx) {
	// Select triggers.
	switch (trigger) {
	// Rising edge only.
	case EXTI_TRIGGER_RISING_EDGE:
		EXTI -> RTSR |= (0b1 << line_idx);
		EXTI -> FTSR &= ~(0b1 << line_idx);
		break;
	// Falling edge only.
	case EXTI_TRIGGER_FALLING_EDGE:
		EXTI -> RTSR &= ~(0b1 << line_idx);
		EXTI -> FTSR |= (0b1 << line_idx);
		break;
	// Both edges.
	case EXTI_TRIGGER_ANY_EDGE:
		EXTI -> RTSR |= (0b1 << line_idx);
		EXTI -> FTSR |= (0b1 << line_idx);
		break;
	// Unknown configuration.
	default:
		goto errors;
	}
	// Clear flag.
	EXTI -> PR |= (0b1 << line_idx);
errors:
	return;
}

/*** EXTI functions ***/

/*******************************************************************/
void EXTI_init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 0); // SYSCFEN='1'.
	// Mask all sources by default.
	EXTI -> IMR = 0;
	// Clear all flags.
	EXTI -> PR |= 0x007BFFFF; // PIFx='1'.
}

/*******************************************************************/
void EXTI_configure_line(EXTI_line_t line, EXTI_trigger_t trigger) {
	// Select triggers.
	_EXTI_set_trigger(trigger, line);
	// Set mask.
	EXTI -> IMR |= (0b1 << line); // IMx='1'.
}

/*******************************************************************/
void EXTI_release_line(EXTI_line_t line) {
	// Set mask.
	EXTI -> IMR &= ~(0b1 << line); // IMx='0'.
}

/*******************************************************************/
void EXTI_clear_line_flag(EXTI_line_t line) {
	// Clear flag.
	EXTI -> PR |= line; // PIFx='1'.
}

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_configure_gpio(const GPIO_pin_t* gpio, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback, uint8_t nvic_priority) {
	// Select GPIO port.
	SYSCFG -> EXTICR[((gpio -> pin) >> 2)] &= ~(0b1111 << (((gpio -> pin) % 4) << 2));
	SYSCFG -> EXTICR[((gpio -> pin) >> 2)] |= ((gpio -> port_index) << (((gpio -> pin) % 4) << 2));
	// Select triggers.
	_EXTI_set_trigger(trigger, (gpio -> pin));
	// Set mask.
	EXTI -> IMR |= (0b1 << ((gpio -> pin))); // IMx='1'.
	// Set interrupt priority.
	NVIC_set_priority(EXTI_DESCRIPTOR[gpio -> pin].nvic_interrupt, nvic_priority);
	// Register callback.
	exti_ctx.gpio_irq_callbacks[gpio -> pin] = irq_callback;

}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_release_gpio(const GPIO_pin_t* gpio) {
	// Set mask.
	EXTI -> IMR &= ~(0b1 << ((gpio -> pin))); // IMx='0'.
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_clear_gpio_flag(const GPIO_pin_t* gpio) {
	// Clear flag.
	EXTI -> PR |= (gpio -> pin); // PIFx='1'.
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_enable_gpio_interrupt(const GPIO_pin_t* gpio) {
	// Enable interrupt.
	NVIC_enable_interrupt(EXTI_DESCRIPTOR[gpio -> pin].nvic_interrupt);
	// Update mask.
	exti_ctx.enabled_gpio_mask |= (0b1 << (gpio -> pin));
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_disable_gpio_interrupt(const GPIO_pin_t* gpio) {
	// Update mask.
	exti_ctx.enabled_gpio_mask &= ~(0b1 << (gpio -> pin));
	// Disable interrupt.
	if ((exti_ctx.enabled_gpio_mask & EXTI_DESCRIPTOR[gpio -> pin].nvic_shared_mask) == 0) {
		NVIC_disable_interrupt(EXTI_DESCRIPTOR[gpio -> pin].nvic_interrupt);
	}
}
#endif
