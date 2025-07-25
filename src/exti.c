/*
 * exti.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE
#include "stm32l0xx_drivers_flags.h"
#endif

#ifndef STM32L0XX_DRIVERS_DISABLE

#include "exti.h"

#include "exti_registers.h"
#include "gpio.h"
#include "nvic.h"
#include "rcc_registers.h"
#include "syscfg_registers.h"
#include "types.h"

/*** EXTI local macros ***/

#define EXTI_RTSR_FTSR_RESERVED_INDEX           18
#define EXTI_RTSR_FTSR_MAX_INDEX                22

#define EXTI_NVIC_SHARED_GPIO_MASK_PIN0_PIN1    0x0003
#define EXTI_NVIC_SHARED_GPIO_MASK_PIN2_PIN3    0x000C
#define EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15   0xFFF0

#define EXTI_REGISTER_MASK_IMR_EMR              0x37FFFFFF
#define EXTI_REGISTER_MASK_PR                   0x007BFFFF

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)

/*** EXTI local structures ***/

/*******************************************************************/
typedef struct {
    NVIC_interrupt_t nvic_interrupt;
    uint16_t nvic_shared_mask;
} EXTI_gpio_descriptor_t;

/*******************************************************************/
typedef struct {
    EXTI_gpio_irq_cb_t edge_irq_callback;
} EXTI_gpio_context_t;

/*******************************************************************/
typedef struct {
    uint16_t enabled_gpio_mask;
    EXTI_gpio_context_t gpio_ctx[GPIO_PINS_PER_PORT];
} EXTI_context_t;

/*** EXTI local global variables ***/

static const EXTI_gpio_descriptor_t EXTI_GPIO_DESCRIPTOR[GPIO_PINS_PER_PORT] = {
    { NVIC_INTERRUPT_EXTI_0_1,  EXTI_NVIC_SHARED_GPIO_MASK_PIN0_PIN1  },
    { NVIC_INTERRUPT_EXTI_0_1,  EXTI_NVIC_SHARED_GPIO_MASK_PIN0_PIN1  },
    { NVIC_INTERRUPT_EXTI_2_3,  EXTI_NVIC_SHARED_GPIO_MASK_PIN2_PIN3  },
    { NVIC_INTERRUPT_EXTI_2_3,  EXTI_NVIC_SHARED_GPIO_MASK_PIN2_PIN3  },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 },
    { NVIC_INTERRUPT_EXTI_4_15, EXTI_NVIC_SHARED_GPIO_MASK_PIN4_PIN15 }
};

static EXTI_context_t exti_ctx = {
    .enabled_gpio_mask = 0,
    .gpio_ctx = {
        [0 ... (GPIO_PINS_PER_PORT - 1)] {
            .edge_irq_callback = NULL
        }
    }
};

/*** EXTI local functions ***/

/*******************************************************************/
#define _EXTI_irq_handler(pin) { \
    /* Check flag */ \
    if (((EXTI->PR) & (0b1 << pin)) != 0) { \
        /* Check mask and callback */ \
        if ((((EXTI->IMR) & (0b1 << pin)) != 0) && (exti_ctx.gpio_ctx[pin].edge_irq_callback != NULL)) { \
            /* Execute callback */ \
            exti_ctx.gpio_ctx[pin].edge_irq_callback(); \
        } \
        /* Clear flag */ \
        EXTI->PR = (0b1 << pin); \
    } \
}

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_NVIC_SHARED_GPIO_MASK_PIN0_PIN1) != 0)
/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI0_1_IRQHandler(void) {
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN0) != 0)
    _EXTI_irq_handler(0);
#endif
#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_PIN1) != 0)
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

#endif /* STM32L0XX_DRIVERS_EXTI_GPIO_MASK */

/*******************************************************************/
static void _EXTI_set_trigger(EXTI_line_t line, EXTI_trigger_t trigger) {
    // Select triggers.
    switch (trigger) {
    // Rising edge only.
    case EXTI_TRIGGER_RISING_EDGE:
        EXTI->RTSR |= (0b1 << line);
        EXTI->FTSR &= ~(0b1 << line);
        break;
    // Falling edge only.
    case EXTI_TRIGGER_FALLING_EDGE:
        EXTI->RTSR &= ~(0b1 << line);
        EXTI->FTSR |= (0b1 << line);
        break;
    // Both edges.
    case EXTI_TRIGGER_ANY_EDGE:
        EXTI->RTSR |= (0b1 << line);
        EXTI->FTSR |= (0b1 << line);
        break;
    // Unknown configuration.
    default:
        goto errors;
    }
    // Clear flag.
    EXTI->PR = (0b1 << line);
errors:
    return;
}

/*** EXTI functions ***/

/*******************************************************************/
void EXTI_init(void) {
    // Enable peripheral clock.
    RCC->APB2ENR |= (0b1 << 0);
    // Mask all sources by default.
    EXTI->IMR &= (~EXTI_REGISTER_MASK_IMR_EMR);
    EXTI->EMR &= (~EXTI_REGISTER_MASK_IMR_EMR);
    // Clear all flags.
    EXTI->PR = EXTI_REGISTER_MASK_PR;
}

/*******************************************************************/
void EXTI_de_init(void) {
    // Clear all flags.
    EXTI->PR = EXTI_REGISTER_MASK_PR;
    // Mask all sources.
    EXTI->IMR &= (~EXTI_REGISTER_MASK_IMR_EMR);;
    EXTI->EMR &= (~EXTI_REGISTER_MASK_IMR_EMR);;
    // Disable peripheral clock.
    RCC->APB2ENR &= ~(0b1 << 0);
}

/*******************************************************************/
void EXTI_enable_line(EXTI_line_t line, EXTI_trigger_t trigger) {
    // Select triggers.
    _EXTI_set_trigger(line, trigger);
    // Set mask.
    EXTI->IMR |= (0b1 << line);
}

/*******************************************************************/
void EXTI_disable_line(EXTI_line_t line) {
    // Set mask.
    EXTI->IMR &= ~(0b1 << line);
}

/*******************************************************************/
void EXTI_clear_line_flag(EXTI_line_t line) {
    // Clear flag.
    EXTI->PR = (0b1 << line);
}

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_configure_gpio(const GPIO_pin_t* gpio, GPIO_pull_resistor_t pull_resistor, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback, uint8_t nvic_priority) {
    // Local variables.
    uint8_t line_idx = (gpio->pin);
    // Select GPIO port.
    SYSCFG->EXTICR[(line_idx >> 2)] &= ~(0b1111 << ((line_idx % 4) << 2));
    SYSCFG->EXTICR[(line_idx >> 2)] |= ((gpio->port_index) << ((line_idx % 4) << 2));
    // Select triggers.
    _EXTI_set_trigger(line_idx, trigger);
    // Set interrupt priority.
    NVIC_set_priority(EXTI_GPIO_DESCRIPTOR[line_idx].nvic_interrupt, nvic_priority);
    // Register callback.
    exti_ctx.gpio_ctx[line_idx].edge_irq_callback = irq_callback;
    // Configure GPIO.
    GPIO_configure(gpio, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, pull_resistor);
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_release_gpio(const GPIO_pin_t* gpio, GPIO_mode_t released_mode) {
    // Set state.
    EXTI_disable_gpio_interrupt(gpio);
    // Release GPIO.
    GPIO_configure(gpio, released_mode, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_enable_gpio_interrupt(const GPIO_pin_t* gpio) {
    // Local variables.
    uint8_t line_idx = (gpio->pin);
    // Update mask.
    exti_ctx.enabled_gpio_mask |= (0b1 << line_idx);
    // Set mask.
    EXTI->IMR |= (0b1 << line_idx);
    // Enable interrupt.
    NVIC_enable_interrupt(EXTI_GPIO_DESCRIPTOR[line_idx].nvic_interrupt);
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_disable_gpio_interrupt(const GPIO_pin_t* gpio) {
    // Local variables.
    uint8_t line_idx = (gpio->pin);
    // Update mask.
    exti_ctx.enabled_gpio_mask &= ~(0b1 << line_idx);
    // Set mask.
    EXTI->IMR &= ~(0b1 << line_idx);
    // Disable interrupt.
    if ((exti_ctx.enabled_gpio_mask & EXTI_GPIO_DESCRIPTOR[line_idx].nvic_shared_mask) == 0) {
        NVIC_disable_interrupt(EXTI_GPIO_DESCRIPTOR[line_idx].nvic_interrupt);
    }
}
#endif

#if ((STM32L0XX_DRIVERS_EXTI_GPIO_MASK & EXTI_GPIO_MASK_ALL) != 0)
/*******************************************************************/
void EXTI_clear_gpio_flag(const GPIO_pin_t* gpio) {
    // Local variables.
    uint8_t line_idx = (gpio->pin);
    // Clear flag.
    EXTI->PR = (0b1 << line_idx);
}
#endif

#endif /* STM32L0XX_DRIVERS_DISABLE */
