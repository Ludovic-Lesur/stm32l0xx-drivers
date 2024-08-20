/*
 * exti.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __EXTI_H__
#define __EXTI_H__

#include "gpio.h"
#include "types.h"

/*** EXTI structures ***/

/*!******************************************************************
 * \enum EXTI_line_t
 * \brief EXTI lines list.
 *******************************************************************/
typedef enum {
	EXTI_LINE_GPIO_0 = 0,
	EXTI_LINE_GPIO_1 = 1,
	EXTI_LINE_GPIO_2 = 2,
	EXTI_LINE_GPIO_3 = 3,
	EXTI_LINE_GPIO_4 = 4,
	EXTI_LINE_GPIO_5 = 5,
	EXTI_LINE_GPIO_6 = 6,
	EXTI_LINE_GPIO_7 = 7,
	EXTI_LINE_GPIO_8 = 8,
	EXTI_LINE_GPIO_9 = 9,
	EXTI_LINE_GPIO_10 = 10,
	EXTI_LINE_GPIO_11 = 11,
	EXTI_LINE_GPIO_12 = 12,
	EXTI_LINE_GPIO_13 = 13,
	EXTI_LINE_GPIO_14 = 14,
	EXTI_LINE_GPIO_15 = 15,
	EXTI_LINE_PVD = 16,
	EXTI_LINE_RTC_ALARM = 17,
	EXTI_LINE_RTC_TAMPER_TIMESTAMP = 19,
	EXTI_LINE_RTC_WAKEUP_TIMER = 20,
	EXTI_LINE_COMP1 = 21,
	EXTI_LINE_COMP2 = 22,
	EXTI_LINE_I2C1 = 23,
	EXTI_LINE_I2C3 = 24,
	EXTI_LINE_USART1 = 25,
	EXTI_LINE_USART2 = 26,
	EXTI_LINE_LPUART1 = 28,
	EXTI_LINE_LPTIM1 = 29,
	EXTI_LINE_LAST
} EXTI_line_t;

/*!******************************************************************
 * \enum EXTI_trigger_t
 * \brief EXTI trigger modes.
 *******************************************************************/
typedef enum {
	EXTI_TRIGGER_RISING_EDGE,
	EXTI_TRIGGER_FALLING_EDGE,
	EXTI_TRIGGER_ANY_EDGE,
	EXTI_TRIGGER_LAST
} EXTI_trigger_t;

/*!******************************************************************
 * \fn EXTI_gpio_irq_cb_t
 * \brief EXTI GPIO callback.
 *******************************************************************/
typedef void (*EXTI_gpio_irq_cb_t)(void);

/*** EXTI functions ***/

/*!******************************************************************
 * \fn void EXTI_init(void)
 * \brief Init EXTI driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void EXTI_init(void);

/*!******************************************************************
 * \fn void EXTI_configure_line(EXTI_line_t line, EXTI_trigger_t trigger)
 * \brief Configure EXTI line interrupt.
 * \param[in]  	line: Line to configure as interrupt input.
 * \param[in]	trigger: GPIO edge trigger.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void EXTI_configure_line(EXTI_line_t line, EXTI_trigger_t trigger);

/*!******************************************************************
 * \fn void EXTI_release_line(EXTI_line_t line)
 * \brief Release EXTI line interrupt.
 * \param[in]  	line: Line to release.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void EXTI_release_line(EXTI_line_t line);

/*!******************************************************************
 * \fn void EXTI_clear_line_flag(EXTI_line_t line)
 * \brief Clear EXTI line flag.
 * \param[in]  	line: Line to clear.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void EXTI_clear_line_flag(EXTI_line_t line);

#if (STM32L0XX_DRIVERS_EXTI_GPIO_MASK != 0)
/*!******************************************************************
 * \fn void EXTI_configure_gpio(const GPIO_pin_t* gpio, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback, uint8_t nvic_priority)
 * \brief Configure EXTI GPIO interrupt.
 * \param[in]  	gpio: GPIO to configure as interrupt input.
 * \param[in]	trigger: Edge trigger selection.
 * \param[in]	irq_callback: Function to call on GPIO interrupt.
 * \param[in]	nvic_priority: Interrupt priority.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void EXTI_configure_gpio(const GPIO_pin_t* gpio, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback, uint8_t nvic_priority);
#endif

#if (STM32L0XX_DRIVERS_EXTI_GPIO_MASK != 0)
/*!******************************************************************
 * \fn void EXTI_release_gpio(const GPIO_pin_t* gpio)
 * \brief Release GPIO external interrupt.
 * \param[in]  	gpio: GPIO to release.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void EXTI_release_gpio(const GPIO_pin_t* gpio);
#endif

#if (STM32L0XX_DRIVERS_EXTI_GPIO_MASK != 0)
/*!******************************************************************
 * \fn void EXTI_clear_gpio_flag(const GPIO_pin_t* gpio)
 * \brief Clear EXTI GPIO flag.
 * \param[in]  	gpio: GPIO to clear.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void EXTI_clear_gpio_flag(const GPIO_pin_t* gpio);
#endif

#if (STM32L0XX_DRIVERS_EXTI_GPIO_MASK != 0)
/*!******************************************************************
 * \fn void EXTI_enable_gpio_interrupt(const GPIO_pin_t* gpio)
 * \brief Enable GPIO external interrupt.
 * \param[in]  	gpio: GPIO to enable.
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void EXTI_enable_gpio_interrupt(const GPIO_pin_t* gpio);
#endif

#if (STM32L0XX_DRIVERS_EXTI_GPIO_MASK != 0)
/*!******************************************************************
 * \fn void EXTI_disable_gpio_interrupt(const GPIO_pin_t* gpio)
 * \brief Disable GPIO external interrupt.
 * \param[in]  	gpio: GPIO to disable.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
void EXTI_disable_gpio_interrupt(const GPIO_pin_t* gpio);
#endif

#endif /* __EXTI_H__ */
