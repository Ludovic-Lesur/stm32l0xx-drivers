/*
 * nvic.h
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#ifndef __NVIC_H__
#define __NVIC_H__

#include "types.h"

/*** NVIC structures ***/

/*!******************************************************************
 * \enum NVIC_interrupt_t
 * \brief NVIC interrupt vector.
 *******************************************************************/
typedef enum {
    NVIC_INTERRUPT_WWDG = 0,
    NVIC_INTERRUPT_PVD = 1,
    NVIC_INTERRUPT_RTC = 2,
    NVIC_INTERRUPT_FLASH = 3,
    NVIC_INTERRUPT_RCC_CRS = 4,
    NVIC_INTERRUPT_EXTI_0_1 = 5,
    NVIC_INTERRUPT_EXTI_2_3 = 6,
    NVIC_INTERRUPT_EXTI_4_15 = 7,
    NVIC_INTERRUPT_DMA1_CH_1 = 9,
    NVIC_INTERRUPT_DMA1_CH_2_3 = 10,
    NVIC_INTERRUPT_DMA1_CH_4_7 = 11,
    NVIC_INTERRUPT_ADC_COMP = 12,
    NVIC_INTERRUPT_LPTIM1 = 13,
    NVIC_INTERRUPT_USART4_USART5 = 14,
    NVIC_INTERRUPT_TIM2 = 15,
    NVIC_INTERRUPT_TIM3 = 16,
    NVIC_INTERRUPT_TIM6 = 17,
    NVIC_INTERRUPT_TIM7 = 18,
    NVIC_INTERRUPT_TIM21 = 20,
    NVIC_INTERRUPT_I2C3 = 21,
    NVIC_INTERRUPT_TIM22 = 22,
    NVIC_INTERRUPT_I2C1 = 23,
    NVIC_INTERRUPT_I2C2 = 24,
    NVIC_INTERRUPT_SPI1 = 25,
    NVIC_INTERRUPT_SPI2 = 26,
    NVIC_INTERRUPT_USART1 = 27,
    NVIC_INTERRUPT_USART2 = 28,
    NVIC_INTERRUPT_LPUART1 = 29,
    NVIC_INTERRUPT_LAST
} NVIC_interrupt_t;

/*** NVIC functions ***/

/*!******************************************************************
 * \fn void NVIC_init(void)
 * \brief Init interrupts vector.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void NVIC_init(void);

/*!******************************************************************
 * \fn void NVIC_enable_interrupt(NVIC_interrupt_t irq_index)
 * \brief Enable interrupt.
 * \param[in]   irq_index: Interrupt to enable.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void NVIC_enable_interrupt(NVIC_interrupt_t irq_index);

/*!******************************************************************
 * \fn void NVIC_disable_interrupt(NVIC_interrupt_t irq_index)
 * \brief Disable interrupt.
 * \param[in]   irq_index: Interrupt to enable.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void NVIC_disable_interrupt(NVIC_interrupt_t irq_index);

/*!******************************************************************
 * \fn void NVIC_set_priority(NVIC_interrupt_t irq_index, uint8_t priority)
 * \brief Set interrupt priority.
 * \param[in]   irq_index: Interrupt to configure.
 * \param[in]   priority: Interrupt priority to set.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void NVIC_set_priority(NVIC_interrupt_t irq_index, uint8_t priority);

#endif /* __NVIC_H__ */
