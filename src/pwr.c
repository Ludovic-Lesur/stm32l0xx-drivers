/*
 * pwr.c
 *
 *  Created on: 01 aug. 2024
 *      Author: Ludo
 */

#include "pwr.h"

#include "exti_registers.h"
#include "flash_registers.h"
#include "nvic_registers.h"
#include "pwr_registers.h"
#include "rcc.h"
#include "rcc_registers.h"
#include "rtc_registers.h"
#include "scb_registers.h"
#include "types.h"

/*** PWR local functions ***/

/*******************************************************************/
static void __attribute__((optimize("-O0"))) _PWR_reset_backup_domain(void) {
    // Local variables.
    uint8_t count = 0;
    // Unlock back-up registers.
    PWR->CR |= (0b1 << 8); // DBP='1'.
    // Perform manual reset and delay.
    RCC->CSR |= (0b1 << 19); // RTCRST='1'.
    for (count = 0; count < 100; count++);
    RCC->CSR &= ~(0b1 << 19); // RTCRST='0'.
}

/*** PWR functions ***/

/*******************************************************************/
void PWR_init(void) {
    // Enable power interface clock.
    RCC->APB1ENR |= (0b1 << 28); // PWREN='1'.
    // Reset backup domain.
    _PWR_reset_backup_domain();
    // Power memories down when entering sleep mode.
    FLASH->ACR |= (0b1 << 3); // SLEEP_PD='1'.
    // Switch internal voltage reference off in low power mode and ignore startup time.
    PWR->CR |= (0b11 << 9); // ULP='1' and FWU='1'.
    // Never return in low power sleep mode after wake-up.
    SCB->SCR &= ~(0b1 << 1); // SLEEPONEXIT='0'.
}

/*******************************************************************/
void PWR_de_init(void) {
    // Disable power interface clock.
    RCC->APB1ENR &= ~(0b1 << 28); // PWREN='0'.
}

/*******************************************************************/
void PWR_enter_sleep_mode(PWR_sleep_mode_t sleep_mode) {
    // Configure mode.
    switch (sleep_mode) {
    case PWR_SLEEP_MODE_NORMAL:
        // Regulator in normal mode.
        PWR->CR &= ~(0b1 << 0); // LPSDSR='0'.
        break;
    case PWR_SLEEP_MODE_LOW_POWER:
        // Regulator in low power mode.
        PWR->CR |= (0b1 << 0); // LPSDSR='1'.
        break;
    default:
        break;
    }
    // Enter sleep mode.
    SCB->SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
    // Wait For Interrupt core instruction.
    __asm volatile ("wfi");
}

/*******************************************************************/
void PWR_enter_deepsleep_mode(PWR_deepsleep_mode_t deepsleep_mode) {
    // Select wakeup clock in case of stop mode.
    if (RCC_get_system_clock() == RCC_CLOCK_MSI) {
        RCC->CFGR &= ~(0b1 << 15); // Use MSI.
    }
    else {
        RCC->CFGR |= (0b1 << 15); // Use HSI.
    }
    // Select low power mode.
    switch (deepsleep_mode) {
    case PWR_DEEPSLEEP_MODE_STOP:
        PWR->CR &= ~(0b1 << 1); // PDDS='0'.
        break;
    case PWR_DEEPSLEEP_MODE_STANDBY:
        PWR->CR |= (0b1 << 1); // PDDS='1'.
        break;
    default:
        break;
    }
    // Regulator in low power mode.
    PWR->CR |= (0b1 << 0); // LPSDSR='1'.
    // Clear wake-up flag.
    PWR->CR |= (0b1 << 2); // CWUF='1'.
    // Clear all EXTI, RTC and peripherals interrupt pending bits.
    RCC->CICR |= 0x000001BF;
    EXTI->PR |= 0x007BFFFF;
    RTC->ISR &= 0xFFFF005F;
    NVIC->ICPR = 0xFFFFFFFF;
    // Enter deep sleep mode.
    SCB->SCR |= (0b1 << 2); // SLEEPDEEP='1'.
    // Wait For Interrupt core instruction.
    __asm volatile ("wfi");
}

/*******************************************************************/
void PWR_software_reset(void) {
    // Trigger software reset.
    SCB->AIRCR = 0x05FA0000 | ((SCB->AIRCR) & 0x0000FFFF) | (0b1 << 2);
}

/*******************************************************************/
uint8_t PWR_get_reset_flags(void) {
    // Local variables.
    return ((uint8_t) (((RCC->CSR) >> 24) & 0xFF));
}

/*******************************************************************/
void PWR_clear_reset_flags(void) {
    RCC->CSR |= (0b1 << 23);
}
