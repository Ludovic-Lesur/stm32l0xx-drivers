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
void PWR_enter_sleep_mode(void) {
    // Regulator in normal mode.
    PWR->CR &= ~(0b1 << 0); // LPSDSR='0'.
    // Enter low power sleep mode.
    SCB->SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
    __asm volatile ("wfi");
    // Wait For Interrupt core instruction.
}

/*******************************************************************/
void PWR_enter_low_power_sleep_mode(void) {
    // Regulator in low power mode.
    PWR->CR |= (0b1 << 0); // LPSDSR='1'.
    // Enter low power sleep mode.
    SCB->SCR &= ~(0b1 << 2); // SLEEPDEEP='0'.
    __asm volatile ("wfi");
    // Wait For Interrupt core instruction.
}

/*******************************************************************/
void PWR_enter_stop_mode(void) {
    // Select wakeup clock.
    if (RCC_get_system_clock() == RCC_CLOCK_MSI) {
        // Use MSI.
        RCC->CFGR &= ~(0b1 << 15);
    }
    else {
        // Use HSI.
        RCC->CFGR |= (0b1 << 15);
    }
    // Regulator in low power mode.
    PWR->CR |= (0b1 << 0); // LPSDSR='1'.
    // Clear WUF flag.
    PWR->CR |= (0b1 << 2); // CWUF='1'.
    // Enter stop mode when CPU enters deepsleep.
    PWR->CR &= ~(0b1 << 1); // PDDS='0'.
    // Clear all EXTI, RTC and peripherals interrupt pending bits.
    RCC->CICR |= 0x000001BF;
    EXTI->PR |= 0x007BFFFF; // PIFx='1'.
    RTC->ISR &= 0xFFFF005F; // Reset alarms, wake-up, tamper and timestamp flags.
    NVIC->ICPR = 0xFFFFFFFF; // CLEARPENDx='1'.
    // Enter stop mode.
    SCB->SCR |= (0b1 << 2); // SLEEPDEEP='1'.
    __asm volatile ("wfi");
    // Wait For Interrupt core instruction.
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
