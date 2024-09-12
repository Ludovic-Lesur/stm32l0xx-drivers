# Description

This repository contains the **peripherals drivers** of the STM32L0xx MCUs.

# Dependencies

The drivers rely on:

* The **STM32L0xx registers** defined in the [stm32l0xx-registers](https://github.com/Ludovic-Lesur/stm32l0xx-registers) repository.
* The **embedded utility functions** defined in the [embedded-utils](https://github.com/Ludovic-Lesur/embedded-utils) repository.

Here is the versions compatibility table:

| **stm32l0xx-drivers** | **stm32l0xx-registers** | **embedded-utils** |
|:---:|:---:|:---:|
| [sw2.0](https://github.com/Ludovic-Lesur/stm32l0xx-drivers/releases/tag/sw2.0) | >= [sw1.1](https://github.com/Ludovic-Lesur/stm32l0xx-registers/releases/tag/sw1.1) | >= [sw2.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw2.0) |
| [sw1.2](https://github.com/Ludovic-Lesur/stm32l0xx-drivers/releases/tag/sw1.2) | >= [sw1.1](https://github.com/Ludovic-Lesur/stm32l0xx-registers/releases/tag/sw1.1) | [sw1.3](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.3) to [sw1.4](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.4) |
| [sw1.1](https://github.com/Ludovic-Lesur/stm32l0xx-drivers/releases/tag/sw1.1) | >= [sw1.1](https://github.com/Ludovic-Lesur/stm32l0xx-registers/releases/tag/sw1.1) |  [sw1.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.0) to [sw1.2](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.2) |
| [sw1.0](https://github.com/Ludovic-Lesur/stm32l0xx-drivers/releases/tag/sw1.0) | >= [sw1.1](https://github.com/Ludovic-Lesur/stm32l0xx-registers/releases/tag/sw1.1) | [sw1.0](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.0) to [sw1.2](https://github.com/Ludovic-Lesur/embedded-utils/releases/tag/sw1.2)

# Compilation flags

| **Flag name** | **Value** | **Description** |
|:---:|:---:|:---:|
| `STM32L0XX_DRIVERS_DISABLE_FLAGS_FILE` | `defined` / `undefined` | Disable the `stm32l0xx_drivers_flags.h` header file inclusion when compilation flags are given in the project settings or by command line. |
| `STM32L0XX_DRIVERS_DMA_CHANNEL_MASK` | `0x00` to `0x7F` | 7-bits field which defines the enabled DMA channels. |
| `STM32L0XX_DRIVERS_EXTI_GPIO_MASK` | `0x0000` to `0xFFFF` | 16-bits field which defines the enabled EXTI GPIO lines. |
| `STM32L0XX_DRIVERS_LPUART_MODE` | `0` / `1` / `2` / `3` | LPUART operation mode: `0` = RXNE interrupt `1` = CMF interrupt `2` = RS485 slave mode `3` = RS485 master mode. |
| `STM32L0XX_DRIVERS_LPUART_DISABLE_TX_0` | `defined` / `undefined` | Disable the transmission of byte 0x00 if defined. |
| `STM32L0XX_DRIVERS_RCC_HSE_ENABLE` | `defined` / `undefined` | Enable or disable external oscillator functions. |
| `STM32L0XX_DRIVERS_RCC_HSE_FREQUENCY_HZ` | `<value>` | Defines the external high speed crystal frequency in Hz (if used). |
| `STM32L0XX_DRIVERS_RCC_LSE_MODE` | `0` / `1` / `2` | LSE crystal mode: `0` = disabled `1` = enabled with LSI/HSI fallback `2` = enabled and mandatory. |
| `STM32L0XX_DRIVERS_RCC_LSE_FREQUENCY_HZ` | `<value>` | Defines the external low speed crystal frequency in Hz (if used). |
| `STM32L0XX_DRIVERS_RTC_WAKEUP_PERIOD_SECONDS` | `<value>` | RTC wakeup period in seconds. |
| `STM32L0XX_DRIVERS_RTC_ALARM_MASK` | `0x00` to `0x03`| 2-bits field which defines the enabled RTC alarms. |
| `STM32L0XX_DRIVERS_TIM_MODE_MASK` | `0x00` to `0x1F`| 5-bits field which defines the enabled timer operation modes: `0` = standard `1` = multi-channel `2` = calibration `3` = PWM `4` = one pulse. |
| `STM32L0XX_DRIVERS_USART_MODE` | `0` / `1` | USART operation mode: `0` = RXNE interrupt `1` = CMF interrupt. |
| `STM32L0XX_DRIVERS_USART_DISABLE_TX_0` | `defined` / `undefined` | Disable the transmission of byte 0x00 if defined. |


