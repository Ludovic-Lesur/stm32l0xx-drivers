#
# CMakeLists.txt
#
#  Created on: 09 dec. 2025
#      Author: Ludo
#

# Minimum CMake version.
cmake_minimum_required(VERSION 3.23)

# Project creation.
project(stm32l0xx-drivers)

# Build mode.
if(NOT DEFINED BUILD_MODE)
    set(BUILD_MODE "STATIC")
endif()

# Create library.
add_library(${PROJECT_NAME} ${BUILD_MODE})

# Static library.
if(${BUILD_MODE} STREQUAL STATIC)

    # Hardware specific settings.
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m0plus -O0")

    # Macro to convert options to variables.
    macro(add_compilation_flag FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
        option(FLAG_NAME FLAG_DESCRIPTION FLAG_DEFAULT_VALUE)
        set(${FLAG_NAME} ${FLAG_DEFAULT_VALUE} CACHE STRING ${FLAG_DESCRIPTION})
        list(APPEND COMPILATION_FLAGS_LIST ${FLAG_NAME})
    endmacro()
    
    # Compilation flags.
    add_compilation_flag(STM32L0XX_DRIVERS_DISABLE "Disable the STM32L0xx drivers." OFF)
    add_compilation_flag(STM32L0XX_DRIVERS_DMA_CHANNEL_MASK "7-bits field which defines the enabled DMA channels." 0x7F)
    add_compilation_flag(STM32L0XX_DRIVERS_EXTI_GPIO_MASK "16-bits field which defines the enabled EXTI GPIO lines." 0xFFFF)
    add_compilation_flag(STM32L0XX_DRIVERS_I2C_FAST_MODE "Enable or disable the I2C fast mode (400kHz SCL frequency)." OFF)
    add_compilation_flag(STM32L0XX_DRIVERS_LPUART_RS485 "Enable or disable RS485 operation." OFF)
    add_compilation_flag(STM32L0XX_DRIVERS_LPUART_DISABLE_TX_0 "Disable the transmission of byte 0x00 if defined." ON)
    add_compilation_flag(STM32L0XX_DRIVERS_RCC_HSE_ENABLE "Enable or disable external oscillator functions." OFF)
    add_compilation_flag(STM32L0XX_DRIVERS_RCC_HSE_FREQUENCY_HZ "Defines the external high speed crystal frequency in Hz (if used)." 16000000)
    add_compilation_flag(STM32L0XX_DRIVERS_RCC_LSE_MODE "LSE crystal mode: 0 = disabled, 1 = enabled with LSI/HSI fallback, 2 = enabled and mandatory." 1)
    add_compilation_flag(STM32L0XX_DRIVERS_RCC_LSE_FREQUENCY_HZ "Defines the external low speed crystal frequency in Hz (if used)." 32768)
    add_compilation_flag(STM32L0XX_DRIVERS_RTC_WAKEUP_PERIOD_SECONDS "RTC wakeup period in seconds." 10)
    add_compilation_flag(STM32L0XX_DRIVERS_RTC_ALARM_MASK "2-bits field which defines the enabled RTC alarms." 0x03)
    add_compilation_flag(STM32L0XX_DRIVERS_TIM_MODE_MASK "5-bits field which defines the enabled timer operation modes: 0 = standard, 1 = multi-channel, 2 = calibration, 3 = PWM, 4 = one pulse." 0x7F)
    add_compilation_flag(STM32L0XX_DRIVERS_USART_DISABLE_TX_0 "Disable the transmission of byte 0x00 if defined." ON)
    
    # Remove OFF flags from list and keep flags set to value 0.
    foreach(FLAG ${COMPILATION_FLAGS_LIST})
        if((NOT ${FLAG} STREQUAL OFF) AND (NOT ${FLAG} STREQUAL ON))
            set(${FLAG} "(${${FLAG}})")
        endif()
    endforeach()   
    
    # Create flags file.
    configure_file(stm32l0xx_drivers_flags.h.in stm32l0xx_drivers_flags.h @ONLY)
    
    # Fixed compilation flags of dependencies.
    if(${BUILD_MODE} STREQUAL STATIC)
        target_compile_definitions(${PROJECT_NAME}
            PRIVATE
                STM32L0XX_REGISTERS_DISABLE_FLAGS_FILE
                EMBEDDED_UTILS_DISABLE_FLAGS_FILE
        )
    endif()
    
    # Dependencies folders.
    target_include_directories(${PROJECT_NAME}
        PUBLIC
            ${CMAKE_CURRENT_BINARY_DIR}
            ${TYPES_PATH}
            ${STM32L0XX_REGISTERS_PATH}/inc
            ${EMBEDDED_UTILS_PATH}/inc
    )
    
    # Print archive size.
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD 
        COMMAND ${CMAKE_SIZE_UTIL} -t lib${PROJECT_NAME}.a
    )
    
endif()

# Source files list.
target_sources(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/adc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/aes.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/dma.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/exti.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/fault.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/flash.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gpio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/i2c.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/iwdg.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/lptim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/lpuart.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/nvic.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/nvm.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/pwr.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/rcc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/rtc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/spi.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/usart.c
)

# Header files folder.
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)