/**
 * @file    stm32_i2c_memory.h
 * @author  Amirhossein Askari
 * @version 1.0.0
 * @date    2025-05-27
 * @email   theamiraskarii@gmail.com
 * @see     https://github.com/AmirhoseinAskari
 * @brief   STM32 HAL header selection and I²C memory access helper functions.
 *
 * @details
 * This header conditionally includes the appropriate STM32 HAL driver header
 * based on the defined STM32 series macro (e.g., STM32F1, STM32L4).
 * It also provides inline wrappers for common I²C memory operations and
 * GPIO control using STM32 HAL drivers.
 * 
 * @note
 * Exactly one STM32 series macro must be defined in the project configuration
 * before including this header to avoid build errors.
 *
 * @warning
 * Ensure that the defined STM32 series macro matches the actual MCU on your hardware.
 */


#ifndef  STM32_I2C_MEMORY_H
#define  STM32_I2C_MEMORY_H


/* ------------------------------------- Includes ------------------------------------- */

#include "stm32_i2c_memory_config.h"  /**< STM32 series selection */


/**
 * @brief Include the STM32 HAL header based on the selected MCU series.
 *
 * This conditional compilation block selects the appropriate STM32 HAL driver
 * header file according to the STM32 series macro defined in 
 * stm32_i2c_memory_config.h.
 *
 * @note
 * Exactly one of the `_STM32xx` macros must be defined to ensure correct HAL inclusion.
 *
 * @error
 * Compilation will fail with an error if no supported STM32 series macro is defined.
 */
#if defined (_STM32F0)   
    #include "stm32f0xx_hal.h"   
#elif defined (_STM32F1)
    #include "stm32f1xx_hal.h"  
#elif defined (_STM32F2)
    #include "stm32f2xx_hal.h" 
#elif defined (_STM32F3)
    #include "stm32f3xx_hal.h"  
#elif defined (_STM32F4)
    #include "stm32f4xx_hal.h"  
#elif defined (_STM32F7)
    #include "stm32f7xx_hal.h"   
#elif defined (_STM32G0)
    #include "stm32g0xx_hal.h" 
#elif defined (_STM32G4)
    #include "stm32g4xx_hal.h"   
#elif defined (_STM32H7)
    #include "stm32h7xx_hal.h" 
#elif defined (_STM32L0)
    #include "stm32l0xx_hal.h" 
#elif defined (_STM32L1)
    #include "stm32l1xx_hal.h" 
#elif defined (_STM32L4)
    #include "stm32l4xx_hal.h"   
#elif defined (_STM32L5)
    #include "stm32l5xx_hal.h" 
#elif defined (_STM32U0)
    #include "stm32u0xx_hal.h" 
#elif defined (_STM32U5)
    #include "stm32u5xx_hal.h" 
#else                             
    #error "STM32 microcontroller not supported. Define one of the _STM32xx macros in stm32_i2c_memory_config.h."
#endif 


/* ---------------------------------- Compiler Support -------------------------------- */

/**
 * @brief Suppress 'unused function' warnings for supported compilers.
 *
 * This directive suppresses warnings about unused static or inline functions,
 * which can occur depending on compilation units or configuration settings.
 * 
 * Supported compilers:
 * - IAR Embedded Workbench (ICCARM)
 * - GCC (GNU Compiler Collection)
 *
 * @note
 * If using an unsupported compiler, a compilation error will be triggered.
 */
#if defined (__ICCARM__)             
    #pragma diag_suppress = Pe177                     
#elif defined (__GNUC__)   
    #pragma GCC diagnostic ignored "-Wunused-function"     
#else
    #error "Unsupported compiler for STM32 HAL I2C memory wrapper"  
#endif


/* -------------------------------------- Defines ------------------------------------- */

/**
 * @brief I2C memory address size definitions mapped to STM32 HAL values.
 */
#define  I2C_MEM_ADD_SIZE_8BIT   I2C_MEMADD_SIZE_8BIT
#define  I2C_MEM_ADD_SIZE_16BIT  I2C_MEMADD_SIZE_16BIT

/**
 * @brief GPIO pin state definitions mapped to STM32 HAL GPIO pin states.
 */
#define  I2C_MEM_GPIO_SET    GPIO_PIN_SET
#define  I2C_MEM_GPIO_RESET  GPIO_PIN_RESET


/* ------------------------------------- Functions ------------------------------------ */

/**
 * @brief Write data to an I2C memory device using STM32 HAL driver.
 *
 * This function is a thin wrapper around the HAL_I2C_Mem_Write API, providing
 * a convenient interface for writing data to an EEPROM or similar I2C memory device.
 *
 * @param[in]  hi2c        Pointer to the initialized I2C handle.
 * @param[in]  devAdd      7-bit I2C device address (left-aligned).
 * @param[in]  memAdd      Memory address within the EEPROM to write to.
 * @param[in]  memAddSize  Size of the memory address in bytes (e.g., I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT).
 * @param[in]  pData       Pointer to the buffer containing data to write.
 * @param[in]  dataSize    Number of bytes to write from pData.
 * @param[in]  timeout     Timeout duration in milliseconds for the write operation.
 *
 * @return HAL status code (HAL_OK, HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT).
 */
static inline HAL_StatusTypeDef i2c_mem_write(
    I2C_HandleTypeDef *hi2c,
    uint16_t devAdd,
    uint16_t memAdd,
    uint16_t memAddSize,
    const uint8_t *pData,
    uint16_t dataSize,
    uint32_t timeout)
{
    return HAL_I2C_Mem_Write(hi2c, devAdd, memAdd, memAddSize, (uint8_t *)pData, dataSize, timeout);
}

/**
 * @brief Read data from an I2C memory device using STM32 HAL driver.
 *
 * This function wraps the HAL_I2C_Mem_Read API to read data from an EEPROM
 * or similar I2C memory device.
 *
 * @param[in]  hi2c        Pointer to the initialized I2C handle.
 * @param[in]  devAdd      7-bit I2C device address (left-aligned).
 * @param[in]  memAdd      Memory address within the EEPROM to read from.
 * @param[in]  memAddSize  Size of the memory address in bytes (e.g., I2C_MEMADD_SIZE_8BIT or I2C_MEMADD_SIZE_16BIT).
 * @param[out] pData       Pointer to the buffer where the read data will be stored.
 * @param[in]  dataSize    Number of bytes to read into pData.
 * @param[in]  timeout     Timeout duration in milliseconds for the read operation.
 *
 * @return HAL status code (HAL_OK, HAL_ERROR, HAL_BUSY, or HAL_TIMEOUT).
 */
static inline HAL_StatusTypeDef i2c_mem_read(
    I2C_HandleTypeDef *hi2c,
    uint16_t devAdd,
    uint16_t memAdd,
    uint16_t memAddSize,
    uint8_t *pData,
    uint16_t dataSize,
    uint32_t timeout)
{
    return HAL_I2C_Mem_Read(hi2c, devAdd, memAdd, memAddSize, pData, dataSize, timeout);
}

/**
 * @brief Set the state of a GPIO pin using STM32 HAL.
 *
 * This function sets the specified GPIO pin to the desired logical state
 * (HIGH or LOW) via the STM32 HAL GPIO driver.
 *
 * @param[in] port      GPIO port base address (e.g., GPIOA, GPIOB, ...).
 * @param[in] pin       GPIO pin number (e.g., GPIO_PIN_0, GPIO_PIN_1, ...).
 * @param[in] pinState  Desired pin state: GPIO_PIN_SET or GPIO_PIN_RESET.
 */
static inline void i2c_mem_gpio_write_pin(GPIO_TypeDef *const port, uint16_t pin, GPIO_PinState pinState)
{
    HAL_GPIO_WritePin(port, pin, pinState);
}

/**
 * @brief Delay execution for a specified number of milliseconds using STM32 HAL.
 *
 * This function wraps the HAL_Delay function to block execution
 * for the specified duration in milliseconds.
 *
 * @param[in] ms Duration of the delay in milliseconds.
 *
 * @note This is a blocking delay; avoid using in time-critical code.
 */
static inline void i2c_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}


#endif  /* STM32_I2C_MEMORY_H */
