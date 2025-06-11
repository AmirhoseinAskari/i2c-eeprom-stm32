/**
 * @file    stm32_i2c_memory_config.h
 * @author  Amirhossein Askari
 * @version 1.1.0
 * @date    2025-05-27
 * @email   theamiraskarii@gmail.com
 * @see     https://github.com/AmirhoseinAskari
 * @brief   Configuration header for I²C EEPROM memory driver.
 *
 * @details
 * This file allows selecting the STM32 MCU series.
 * Modify the macros below to fit your specific MCU.
 *
 * @warning
 * Incorrect series selection may cause HAL headers not to match actual hardware.
 * Always ensure the selected macro reflects the correct STM32 device.
 * 
 * @par Supported Platforms
 * STM32 (F0, F1, F2, F3, F4, F7, G0, G4, H7, L0, L1, L4, L5, U0, U5 series)
 */


#ifndef  STM32_I2C_MEMORY_CONFIG_H
#define  STM32_I2C_MEMORY_CONFIG_H


/* ------------------------------------------------------------------------------------- */
/*                               Select STM32 Series (REQUIRED)                          */
/* ------------------------------------------------------------------------------------- */

/**
 * @brief Select the STM32 microcontroller series used in your project.
 *
 * Define exactly one macro corresponding to your STM32 MCU series before
 * including any STM32 HAL headers or using the I2C memory driver.
 *
 * Possible values:
 *   _STM32F0, _STM32F1, _STM32F2, _STM32F3, _STM32F4, _STM32F7,
 *   _STM32G0, _STM32G4, _STM32H7,
 *   _STM32L0, _STM32L1, _STM32L4, _STM32L5,
 *   _STM32U0, _STM32U5
 */

#define  _STM32F1  


#endif  /* STM32_I2C_MEMORY_CONFIG_H */
