/**
 * @file    i2c_memory.h
 * @author  Amirhossein Askari
 * @version 1.0.0
 * @date    2025-05-27
 * @email   theamiraskarii@gmail.com
 * @see     https://github.com/AmirhoseinAskari
 * @brief   Portable driver for I²C-based EEPROM memory (AT24C series).
 * 
 * @details
 * This file implements a platform-agnostic driver for AT24C-series EEPROMs using the I²C protocol.
 * It abstracts low-level memory access and provides high-level APIs for single-byte and 
 * multi-byte (burst) read/write operations.
 * 
 * Each EEPROM instance is represented by an `I2C_MemoryTypeDef` structure, allowing 
 * independent handling of multiple devices on the same I²C bus.
 * 
 * @note
 * The I²C peripheral must be initialized externally (by the user) before calling any 
 * functions from this library. A typical EEPROM self-write cycle requires a minimum 
 * delay of 5 ms between write operations to the same address.
 * 
 * @warning
 * Excessive write operations to the same EEPROM address may reduce lifespan 
 * (typically around 1 million write cycles). Ensure that the EEPROM’s hardware 
 * address pin configuration (A0, A1, A2) matches the corresponding software settings 
 * to avoid bus conflicts.
 * 
 * @section Supported_Devices
 * AT24C01, AT24C02, AT24C04, AT24C08, AT24C16,  
 * AT24C32, AT24C64, AT24C128, AT24C256, AT24C512, AT24CM01
 * 
 * @section Supported_Platforms  
 * STM32 (HAL Drivers)
 */


#ifndef  I2C_MEMORY_H
#define  I2C_MEMORY_H


/* ------------------------------------- Includes ------------------------------------- */

#include <stdint.h>                      /**< Fixed-width integer types */
#include <stdbool.h>                     /**< Standard boolean type */
#include "STM32_I2C/stm32_i2c_memory.h"  /**< STM32 HAL-based I2C EEPROM driver header */


/* -------------------------------------- Defines ------------------------------------- */

/** @brief Base I2C device address for EEPROM */
#define  I2C_MEM_DEV_ADD_BASE  (0xA0U) 

/** @brief Bit positions of hardware address pins in the device address */
#define  I2C_MEM_DEV_ADD_A0_BIT  (1U)       
#define  I2C_MEM_DEV_ADD_A1_BIT  (2U)    
#define  I2C_MEM_DEV_ADD_A2_BIT  (3U)      

/** @brief EEPROM self-write cycle time in milliseconds */
#define  I2C_MEM_STC_MS  (5U)       

/** @brief Address shift value for 8-bit addressing mode */                                      
#define  SHIFT_VAL_8BIT  (0x7U)    

/** @brief Address shift value for 16-bit addressing mode */
#define  SHIFT_VAL_16BIT  (0xFU)    

/** @brief Bitmask for selecting P0 bit in the device address */
#define  P0_BIT_SELECT  (0x2U)  

/** @brief Bitmask for selecting P0 and P1 bits in the device address */
#define  P0_P1_BIT_SELECT  (0x6U)     

/** @brief Bitmask for selecting P0, P1, and P2 bits in the device address */
#define  P0_P1_P2_BIT_SELECT  (0xEU)      


/* --------------------------------------- Types -------------------------------------- */

/**
 * @brief Enumeration of supported I²C EEPROM memory device types.
 *
 * Identifies the specific AT24C-series EEPROM models supported by the driver.
 * Each enumerator corresponds to a device code used internally to configure
 * device-specific parameters such as memory size and addressing mode.
 */
typedef enum
{
    I2C_MEM_CODE_AT24C01  = 0U,  /**< AT24C01 device code */
    I2C_MEM_CODE_AT24C02  = 1U,  /**< AT24C02 device code */
    I2C_MEM_CODE_AT24C04  = 2U,  /**< AT24C04 device code */
    I2C_MEM_CODE_AT24C08  = 3U,  /**< AT24C08 device code */
    I2C_MEM_CODE_AT24C16  = 4U,  /**< AT24C16 device code */
    I2C_MEM_CODE_AT24C32  = 5U,  /**< AT24C32 device code */
    I2C_MEM_CODE_AT24C64  = 6U,  /**< AT24C64 device code */
    I2C_MEM_CODE_AT24C128 = 7U,  /**< AT24C128 device code */
    I2C_MEM_CODE_AT24C256 = 8U,  /**< AT24C256 device code */
    I2C_MEM_CODE_AT24C512 = 9U,  /**< AT24C512 device code */
    I2C_MEM_CODE_AT24CM01 = 10U  /**< AT24CM01 device code */
        
} I2C_MemoryCodeTypeDef;

/**
 * @brief Configuration and state structure for an I²C EEPROM memory device.
 *
 * This structure encapsulates all necessary information to manage an I²C-based
 * EEPROM device, including hardware handles, device addressing, and memory
 * parameters.
 */
typedef struct
{
    I2C_HandleTypeDef I2Cx;  /**< I2C handle (STM32 HAL driver) used for communication */  
    GPIO_TypeDef *WP_GPIO;   /**< GPIO port for Write Protect (WP) pin control */  
    uint16_t WP_Pin;         /**< GPIO pin number for Write Protect (WP) */
            
    I2C_MemoryCodeTypeDef MemoryCode;  /**< EEPROM device type identifier */
    
    uint8_t DevAddress;  /**< Computed base I²C device address including fixed bits */ 
    bool A0_PinState;    /**< Logical level of device address pin A0 (true = high) */   
    bool A1_PinState;    /**< Logical level of device address pin A1 (true = high) */
    bool A2_PinState;    /**< Logical level of device address pin A2 (true = high) */
    
    uint32_t MemorySize;  /**< Total memory capacity in bytes */
    uint32_t PageSize;    /**< EEPROM page size in bytes for burst write operations */
    
} I2C_MemoryTypeDef;

/**
 * @brief I²C EEPROM memory operation status codes.
 *
 * Defines return codes used by the EEPROM driver functions to indicate
 * the outcome of I²C memory operations.
 */
typedef enum
{
    I2C_MEM_STATUS_OK      = 0x00U,  /**< Operation completed successfully */
    I2C_MEM_STATUS_ERROR   = 0x01U,  /**< General failure during operation */
    I2C_MEM_STATUS_BUSY    = 0x02U,  /**< EEPROM device is busy or not ready */
    I2C_MEM_STATUS_TIMEOUT = 0x03U   /**< Operation timed out */
        
} I2C_Memory_StatusTypeDef;


/* ------------------------------------- Functions ------------------------------------ */

/**
 * @brief Calculate I²C device address for EEPROM using 8-bit memory addressing with P0 bit.
 *
 * For EEPROMs with 8-bit internal memory addressing, the device address
 * includes the P0 bit derived from the higher bits of the memory address.
 *
 * @param[in] devAddr Base 7-bit I²C device address (without R/W bit).
 * @param[in] memAddr Internal memory address within the EEPROM.
 * 
 * @return Device address including the P0 bit, ready for I²C transmission.
 */
static inline uint8_t I2C_Mem_CalcDevAddr_P0_8bit(uint8_t devAddr, uint32_t memAddr)
{
    return (uint8_t)(devAddr | ((memAddr >> SHIFT_VAL_8BIT) & P0_BIT_SELECT));
}

/**
 * @brief Calculate I²C device address for EEPROM with P0 and P1 bits set (8-bit memory addressing).
 *
 * Computes the device address by embedding the P0 and P1 bits derived from the
 * memory address for EEPROM devices that use 8-bit internal memory addressing.
 *
 * @param[in] devAddr Base 7-bit I²C device address (excluding R/W bit).
 * @param[in] memAddr Internal memory address within the EEPROM.
 * 
 * @return Device address including P0 and P1 bits, suitable for I²C communication.
 */
static inline uint8_t I2C_Mem_CalcDevAddr_P0_P1_8bit(uint8_t devAddr, uint32_t memAddr)
{
    return (uint8_t)(devAddr | ((memAddr >> SHIFT_VAL_8BIT) & P0_P1_BIT_SELECT));
}

/**
 * @brief Calculate I²C device address for EEPROM with P0, P1, and P2 bits set (8-bit memory addressing).
 *
 * Computes the device address by embedding the P0, P1, and P2 bits derived from
 * the memory address for EEPROM devices using 8-bit internal memory addressing.
 *
 * @param[in] devAddr Base 7-bit I²C device address (excluding R/W bit).
 * @param[in] memAddr Internal memory address within the EEPROM.
 * 
 * @return Device address including P0, P1, and P2 bits, suitable for I²C communication.
 */
static inline uint8_t I2C_Mem_CalcDevAddr_P0_P1_P2_8bit(uint8_t devAddr, uint32_t memAddr)
{
    return (uint8_t)(devAddr | ((memAddr >> SHIFT_VAL_8BIT) & P0_P1_P2_BIT_SELECT));
}

/**
 * @brief Calculate I²C device address for EEPROM with P0 bit set (16-bit memory addressing).
 *
 * Computes the device address by incorporating the P0 bit derived from the
 * higher bits of the memory address for EEPROM devices that use 16-bit internal memory addressing.
 *
 * @param[in] devAddr Base 7-bit I²C device address (excluding R/W bit).
 * @param[in] memAddr Internal memory address within the EEPROM.
 * 
 * @return Device address including the P0 bit, suitable for I²C communication.
 */
static inline uint8_t I2C_Mem_CalcDevAddr_P0_16bit(uint8_t devAddr, uint32_t memAddr)
{
    return (uint8_t)(devAddr | ((memAddr >> SHIFT_VAL_16BIT) & P0_BIT_SELECT));
}

/**
 * @brief Initializes an I²C EEPROM memory device instance.
 * 
 * This function configures the EEPROM instance with device-specific parameters such as 
 * page size, total memory capacity, and the computed I²C address derived from hardware 
 * address pins A0, A1, and A2.
 * 
 * @param[in,out] pMemory Pointer to an initialized @ref I2C_MemoryTypeDef structure.
 *                        The structure must be allocated and pre-configured with basic
 *                        parameters (e.g., device type and pin states) by the caller.
 * 
 * @note
 * - The caller must ensure that the I²C peripheral, GPIOs (including the write-protect pin),
 *   and other hardware components are properly initialized prior to calling this function.
 * - Only device-specific setup is handled by this function; hardware abstraction remains external.
 * 
 * @warning
 * Improper configuration of `pMemory` members (e.g., `MemoryCode`, `A0_PinState`, 
 * `A1_PinState`, `A2_PinState`, `WP_GPIO`, and `WP_Pin`) may result in incorrect I²C addressing 
 * or write protection behavior. Always validate these fields before initialization.
 */
void I2C_Memory_Init(I2C_MemoryTypeDef *pMemory);

/**
 * @brief Writes a single byte to the specified address in the EEPROM memory.
 *
 * This function transmits one byte of data to the EEPROM at the given memory address.
 * The addressing mode (8-bit or 16-bit) is automatically selected based on the EEPROM type
 * defined in the @ref I2C_MemoryTypeDef configuration.
 *
 * @param[in,out] pMemory Pointer to an initialized @ref I2C_MemoryTypeDef instance representing the target EEPROM.
 *                        Must be properly configured using @ref I2C_Memory_Init before use.
 * @param[in]     address EEPROM memory address to write the byte to.
 * @param[in]     data    The byte value to be written to memory.
 * @param[in]     timeout Timeout duration (in milliseconds) for the I²C write operation.
 *
 * @retval I2C_MEM_STATUS_OK       Write completed successfully.
 * @retval I2C_MEM_STATUS_ERROR    General error during the write operation.
 * @retval I2C_MEM_STATUS_BUSY     EEPROM is busy or not responding.
 * @retval I2C_MEM_STATUS_TIMEOUT  The I²C operation timed out.
 *
 * @note
 * A typical EEPROM write cycle requires approximately 5 ms to complete. Ensure a sufficient 
 * delay between successive write operations to maintain data integrity.
 *
 * @warning
 * Before calling this function, make sure the I²C peripheral and all related GPIO pins 
 * (including WP if used) are correctly initialized and configured.
 */
I2C_Memory_StatusTypeDef I2C_Memory_SingleWrite(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t data, uint32_t timeout);

/**
 * @brief Performs a burst (page-wise) write of multiple bytes to an EEPROM device.
 *
 * Writes a block of data to the EEPROM starting from the specified memory address.
 * The data is automatically segmented into page-sized chunks based on the EEPROM's 
 * configured page size, ensuring writes do not cross page boundaries.
 *
 * @param[in,out] pMemory Pointer to an initialized @ref I2C_MemoryTypeDef instance representing the target EEPROM.
 *                        Must be fully configured using @ref I2C_Memory_Init prior to this call.
 * @param[in]     address Starting memory address for the write operation.
 * @param[in]     pData   Pointer to the data buffer to write. Must not be NULL if @p size > 0.
 * @param[in]     size    Number of bytes to write from the buffer.
 * @param[in]     timeout Timeout (in milliseconds) applied to each individual I²C page write.
 *
 * @retval I2C_MEM_STATUS_OK       All data written successfully.
 * @retval I2C_MEM_STATUS_ERROR    A general I²C error occurred during the write operation.
 * @retval I2C_MEM_STATUS_BUSY     EEPROM is busy or not responding.
 * @retval I2C_MEM_STATUS_TIMEOUT  One or more I²C operations exceeded the timeout duration.
 *
 * @note
 * - Write operations are automatically aligned with EEPROM page boundaries to avoid data wrapping.
 * - A minimum delay of 5 ms is applied between successive page writes to satisfy the EEPROM's 
 *   internal write cycle timing.
 *
 * @warning
 * Ensure that the I²C peripheral, write-protect pin (if applicable), and related GPIOs 
 * are properly initialized and configured before invoking this function.
 */
I2C_Memory_StatusTypeDef I2C_Memory_BurstWrite(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t size, uint32_t timeout);

/**
 * @brief Reads a single byte from the EEPROM at the specified memory address.
 *
 * Performs a single-byte read operation from the EEPROM using the I²C protocol.
 * The function automatically selects the appropriate device I²C address and 
 * addressing mode (8-bit or 16-bit) based on the EEPROM type and memory address.
 *
 * @param[in,out] pMemory Pointer to an initialized @ref I2C_MemoryTypeDef instance representing the target EEPROM.
 *                        Must be fully configured using @ref I2C_Memory_Init before use.
 * @param[in]     address EEPROM memory address to read from.
 * @param[out]    pData   Pointer to a variable where the read byte will be stored.
 *                        Must not be @c NULL.
 * @param[in]     timeout Timeout duration in milliseconds for the I²C read operation.
 *
 * @retval I2C_MEM_STATUS_OK       Byte read successfully.
 * @retval I2C_MEM_STATUS_ERROR    A general I²C error occurred during the read.
 * @retval I2C_MEM_STATUS_BUSY     EEPROM is currently busy or not responding.
 * @retval I2C_MEM_STATUS_TIMEOUT  Read operation timed out.
 *
 * @note
 * Ensure that the I²C peripheral and all necessary GPIO pins are properly initialized 
 * before calling this function.
 *
 * @warning
 * Passing a @c NULL pointer to @p pData results in undefined behavior and must be avoided.
 */
I2C_Memory_StatusTypeDef I2C_Memory_SingleRead(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t timeout);

/**
 * @brief Reads multiple bytes from the EEPROM starting at a specified memory address.
 *
 * Performs a burst read operation from the EEPROM using the I²C protocol. The data is
 * read in page-aligned chunks to ensure compatibility with internal memory structures.
 * The function automatically handles page boundary calculations and selects the appropriate 
 * addressing mode based on the EEPROM model.
 *
 * @param[in,out] pMemory Pointer to an initialized @ref I2C_MemoryTypeDef instance representing the target EEPROM.
 *                        Must be fully configured using @ref I2C_Memory_Init before use.
 * @param[in]     address Starting EEPROM memory address to begin reading from.
 * @param[out]    pData   Pointer to the buffer where the read data will be stored.
 *                        Must not be @c NULL and must be at least @p size bytes in length.
 * @param[in]     size    Number of bytes to read.
 * @param[in]     timeout Timeout (in milliseconds) applied to each I²C read transaction.
 *
 * @retval I2C_MEM_STATUS_OK       All data read successfully.
 * @retval I2C_MEM_STATUS_ERROR    A general I²C error occurred during one or more read operations.
 * @retval I2C_MEM_STATUS_BUSY     EEPROM is busy or not responding.
 * @retval I2C_MEM_STATUS_TIMEOUT  One or more read operations timed out.
 *
 * @note
 * - The function internally splits reads at EEPROM page boundaries to ensure compliance
 *   with device addressing requirements.
 * - Reading beyond the physical memory size may result in wraparound or invalid data,
 *   depending on device behavior.
 *
 * @warning
 * - Ensure @p pData points to a valid buffer of at least @p size bytes.
 * - The I²C peripheral, write-protect GPIO (if used), and related hardware must be initialized.
 * - Passing invalid pointers or using uninitialized hardware results in undefined behavior.
 */
I2C_Memory_StatusTypeDef I2C_Memory_BurstRead(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t size, uint32_t timeout);


#endif /* I2C_MEMORY_H */
