/**
 * @file    i2c_memory.c
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


/* ------------------------------------- Includes ------------------------------------- */

#include "i2c_memory.h"  /**< Header file declaring the EEPROM memory driver functions */


/* ------------------------------------- Variables ------------------------------------ */

/**
 * @brief Array of EEPROM memory sizes in bytes.
 *
 * This constant array contains the total memory capacity for each supported EEPROM device.
 * The index corresponds to the values of the `I2C_MemoryCodeTypeDef` enumeration.
 */
static const uint32_t MemorySizeList[11U] = {128U, 256U, 512U, 1024U, 2048U, 4096U, 8192U, 16384U, 32768U, 65536U, 131072U};

/**
 * @brief Array of EEPROM page sizes in bytes.
 *
 * This constant array defines the write page size for each supported EEPROM device.
 * The index corresponds to the values of the `I2C_MemoryCodeTypeDef` enumeration.
 */
static const uint16_t PageSizeList[11U] = {8U, 8U, 16U, 16U, 16U, 32U, 32U, 64U, 64U, 128U, 256U};


/* ------------------------------------- Functions ------------------------------------ */

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
void I2C_Memory_Init(I2C_MemoryTypeDef *pMemory)
{      
    /* Enable write protection by setting the WP pin high */
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_SET);
        
    /* Compute the device address from base addressand A2, A1, A0 pin states */   
    pMemory->DevAddress = I2C_MEM_DEV_ADD_BASE |
                          (pMemory->A2_PinState << I2C_MEM_DEV_ADD_A2_BIT) |
                          (pMemory->A1_PinState << I2C_MEM_DEV_ADD_A1_BIT) |
                          (pMemory->A0_PinState << I2C_MEM_DEV_ADD_A0_BIT);
    
    /* Set memory size and page size according to device code */   
    pMemory->MemorySize = MemorySizeList[pMemory->MemoryCode];
    pMemory->PageSize = PageSizeList[pMemory->MemoryCode];       
}

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
I2C_Memory_StatusTypeDef I2C_Memory_SingleWrite(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t data, uint32_t timeout)
{     
    I2C_Memory_StatusTypeDef status;
    
    /* Disable write protection by setting the WP pin low */
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_RESET);
    i2c_delay_ms(I2C_MEM_STC_MS);
  
    /* Handle device address calculation and write for AT24C04 */
    if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C04])
    {
        pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_8bit(pMemory->DevAddress, address);
        status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, address, I2C_MEM_ADD_SIZE_8BIT, &data, 1U, timeout);
    }  
    
    /* Handle device address calculation and write for AT24C08 */
    else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C08])
    {
        pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_P1_8bit(pMemory->DevAddress, address);
        status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, address, I2C_MEM_ADD_SIZE_8BIT, &data, 1U, timeout);
    }  
    
    /* Handle device address calculation and write for AT24C16 */
    else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C16])
    {
        pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_P1_P2_8bit(pMemory->DevAddress, address);
        status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, address, I2C_MEM_ADD_SIZE_8BIT, &data, 1U, timeout);
    } 
    
    /* Handle device address calculation and write for AT24CM01 */ 
    else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24CM01])
    {
        pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_16bit(pMemory->DevAddress, address);
        status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, address, I2C_MEM_ADD_SIZE_16BIT, &data, 1U, timeout);      
    }  
       
    /* AT24C01, AT24C02, AT24C32, AT24C64, AT24C128, AT24C256, AT24C512 */
    else
    {
        status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, address, (pMemory->MemorySize > UINT8_MAX ? I2C_MEM_ADD_SIZE_16BIT : I2C_MEM_ADD_SIZE_8BIT), &data, 1U, timeout);
    }
        
    /* Enable write protection by setting the WP pin high */	    
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_SET); 
       
    return status;          
}

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
I2C_Memory_StatusTypeDef I2C_Memory_BurstWrite(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t size, uint32_t timeout)
{   
    I2C_Memory_StatusTypeDef status;
    uint32_t current_address = address;
    uint32_t remaining_bytes = size;    
    uint16_t page_number = current_address / pMemory->PageSize;
    uint32_t byte_index = current_address - (page_number * pMemory->PageSize);
    uint32_t bytes_to_write = pMemory->PageSize - byte_index;
       
    /* Disable write protection by setting the WP pin low */
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_RESET);
    i2c_delay_ms(I2C_MEM_STC_MS);
       
    while (remaining_bytes > 0U)
    {
        /* Determine how many bytes to write in the current page */
        bytes_to_write = bytes_to_write > remaining_bytes ? remaining_bytes : bytes_to_write;
        
        /* Handle device address calculation and write for AT24C04 */              
        if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C04])
        {
            pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_8bit(pMemory->DevAddress, current_address);
            status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, current_address, I2C_MEM_ADD_SIZE_8BIT, pData, bytes_to_write, timeout);
        } 
        
        /* Handle device address calculation and write for AT24C08 */  
        else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C08])
        {
            pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_P1_8bit(pMemory->DevAddress, current_address);
            status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, current_address, I2C_MEM_ADD_SIZE_8BIT, pData, bytes_to_write, timeout);
        }  
        
        /* Handle device address calculation and write for AT24C16 */ 
        else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C16])
        {
            pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_P1_P2_8bit(pMemory->DevAddress, current_address);
            status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, current_address, I2C_MEM_ADD_SIZE_8BIT, pData, bytes_to_write, timeout);
        } 
        
        /* Handle device address calculation and write for AT24CM01 */
        else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24CM01])
        {
            pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_16bit(pMemory->DevAddress, current_address);
            status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, current_address, I2C_MEM_ADD_SIZE_16BIT, pData, bytes_to_write, timeout);      
        }  
        
        /* AT24C01, AT24C02, AT24C32, AT24C64, AT24C128, AT24C256, AT24C512 */ 
        else
        {
            status = (I2C_Memory_StatusTypeDef) i2c_mem_write(&pMemory->I2Cx, pMemory->DevAddress, current_address, (pMemory->MemorySize > UINT8_MAX ? I2C_MEM_ADD_SIZE_16BIT : I2C_MEM_ADD_SIZE_8BIT), pData, bytes_to_write, timeout);
        }
        
        /* Wait for EEPROM internal write cycle to complete */
        i2c_delay_ms(I2C_MEM_STC_MS);
        
        /* Update pointers and counters for next iteration */
        pData += bytes_to_write;
        remaining_bytes -= bytes_to_write;
        current_address += bytes_to_write;
        page_number = current_address / pMemory->PageSize;
        byte_index = current_address - (page_number * pMemory->PageSize);
        bytes_to_write = pMemory->PageSize - byte_index;
    }
    
    /* Enable write protection by setting the WP pin hig */
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_SET);

    return status; 
}

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
I2C_Memory_StatusTypeDef I2C_Memory_SingleRead(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t timeout)
{  
    I2C_Memory_StatusTypeDef status;
       
    /* Disable write protection by setting the WP pin low */
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_RESET);
    i2c_delay_ms(I2C_MEM_STC_MS);
    
    /* Handle device address calculation and write for AT24C04 */
    if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C04])
    {
        pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_8bit(pMemory->DevAddress, address);
        status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, address, I2C_MEM_ADD_SIZE_8BIT, pData, 1U, timeout);
    } 
    
    /* Handle device address calculation and write for AT24C08 */
    else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C08])
    {
        pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_P1_8bit(pMemory->DevAddress, address);
        status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, address, I2C_MEM_ADD_SIZE_8BIT, pData, 1U, timeout);
    } 
    
    /* Handle device address calculation and write for AT24C16 */
    else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C16])
    {
        pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_P1_P2_8bit(pMemory->DevAddress, address);
        status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, address, I2C_MEM_ADD_SIZE_8BIT, pData, 1U, timeout);
    } 
    
    /* Handle device address calculation and write for AT24CM01 */
    else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24CM01])
    {
        pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_16bit(pMemory->DevAddress, address);
        status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, address, I2C_MEM_ADD_SIZE_16BIT, pData, 1U, timeout);      
    }  
    
    /* AT24C01, AT24C02, AT24C32, AT24C64, AT24C128, AT24C256, AT24C512 */
    else
    {
        status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, address, (pMemory->MemorySize > UINT8_MAX ? I2C_MEM_ADD_SIZE_16BIT : I2C_MEM_ADD_SIZE_8BIT), pData, 1U, timeout);
    }
        
    /* Enable write protection by setting the WP pin high */
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_SET); 
       
    return status;   
}

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
I2C_Memory_StatusTypeDef I2C_Memory_BurstRead(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t size, uint32_t timeout)
{   
    I2C_Memory_StatusTypeDef status;
    uint32_t current_address = address;
    uint32_t remaining_bytes = size;    
    uint16_t page_number = current_address / pMemory->PageSize;
    uint32_t byte_index = current_address - (page_number * pMemory->PageSize);
    uint32_t bytes_to_read = pMemory->PageSize - byte_index;
       
    /* Disable write protection by setting the WP pin low */
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_RESET);
    i2c_delay_ms(I2C_MEM_STC_MS);
       
    while (remaining_bytes > 0U)
    {
        /* Ensure read size does not cross the page boundary or exceed remaining bytes */
        bytes_to_read = bytes_to_read > remaining_bytes ? remaining_bytes : bytes_to_read;
                        
        /* Handle device address calculation and write for AT24C04 */
        if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C04])
        {
            pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_8bit(pMemory->DevAddress, current_address);
            status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, current_address, I2C_MEM_ADD_SIZE_8BIT, pData, bytes_to_read, timeout);
        }  
        
        /* Handle device address calculation and write for AT24C08 */ 
        else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C08])
        {
            pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_P1_8bit(pMemory->DevAddress, current_address);
            status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, current_address, I2C_MEM_ADD_SIZE_8BIT, pData, bytes_to_read, timeout);
        }  
        
        /* Handle device address calculation and write for AT24C16 */ 
        else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24C16])
        {
            pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_P1_P2_8bit(pMemory->DevAddress, current_address);
            status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, current_address, I2C_MEM_ADD_SIZE_8BIT, pData, bytes_to_read, timeout);
        }  
        
        /* Handle device address calculation and write for AT24CM01 */ 
        else if (pMemory->MemorySize == MemorySizeList[I2C_MEM_CODE_AT24CM01])
        {
            pMemory->DevAddress = I2C_Mem_CalcDevAddr_P0_16bit(pMemory->DevAddress, current_address);
            status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, current_address, I2C_MEM_ADD_SIZE_16BIT, pData, bytes_to_read, timeout);      
        } 
        
        /* AT24C01, AT24C02, AT24C32, AT24C64, AT24C128, AT24C256, AT24C512 */  
        else
        {
            status = (I2C_Memory_StatusTypeDef) i2c_mem_read(&pMemory->I2Cx, pMemory->DevAddress, current_address, (pMemory->MemorySize > UINT8_MAX ? I2C_MEM_ADD_SIZE_16BIT : I2C_MEM_ADD_SIZE_8BIT), pData, bytes_to_read, timeout);
        }
        
        /* Wait for EEPROM internal write cycle time before next read chunk */
        i2c_delay_ms(I2C_MEM_STC_MS);
        
        /* Update pointers and counters for next iteration */
        pData += bytes_to_read;
        remaining_bytes -= bytes_to_read;
        current_address += bytes_to_read;
        page_number = current_address / pMemory->PageSize;
        byte_index = current_address - (page_number * pMemory->PageSize);
        bytes_to_read = pMemory->PageSize - byte_index;
    }
    
    /* Enable write protection by setting the WP pin high */
    i2c_mem_gpio_write_pin(pMemory->WP_GPIO, pMemory->WP_Pin, I2C_MEM_GPIO_SET);

    return status;   
}


/* i2c_memory.c */
