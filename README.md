# stm32-i2c-eeprom

Driver for using Serial EEPROM Products (AT24C family) in STM32 microcontrollers

## 🔧 Features

- ✅ Fully compatible with STM32 HAL I²C drivers  
- 📦 Supports all standard AT24C EEPROM memory series
- 🧠 Handles multiple EEPROM devices on the same I²C bus 
- 🛡️ Built with MISRA-C coding practices for safety and portability  
- 🧱 Supports 8-bit and 16-bit memory addressing  
- 🕒 Timeout-based API for better fault tolerance  
- 🚀 Page write and burst read support for optimized performance  
- 🔌 Write-protect pin control included via GPIO abstraction  
- 🧰 Modular and portable — drop into any STM32CubeIDE or Makefile-based project 

## ✅ Requirements

- STM32Cube HAL drivers  
- Properly initialized I²C peripheral (via STM32CubeMX or manual config)  
- Compatible EEPROM from the AT24C series

## ⚙️ Configuration & Integration

1. **Configure I²C** in STM32CubeMX:  
   - Enable an I²C instance (e.g., I2C1)  
   - Set the **I²C Speed Mode** to **Standard Mode (100 kHz)**

2. **Add the Library** to your project:  
   - Include `i2c_memory.h` in your `main.c` or application code  
   - Define your MCU series in `stm32_i2c_memory_config.h` 

3. **Define EEPROM Configuration**:
   - Instantiate one or more `I2C_MemoryTypeDef` structs and assign unique addresses (A0/A1/A2 pins)

4. **Build and flash**. See `example/main.c` for usage.

## 🧪 API Reference

Each function returns a status of type `I2C_Memory_StatusTypeDef`.

### `void I2C_Memory_Init(I2C_MemoryTypeDef *pMemory)`
Initializes memory configuration, computes device address, and sets up write-protect GPIO (if used).

### `I2C_Memory_StatusTypeDef I2C_Memory_SingleWrite(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t data, uint32_t timeout)`
Writes a single byte to the given EEPROM memory address.

### `I2C_Memory_StatusTypeDef I2C_Memory_BurstWrite(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t size, uint32_t timeout)`
Writes multiple bytes (burst/page write) starting from a specified memory address.

### `I2C_Memory_StatusTypeDef I2C_Memory_SingleRead(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t timeout)`
Reads a single byte from the specified memory address.

### `I2C_Memory_StatusTypeDef I2C_Memory_BurstRead(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t size, uint32_t timeout)`
Reads multiple bytes starting from the specified memory address.

## 💡 Example
A working example is provided in [`example/main.c`](./example/main.c), showing full integration with HAL, multiple EEPROMs, and read/write testing.

## 📜 License
This project is licensed under the [MIT License](./LICENSE).

## 👤 Author
**Amirhossein Askari**  
📧 theamiraskarii@gmail.com  
🔗 [GitHub Profile](https://github.com/AmirhoseinAskari)
