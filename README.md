# stm32-i2c-eeprom

Driver for using Serial EEPROM Products (AT24C family) in STM32 microcontrollers

## 🔧 Features

- Compatible with STM32 HAL I²C driver 
- Supports all AT24C EEPROM memory series 
- **Developed with consideration of MISRA-C guidelines** for safety-critical and embedded systems  

## ✅ Requirements

- STM32Cube HAL drivers  
- Properly configured I²C peripheral (via STM32CubeMX or manually)  
- Compatible EEPROM from the AT24C series

## 🧪 API Reference

### `void I2C_Memory_Init(I2C_MemoryTypeDef *pMemory)`
### `I2C_Memory_StatusTypeDef I2C_Memory_SingleWrite(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t data, uint32_t timeout)`
### `I2C_Memory_StatusTypeDef I2C_Memory_BurstWrite(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t size, uint32_t timeout)`
### `I2C_Memory_StatusTypeDef I2C_Memory_SingleRead(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t timeout)`
### `I2C_Memory_StatusTypeDef I2C_Memory_BurstRead(I2C_MemoryTypeDef *pMemory, uint32_t address, uint8_t *pData, uint32_t size, uint32_t timeout)`

## 💡 Example
An example showing how to use the library is provided in [`example/main.c`](./example/main.c). 

## 📜 License
This project is licensed under the [MIT License](./LICENSE).

## 👤 Author
**Amirhossein Askari**  
📧 theamiraskarii@gmail.com  
🔗 [GitHub Profile](https://github.com/AmirhoseinAskari)
