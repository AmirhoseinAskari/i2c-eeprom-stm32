# stm32-i2c-eeprom

Driver for using Serial EEPROM Products (AT24C family) in STM32 microcontrollers

## 🔧 Features

- Compatible with STM32 HAL I²C driver 
- Supports all AT24C EEPROM memory series 
- Iterative Newton–Raphson method for temperature calculation  
- Temperature range: **-200°C to +850°C**, compliant with IEC 60751 standard  
- Lightweight, portable C code  
- **Developed with consideration of MISRA-C guidelines** for safety-critical and embedded systems  

## ✅ Requirements

- STM32Cube HAL drivers  
- Properly configured I²C peripheral (via STM32CubeMX or manually)  
- Compatible EEPROM from the AT24C series

## 💡 Example
An example showing how to use the library is provided in [`example/main.c`](./example/main.c). 

## 📜 License
This project is licensed under the [MIT License](./LICENSE).

## 👤 Author
**Amirhossein Askari**  
📧 theamiraskarii@gmail.com  
🔗 [GitHub Profile](https://github.com/AmirhoseinAskari)
