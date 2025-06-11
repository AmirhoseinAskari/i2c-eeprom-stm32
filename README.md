# stm32-i2c-eeprom

Driver for using Serial EEPROM Products (AT24C family) in STM32 microcontrollers

## 🔧 Features

- ✅ **Fully compatible with STM32 HAL** — supports **all STM32 MCU series**
- 📦 **Universal AT24Cxx support** — automatic 8-bit / 16-bit addressing and size handling
- 🧠 **Multi-device support** — independently manage multiple EEPROMs on the same I²C bus
- 🛡️ **MISRA-C-style design** — clean, safe, and portable for embedded and safety-critical systems
- 🔌 **Write-protect control** — simple GPIO abstraction for hardware write protection
- ⚡ **Optimized memory access** — low-overhead, high-speed read/write via lightweight HAL wrappers
- 🧭 **Structured memory object** — define EEPROMs using `I2C_MemoryTypeDef` with clean configuration
- 📐 **Modular and portable** — works out of the box with STM32CubeIDE, Keil, IAR, and Makefile-based workflows


## ⚙️ Configuration & Integration

1. **Configure I²C in STM32CubeMX**  
   - Enable an I²C peripheral (e.g., I2C1)  
   - Set the **I²C Speed Mode** to **Standard Mode (100 kHz)** or **Fast Mode (400 kHz)**

2. **Configure GPIO**  
   - Set up a **GPIO output pin** to control the EEPROM's **Write-Protect (WP)** line (optional)

3. **Add the Library to Your Project**  
   - **Include** `i2c_memory.h` in your application source files  
   - **Add** `i2c_memory.c` to your project build  
   - **Define the correct STM32 series** in `stm32_i2c_memory_config.h` (e.g., `_STM32F1`)  
   - **Add the library path** to your compiler’s include directories

4. **Define EEPROM Configuration**  
   - Create one or more `I2C_MemoryTypeDef` instances  
   - Set the I²C handle and hardware address pins (A0/A1/A2)

5. **Build and Flash**  
   - Refer to [`example/main.c`](./example/main.c) for a complete usage demo


## 🧪 API Reference

Each function returns an `I2C_Memory_StatusTypeDef` status code.

### `I2C_Memory_Init(I2C_MemoryTypeDef *pMemory)`  
Initializes an EEPROM instance, calculates the full device address, and prepares WP pin (if used).

### `I2C_Memory_SingleWrite(...)`  
Writes a single byte to the specified EEPROM memory address.

### `I2C_Memory_BurstWrite(...)`  
Writes multiple bytes (burst/page mode) starting from a target memory address.

### `I2C_Memory_SingleRead(...)`  
Reads one byte from a specified memory address.

### `I2C_Memory_BurstRead(...)`  
Reads multiple bytes from the EEPROM starting at a given address.


## 💡 Example

A fully working usage example is included in [`example/main.c`](./example/main.c). It demonstrates how to initialize the memory, perform read/write operations, and work with multiple devices.


## 📜 License

This project is released under the [MIT License](./LICENSE).


## 👤 Author

**Amirhossein Askari**  
📧 theamiraskarii@gmail.com  
🔗 [GitHub Profile](https://github.com/AmirhoseinAskari)
