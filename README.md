# I2C-eeprom-stm32

Driver for using Serial EEPROM Products (AT24C family) in STM32 microcontrollers

## 🔧 Features
- 🔗 **Fully compatible with STM32 HAL** — supports **all STM32 MCU series**
- 💾 **Universal AT24Cxx support** — automatic 8-bit / 16-bit addressing and memory size handling
- 🔀 **Multi-device support** — manage multiple EEPROMs independently on the same I²C bus
- 🛡️ **MISRA-C-style design** — clean, safe, and portable for embedded and safety-critical applications
- 🔒 **Write-protect support** — optional GPIO abstraction for controlling the WP (Write Protect) pin
- ⚡ **Optimized memory access** — low-overhead and high-speed read/write using efficient HAL wrappers
- 🗂️ **Structured configuration** — uses `I2C_MemoryTypeDef` objects for clean, scalable integration
- 🔄 **Modular and portable** — works with STM32CubeIDE, Keil, IAR, or Makefile-based environments

## ⚙️ How to use this library

1. **Configure I²C in STM32CubeMX**
   - Enable an I²C peripheral (e.g., I2C1)
   - Set **I²C Speed Mode** to **Standard Mode (100 kHz)** or **Fast Mode (400 kHz)**

2. **Configure GPIO (Optional)**
   - Set up a **GPIO Output** to control the EEPROM’s **Write-Protect (WP)** pin

3. **Add the Library to Your Project**
   - **Include** `i2c_memory.h` in your application code
   - **Add** `i2c_memory.c` to your build system
   - **Set the STM32 MCU series macro** in `stm32_i2c_memory_config.h`
   - **Add the library folder** to your compiler’s include paths

4. **Define EEPROM Configuration**
   - Create one or more `I2C_MemoryTypeDef` instances
   - Set the I²C handle and the device address (using A0/A1/A2 pin configuration)

5. **Build and Flash**  
   - Use the example in [`example/main.c`](./example/main.c) to guide your implementation

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
A complete working example is available in [`example/main.c`](./example/main.c).
It demonstrates initialization, reading, writing, and working with multiple devices.

## 📜 License
This project is released under the [MIT License](./LICENSE).

## 👤 Author
**Amirhossein Askari**  
📧 theamiraskarii@gmail.com  
🔗 [GitHub Profile](https://github.com/AmirhoseinAskari)
