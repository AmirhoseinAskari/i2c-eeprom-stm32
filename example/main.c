#include "i2c_memory.h"


I2C_MemoryTypeDef memory1, memory2;
I2C_Memory_StatusTypeDef status[8];

uint8_t data_list[5U];
uint8_t data;


int main(void)
{
    /* Initialize all configured peripherals (HAL, GPIO, I2C, etc.) */
    // ...
    // ...
	   // ...
	
   
	   // Setup eeprom memory device 1 parameters
    memory1.I2Cx = hi2c1;
    memory1.WP_GPIO = GPIOB;
    memory1.WP_Pin = GPIO_PIN_3;
    memory1.MemoryCode = I2C_MEM_CODE_AT24C04;
    memory1.A1_PinState = 0;
    memory1.A2_PinState = 0;
    
    I2C_Memory_Init(&memory1);                                          // Initialize eeprom memory device 1
    status[0] = I2C_Memory_SingleWrite(&memory1, 8, 'A', 10);           // Write single byte 'A' at address 8 with timeout 10ms 
    status[1] = I2C_Memory_BurstWrite(&memory1, 100, "Hello", 5, 50);   // Write "Hello" starting at address 100 with timeout 50ms
    status[2] = I2C_Memory_SingleRead(&memory1, 8, &data, 10);          // Read single byte from address 8 with timeout 10ms
    status[3] = I2C_Memory_BurstRead(&memory1, 100, data_list, 5, 50);  // Read 5 bytes starting from address 100 with timeout 50ms
	
	
    // Setup eeprom memory device 2 parameters 
    memory2.I2Cx = hi2c1;
    memory2.WP_GPIO = GPIOB;
    memory2.WP_Pin = GPIO_PIN_4;
    memory2.MemoryCode = I2C_MEM_CODE_AT24C64;
    memory2.A0_PinState = 1;
    memory2.A1_PinState = 1;
    memory2.A2_PinState = 1;
    
    I2C_Memory_Init(&memory2);                                         // Initialize eeprom memory device 2
    status[4] = I2C_Memory_SingleWrite(&memory2, 37, 'B', 10);         // Write single byte 'B' at address 37 with timeout 10ms
    status[5] = I2C_Memory_BurstWrite(&memory2, 88, "World", 5, 50);   // Write "World" starting at address 88 with timeout 50ms
    status[6] = I2C_Memory_SingleRead(&memory2, 37, &data, 10);        // Read single byte from address 37 with timeout 10ms
    status[7] = I2C_Memory_BurstRead(&memory2, 88, data_list, 5, 50);  // Read 5 bytes starting from address 88 with timeout 50ms
        
 
    while (1)
    {
		       // Your application code here   
    }
}
