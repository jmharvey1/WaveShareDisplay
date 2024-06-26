
# ESP32-S3-Touch-LCD-7 - Waveshare

ESP32-S3 7inch Capacitive Touch Display Development Board, ESP32 With Display, 800×480, 5-point Touch, 32-bit LX7 Dual-core Processor

___

This is VS Code/PlatformIO project, Using framework-espidf @ 3.50201.240515 (5.2.1). For the Waveshare ESP32S3-Touch-LCD-7 development board.
In its current state, it runs on my setup. But I have modified at least one of the ESPIDF component files. Namely, the .platformio/packages/framework-espidf/components/driver/i2c/i2c_master.c file.
The main focus of this effort is to create a set of source code files that can be used with the IDE configuration described above. 
The code contained in this repository comes primarily from the code found at the Waveshare Wiki site, for this board. And is assumed to be ‘open source’ code.
The main reason, for the creation of this repository is the code found, at the wiki site, was written around what espressif now calls ‘legacy I2C drivers’. 
Using the 5.2 framework, to compile the wiki source code, creates a binary that will flash, but wont run. Because the resulting .bin file contains references to both the 'legacy', & current I2C drivers.     



    
---
Note for those interested:
The waveshare display uses the GT911 chip, for touch interface, and the ST7262 to drive the display.
  
___



  
___
 
___
 
___
 
