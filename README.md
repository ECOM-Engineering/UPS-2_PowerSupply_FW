# UPS-2 Power Supply Firmware

This project is the firmware part of the superior **UPS-2** project  
https://github.com/ECOM-Engineering/UPS-2_Uninteruptible-Power-Supply.git .

If you are looking for a simple power-down switch, please refer to the  **s_shut** project  
https://github.com/ECOM-Engineering/Raspberry-Shutdown.git .

## Firmware Description

UPS-2 board is controlled by a ST32G030 CPU. It is programmed in ANSI C language using the STM32CubeIDE.  

Detailed firmware description can be generated using [Doxygen](https://www.doxygen.nl/index.html). Configuration file doxyfile can be found here: https://github.com/ECOM-Engineering/UPS-2_PowerSupply_FW/tree/develop/Core

**Important Note:** Switching between main and backup power is NOT performed by the CPU, but independently by the dedicated multiplexer chip. This concept leads to very fast and reliable power control.

Please refer to the hardware repository https://github.com/ECOM-Engineering/UPS-2_PowerSupply_HW

### Firmware Functionality

Onboard CPU tasks:

- Serial (UART) or parallel communication with the Raspberry Pi
- User interface
- Status control
- Voltage and temperature measurements
- Firmware update control

### License

© Copyright (c) 2020-2021 Klaus Mezger, ECOM ENGINEERING   
Open Source licensed under [BSD-2-Clause](https://choosealicense.com/licenses/bsd-2-clause/)

