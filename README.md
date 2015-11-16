Adapted from LPC810_CodeBase

* Updated SPI driver
* New driver for RFM69 Radio module
* UART code using interrupt and buffer
* GPS code to read Ublox GPS modules
* Stripped down version of string library
* Included memcpy
* One Wire/DS18b20 library borrowed from jdesbonnet

Please remember to edit src/setting.h and add your own node ID and location string as well as defining the various functions you want to use.

* DEBUG - passes debug data through UART0
* GATEWAY - passes rx'd data through UART0
* GPS - allows access to a ublox based GPS on UART1
* ADC - uses ACMP2 and internal resistor ladder to measure input voltage
* ZOMBIE_MODE - low power mode where instead of rx'ing between packets sleeps to save power
* RFM_TEMP - measure RFM69's internal temperature
* ACMPVCC - use ACMP to read input voltage
  * VCC_THRES - define mV where switched to rx
* ONE_WIRE - use the one wire library to read a DS18b20 temperature sensor 


LPC810_CodeBase
===============

Open source code base for the ARM Cortex M0+ LPC810 family from NXP.

This code base is intended to work with the LPC810 in a DIP8 package.  While these drivers are based on the original LPC800 example code from NXP, the LPC810 has limited resources (4KB flash and 1KB SRAM), so smaller, lighter-weight drivers had to be written to get the most out of these resources we have.

The current code implements the following peripheral drivers:

- A basic SPI driver
- Some simple GPIO helper functions (although GPIO should normally be accessed directly via the appropriate registers)
- A simple driver for UART0 and printf-redirection that allows 'printf' output to be transmitted to UART0
- A basic multi-rate timer driver that allows us to set delays

The code base also implements a mini printf that takes up much less space than the default printf used in most libc variants.  If necessary, it's easy to change the printf redirection to a location other than UART0 via the printf-redirection.c file.
