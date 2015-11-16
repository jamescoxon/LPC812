#include "LPC8xx.h"

#include "onewire.h"

// http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf

/**
 * Assume the bus comprises just one DS18B20 device.
 */

uint64_t ds18b20_rom_read () {
    
    if ( ! ow_reset() ) {
        return -999;
    }
    
    
    // ROM read - only works with one device on the bus
    ow_byte_write (0x33);
    
    return ow_uint64_read();
}

/**
 * Return temperature in 0.001C units. eg 12100 = 12.1C
 */
int32_t ds18b20_temperature_read () {
    if ( ! ow_reset() ) {
        return -9999;
    }
    
    // Skip ROM command
    ow_byte_write (0xCC);
    
    mrtDelay(10);
    
    // Issue Convert command
    ow_byte_write (0x44);
    
    mrtDelay(800);
    
    if ( ! ow_reset() ) {
        return -9999;
    }
    // Skip ROM command
    ow_byte_write (0xCC);
    
    mrtDelay(10);
    
    // Issue command to read scratch pad
    ow_byte_write (0xBE);
    
    int16_t data =0;
    data = ow_byte_read();
    data |= ow_byte_read()<<8;
    
    // Read data (up to 9 bytes, but only interested in first two)
    //uint64_t data = ow_uint64_read();
    
    //MyUARTPrintHex(LPC_USART0, data >> 32 );
    //MyUARTPrintHex(LPC_USART0, (uint32_t)(data & 0x00000000ffffffff ));
    //MyUARTSendStringZ (LPC_USART0, (uint8_t*)"<\r\n");
    
    // 16 bit temperature data is interpreted as a signed 16 bit integer.
    // of 12 bit resolution (by default -- the DS18B20 can be configured
    // for lower resolutions). To get °C multiply by (1/16)°C
    // Return temperature * 10;
    return  (data * 1000) / 16;
    
    
}