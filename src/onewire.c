/**
 * A simple one wire big-bang library.
 * Lifted from: https://github.com/jdesbonnet/LPC810_SousVide/blob/master/src/onewire.c
 *
 */

#include "LPC8xx.h"

#include "onewire.h"
#include "gpio.h"

#include <stdint.h>

/*
 This code is from Colin O'Flynn - Copyright (c) 2002
 only minor changes by M.Thomas 9/2004
 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#define CRC8INIT    0x00
#define CRC8POLY    0x18              //0X18 = X^8+X^5+X^4+X^0

uint8_t crc8( uint8_t *data, uint16_t number_of_bytes_in_data )
{
    uint8_t  crc;
    uint16_t loop_count;
    uint8_t  bit_counter;
    uint8_t  b;
    uint8_t  feedback_bit;
    
    crc = CRC8INIT;
    
    for (loop_count = 0; loop_count != number_of_bytes_in_data; loop_count++)
    {
        b = data[loop_count];
        
        bit_counter = 8;
        do {
            feedback_bit = (crc ^ b) & 0x01;
            
            if ( feedback_bit == 0x01 ) {
                crc = crc ^ CRC8POLY;
            }
            crc = (crc >> 1) & 0x7F;
            if ( feedback_bit == 0x01 ) {
                crc = crc | 0x80;
            }
            
            b = b >> 1;
            bit_counter--;
            
        } while (bit_counter > 0);
    }
    
    return crc;
}
//Completetion of code

uint32_t ow_port;
uint32_t ow_pin;

void ow_init(int port, int pin) {
    ow_port=port;
    ow_pin=pin;
}

void ow_low() {
    // set direction output
    gpioSetDir(ow_port, ow_pin, 1);
    // set low
    gpioSetValue(ow_port, ow_pin, 0);
}

void ow_high() {
    // set direction input (high Z) and let pull-up R bring high
    gpioSetDir(ow_port, ow_pin, 0);
}

int ow_read() {
    gpioSetDir(ow_port, ow_pin, 0);
    return gpioGetPinValue(ow_port, ow_pin);
}

/**
 * Issue a read slot and return the result. The result must be read within
 * 15µs of the
 *
 * @return 0 or 1
 */
int ow_bit_read () {
    //GPIOSetBitValue(0,2, 0);//debug
    
    // The read slow starts with the bus is diven low.
    // We have 15µs from the falling edge read the bus.
    ow_low();
    UmrtDelay(1); // Must be held low for at least 1µs
    
    // Bring bus high again. And read within the 15µs time interval
    // (already a few µs used by by now...)
    ow_high();
    UmrtDelay(1);
    
    //GPIOSetBitValue(0,2, 1); //debug
    int b = ow_read();
    //GPIOSetBitValue(0,2, 0); //debug
    
    // Read slots must be a minimum of 60µs in duration with a minimum of 1µs
    // recovery time between slots. Rather than monitor bus to check for end
    // of slot, just delay for a period well exceeding the 60µs slot time.
    UmrtDelay(65);
    
    //GPIOSetBitValue(0,2, 1); //debug
    
    return b;
}

int ow_reset() {
    ow_low();
    UmrtDelay(480);
    ow_high();
    UmrtDelay(70);
    
    int detect = ow_read();
    ow_high();
    
    UmrtDelay(410);
    
    return ~detect;
}

void ow_bit_write (int b) {
    
    // Write slot duration min 60µs
    ow_low();
    if (b) {
        // having trouble getting this in the 1-15µs range. Need better delay mechanism.
        UmrtDelay(1); // max 15µs, min 1µs (?)
        ow_high();
        UmrtDelay(60);
    } else {
        UmrtDelay(66);
        ow_high();
    }
    
    // Recovery time
    UmrtDelay(5);
}
void ow_byte_write (int data) {
    int i;
    
    // Send LSB first.
    
    for (i = 0; i < 8; i++) {
        ow_bit_write(data & 0x01);
        data >>= 1;
    }
    
}

int ow_byte_read () {
    int i, data = 0;
    for (i = 0; i < 8; i++) {
        data >>= 1;
        data |= ow_bit_read() ? 0x80 : 0x00;
    }
    return data;
}


uint64_t ow_uint64_read () {
    uint64_t data = 0;
    
    int i;
    for (i = 0; i < 8; i++) {
        data <<= 8;
        data |= ow_byte_read();
    }
    
    return data;
}
