/**************************************************************************/
/*!
 @file     main.c
 
 @section LICENSE
 
 Software License Agreement (BSD License)
 
 Copyright (c) 2013, K. Townsend (microBuilder.eu)
 All rights reserved.
 
 Modified James Coxon (jacoxon@googlemail.com) 2014-2015
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 3. Neither the name of the copyright holders nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**************************************************************************/
//Node settings have been moved seperate file "settings.h"
#include "settings.h"

#include <stdio.h>
#include "LPC8xx.h"
#include "mrt.h"
#include "spi.h"
#include "rfm69.h"

#if defined(GATEWAY) || defined(DEBUG) || defined(GPS)
    #include "uart.h"
#endif

#ifdef GPS
    #include "gps.h"
#endif

#ifdef ZOMBIE
    #include "zombie.h"
#endif


char data_temp[66];

uint8_t data_count = 96; // 'a' - 1 (as the first function will at 1 to make it 'a'
unsigned int rx_packets = 0, random_output = 0, rx_restarts = 0;
int16_t rx_rssi, floor_rssi, rssi_threshold, adc_result = 0;
/**
 * Setup all pins in the switch matrix of the LPC812
 */
void configurePins() {
    /* Enable SWM clock */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);
    
    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0004UL;
    /* SPI0_SCK */
    LPC_SWM->PINASSIGN3 = 0x01ffffffUL;
    /* SPI0_MOSI */
    /* SPI0_MISO */
    /* SPI0_SSEL */
    LPC_SWM->PINASSIGN4 = 0xff0f0809UL;
    
    /* Pin Assign 1 bit Configuration */
    /* SWCLK */
    /* SWDIO */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffb3UL;

    
}

/**
 * Packet data transmission
 * @param Packet length
 */
void transmitData(uint8_t i) {

#ifdef GATEWAY
        printf("rx: %s|0\r\n", data_temp);
#endif

    // Transmit the data (need to include the length of the packet and power in dbmW)
    RFM69_send(data_temp, i, POWER_OUTPUT);
    
#ifdef ZOMBIE_MODE
    //Ensure we are in Sleep mode to save power
    RFM69_setMode(RFM69_MODE_SLEEP);
#else
    //Ensure we are in RX mode
    RFM69_setMode(RFM69_MODE_RX);
    
    mrtDelay(500);
#endif

}

/**
 * This function is called when a packet is received by the radio. It will
 * process the packet.
 */
inline void processData(uint32_t len) {
    uint8_t i, packet_len;
    
    for(i=0; i<len; i++) {
        //finds the end of the packet
        if(data_temp[i] != ']')
            continue;
        
        //then terminates the string, ignore everything afterwards
        data_temp[i+1] = '\0';
        
        //Check validity of string
        // 1) is the first position in array a number
        //printf("%d\r\n", data_temp[0]);
        if((int)data_temp[0] <= 48 || (int)data_temp[0] > 57) {
            //printf("Error1\r\n");
            break;
        }
        
        // 2) is the second position in array a letter
        //      < 'a' or > 'z' then break
        //printf("%d\r\n", data_temp[1]);
        if((int)data_temp[1] < 97 || (int)data_temp[1] > 122){
            //printf("Error2\r\n");
            break;
        }
        
#ifdef GATEWAY
        printf("rx: %s|%d\r\n",data_temp, RFM69_lastRssi());
#endif
        //Reduce the repeat value
        data_temp[0] = data_temp[0] - 1;
        //Now add , and end line and let string functions do the rest
        data_temp[i] = ',';
        data_temp[i+1] = '\0';
        
        if(strstr(data_temp, NODE_ID) != 0)
            break;
        
        strcat(data_temp, NODE_ID); // Add ID
        strcat(data_temp, "]"); // Add ending
        
        packet_len = strlen(data_temp);
        mrtDelay(random_output); // Random delay to try and avoid packet collision
        
        rx_packets++;
        
        transmitData(packet_len);
        break;
    }
}

/**
 * It processing incoming data by the radio or serial connection. This function
 * has to be continously called to keep the node running. This function also
 * adds a delay which is specified as 100ms per unit. Therefore 1000 = 100 sec
 * @param countdown delay added to this function
 */
void awaitData(int countdown) {

    uint8_t rx_len, flags1, old_flags1 = 0x90;

    //Clear buffer
    data_temp[0] = '\0';

    RFM69_setMode(RFM69_MODE_RX);

    rx_restarts = 0;

    while(countdown > 0) {

        flags1 = spiRead(RFM69_REG_27_IRQ_FLAGS1);
#ifdef DEBUG
        if (flags1 != old_flags1) {
            printf("f1: %02x\r\n", flags1);
            old_flags1 = flags1;
        }
#endif
        if (flags1 & RF_IRQFLAGS1_TIMEOUT) {
            // restart the Rx process
            spiWrite(RFM69_REG_3D_PACKET_CONFIG2, spiRead(RFM69_REG_3D_PACKET_CONFIG2) | RF_PACKET2_RXRESTART);
            rx_restarts++;
            // reset the RSSI threshold
            floor_rssi = RFM69_sampleRssi();
#ifdef DEBUG
            // and print threshold
            printf("Restart Rx %d\r\n", RFM69_lastRssiThreshold());
#endif
        }
        // Check rx buffer
        if(RFM69_checkRx() == 1) {
            RFM69_recv(data_temp,  &rx_len);
            data_temp[rx_len - 1] = '\0';
            processData(rx_len);
        }

        countdown--;
        mrtDelay(100);
    }
}

/**
 * Increments packet count which is transmitted in the beginning of each
 * packet. This function has to be called every packet which is initially
 * transmitted by this node.
 * Packet count is starting with 'a' and continues up to 'z', thereafter
 * it's continuing with 'b'. 'a' is only transmitted at the very first
 * transmission!
 */
void incrementPacketCount(void) {
    data_count++;
    // 98 = 'b' up to 122 = 'z'
    if(data_count > 122) {
        data_count = 98; //'b'
    }
}



int main(void)
{
#ifdef ZOMBIE_MODE
    LPC_SYSCON->BODCTRL = 0x11;  //Should be set to Level 1 (Assertion 2.3V, De-assertion 2.4V) reset
#endif

#if defined(GATEWAY) || defined(DEBUG) || defined(GPS)
    // Initialise the UART0 block for printf output
    uart0Init(9600);
#endif
    
    // Configure the multi-rate timer for 1ms ticks
    mrtInit(__SYSTEM_CLOCK/1000);

    /* Enable AHB clock to the Switch Matrix , UART0 , GPIO , IOCON, MRT , ACMP */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7) | (1 << 14) /*| (1 << 6)*//* | (1 << 18)*/
    | (1 << 10) | (1 << 19);
    
    // Configure the switch matrix (setup pins for UART0 and SPI)
    configurePins();
    
#ifdef DEBUG
    mrtDelay(100);
    printf("Node Booted\r\n");
    mrtDelay(100);
#endif

    RFM69_init();
    
#ifdef ZOMBIE_MODE
    //This is to allow the setup to recover from the initial boot and
    // avoid a loop
    RFM69_setMode(RFM69_MODE_SLEEP);
    init_sleep();
    sleepMicro(20000);
#endif
    
#ifdef DEBUG
    printf("Node initialized, version %s\r\n",GIT_VER);
#endif
    
    //Seed random number generator, we can use our 'unique' ID
    random_output = NODE_ID[0] + NODE_ID[1] + NODE_ID[2];

#ifdef GPS
    setupGPS();
#endif
    
    while(1) {

        incrementPacketCount();
        
        //Clear buffer
        data_temp[0] = '\0';
        uint8_t n;
        
        //Create the packet
        int int_temp;
        
#ifdef ZOMBIE_MODE
        //Sleep Mode
        RFM69_setMode(RFM69_MODE_SLEEP);
        init_sleep();
        sleepMicro(10000);
#endif
        
        int_temp = RFM69_readTemp(); // Read transmitter temperature
        
        rx_rssi = RFM69_lastRssi();
        // read the rssi threshold before re-sampling noise floor which will change it
        rssi_threshold = RFM69_lastRssiThreshold();
        floor_rssi = RFM69_sampleRssi();
        
#ifdef ZOMBIE_MODE
        //Sleep Mode
        RFM69_setMode(RFM69_MODE_SLEEP);
        init_sleep();
        sleepMicro(10000);
#endif
    
#ifdef ZOMBIE_MODE
        adc_result = acmpVccEstimate();
        //sleepMicro(20000);
#endif
        
#ifdef DEBUG
        printf("ADC: %d\r\n", adc_result);
        
#endif
#ifdef ZOMBIE_MODE
        //Sleep Mode
        RFM69_setMode(RFM69_MODE_SLEEP);
        init_sleep();
        sleepMicro(10000);
#endif

        if(data_count == 97) {
            n = sprintf(data_temp, "%d%cL%s[%s]", NUM_REPEATS, data_count, LOCATION_STRING, NODE_ID);
        }
        else {
            
#ifdef DEBUG
            //n = sprintf(data_temp, "%d%cT%dR%d,%dC%dX%d,%dV%d[%s]", NUM_REPEATS, data_count, int_temp, rx_rssi, floor_rssi, rx_packets, rx_restarts, rssi_threshold, adc_result, NODE_ID);
            n = sprintf(data_temp, "%d%cT%dV%d[%s]", NUM_REPEATS, data_count, int_temp, adc_result, NODE_ID);
#elif defined(ZOMBIE_MODE)
            n = sprintf(data_temp, "%d%cT%dV%d[%s]", NUM_REPEATS, data_count, int_temp, adc_result, NODE_ID);
#elif defined(GPS)
            gps_get_position();
            mrtDelay(500);
            n = sprintf(data_temp, "%d%cL%d,%d,%dT%dR%d[%s]", NUM_REPEATS, data_count, lat, lon, alt, int_temp, rx_rssi, NODE_ID);
#else
            n = sprintf(data_temp, "%d%cT%dR%d[%s]", NUM_REPEATS, data_count, int_temp, rx_rssi, NODE_ID);
#endif
        }
        
        transmitData(n);

#ifdef ZOMBIE_MODE
        sleepRadio();
#else
        awaitData(TX_GAP);
#endif
    
         }
    
}
 
