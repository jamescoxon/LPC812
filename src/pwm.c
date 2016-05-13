//pwm.c
// adapted from https://github.com/basilfx/LPC810-FanController/blob/13396add42e1e850be6507b6471ecfede6403d4a/firmware/src/main.c

#include "LPC8xx.h"
#include "settings.h"

#define SCT_CONFIG_32BIT_COUNTER        0x00000001	/*!< Operate as 1 32-bit counter */
#define SCT_CONFIG_CLKMODE_BUSCLK       (0x0 << 1)	/*!< Bus clock */

volatile uint32_t cycleTicks;

/**
 * Initialize the PWM mode of the SCT.
 */
void init_pwm()
{
    //Chip_SCT_Init(LPC_SCT);
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);       // enable the SCT clock
    
    /* Configure the SCT as a 32bit counter using the bus clock */
    LPC_SCT->CONFIG = SCT_CONFIG_32BIT_COUNTER | SCT_CONFIG_CLKMODE_BUSCLK;
    
    /* Initial CTOUT0 state is high */
    LPC_SCT->OUTPUT = (7 << 0);
    
    /* The PWM will use a cycle time of (PWM_FREQUENCY)Hz based off the bus clock */
    cycleTicks = __SYSTEM_CLOCK / PWM_FREQUENCY;
    
    /* Setup for match mode */
    LPC_SCT->REGMODE_L = 0;
    
    /* Setup match counter 0 for the number of ticks in a PWM sweep, event 0
     will be used with the match 0 count to reset the counter.  */
    LPC_SCT->MATCH[0].U = cycleTicks;
    LPC_SCT->MATCHREL[0].U = cycleTicks;
    LPC_SCT->EVENT[0].CTRL = 0x00001000;
    LPC_SCT->EVENT[0].STATE = 0xFFFFFFFF;
    LPC_SCT->LIMIT_L = (1 << 0);
    
    /* For CTOUT0 to CTOUT2, event 1 is used to clear the output */
    LPC_SCT->OUT[0].CLR = (1 << 0);
    LPC_SCT->OUT[1].CLR = (1 << 0);
    LPC_SCT->OUT[2].CLR = (1 << 0);
    
    /* Setup event 1 to trigger on match 1 and set CTOUT0 high */
    LPC_SCT->EVENT[1].CTRL = (1 << 0) | (1 << 12);
    LPC_SCT->EVENT[1].STATE = 1;
    LPC_SCT->OUT[0].SET = (1 << 1);
    
    /* Don't use states */
    LPC_SCT->STATE_L = 0;
    
    /* Unhalt the counter to start */
    LPC_SCT->CTRL_U &= ~(1 << 2);
}

/**
 * Set the PWM value. Value is a percentage between 0-100.
 */
void set_pwm(uint8_t percentage)
{
    uint32_t value, newTicks;;
    
    if (percentage >= 100) {
        value = 100;
    }
    else if (percentage == 0) {
        value = cycleTicks + 1;
    }
    else {
        newTicks = (cycleTicks * percentage) / 100;
        
        /* Approximate duty cycle rate */
        value = cycleTicks - newTicks;
    }
    
    LPC_SCT->MATCHREL[1].U = value;
}
