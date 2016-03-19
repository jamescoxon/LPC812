/**************************************************************************/
/* i2c.c	by NECV20													  */
/* i2c functions using registers										  */
/**************************************************************************/

#include "i2c.h"

void I2c_IRQHandler() {
}

void I2cInit (int sda, int scl) {
	/* Enable AHB clock to the I2C domain. */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<5);
	LPC_SYSCON->PRESETCTRL    &= ~(1 << 6);
	LPC_SYSCON->PRESETCTRL    |=  (1 << 6);

	/* Set an internal clock - this is not driving SCL! */
	LPC_I2C->DIV = 0x00000001UL;
}

void I2cAsSlave (uint8_t address) {
	// mark that bit0 of slave adress is cleared
	LPC_I2C->SLVADR0 = address; // put address in address 0 register
	LPC_I2C->CFG = I2C_CFG_SLVEN;
}

uint32_t I2cSlaveAdressed () {
	if (LPC_I2C->STAT & I2C_STAT_SLVSEL) return 1;
	if (!(LPC_I2C->STAT & I2C_STAT_SLVPENDING)) return 0;
	if ((LPC_I2C->STAT & I2C_STAT_SLVSTATE) == I2C_STAT_SLVST_ADDR) {
		LPC_I2C->SLVCTL = I2C_SLVCTL_SLVCONTINUE;
		return 1;
	}
	return 0;
}

uint32_t I2cMasterWantsToSend () {
	if (!(LPC_I2C->STAT & I2C_STAT_SLVPENDING)) return 0;
	return ((LPC_I2C->STAT & I2C_STAT_SLVSTATE) == I2C_STAT_SLVST_RX);
}

uint32_t I2cMasterWantsToReceive () {
	if (!(LPC_I2C->STAT & I2C_STAT_SLVPENDING)) return 0;
	return ((LPC_I2C->STAT & I2C_STAT_SLVSTATE) == I2C_STAT_SLVST_TX);
}

void I2cSlaveSendACK () {
	LPC_I2C->SLVCTL = I2C_SLVCTL_SLVCONTINUE;
}

void I2cSlaveSendNACK () {
	LPC_I2C->SLVCTL = I2C_SLVCTL_SLVNACK;
}

int I2cSlaveReceiving (char *buffer, int max) {
int index = 0;

	while (LPC_I2C->STAT & I2C_STAT_SLVSEL) {
		if (I2cMasterWantsToSend ()) {
			if (index < max) buffer [index++] = LPC_I2C->SLVDAT;
			LPC_I2C->SLVCTL = I2C_SLVCTL_SLVCONTINUE; // ack
		}
	}
	return index;
}

int I2cSlaveSending (char *buffer, int max, int start) {
	int index = 0;
	int i;

	while (LPC_I2C->STAT & I2C_STAT_SLVSEL) {
		if (I2cMasterWantsToReceive ()) {
			i = start + index++;
			if (i < max) LPC_I2C->SLVDAT = buffer [i];
			else LPC_I2C->SLVDAT = '\0';
			LPC_I2C->SLVCTL = I2C_SLVCTL_SLVCONTINUE; // ack
		}
	}
	return index;
}
