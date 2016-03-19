/**************************************************************************/
/* i2c.h	by NECV20													  */
/* i2c functions using registers										  */
/**************************************************************************/

#include "LPC8xx.h"

#define I2C_CFG_MSTEN (0x1)
#define I2C_CFG_SLVEN (0x2)
#define I2C_STAT_MSTPENDING (0x1)
#define I2C_STAT_MSTSTATE (0xe)
#define I2C_STAT_MSTST_IDLE (0x0)
#define I2C_STAT_MSTST_RX (0x2)
#define I2C_STAT_MSTST_TX (0x4)
#define I2C_STAT_MSTST_NACK_ADDR (0x6)
#define I2C_STAT_MSTST_NACK_TX (0x8)
#define I2C_STAT_SLVPENDING (0x100)
#define I2C_STAT_SLVSTATE (0x600)
#define I2C_STAT_SLVST_ADDR (0x000)
#define I2C_STAT_SLVST_RX (0x200)
#define I2C_STAT_SLVST_TX (0x400)
#define I2C_STAT_SLVSEL	(0x4000)
#define I2C_MSTCTL_MSTCONTINUE (0x1)
#define I2C_MSTCTL_MSTSTART (0x2)
#define I2C_MSTCTL_MSTSTOP (0x4)
#define I2C_SLVCTL_SLVCONTINUE (0x1)
#define I2C_SLVCTL_SLVNACK (0x2)

void I2c_IRQHandler();

void I2cInit (int sda, int scl);
void I2cAsSlave (uint8_t adress);
uint32_t I2cSlaveAdressed ();
uint32_t I2cMasterWantsToSend ();
uint32_t I2cMasterWantsToReceive ();
void I2cSlaveSendACK ();
void I2cSlaveSendNACK ();



int I2cSlaveReceiving (char *buffer, int max);
int I2cSlaveSending (char *buffer, int max, int start);
