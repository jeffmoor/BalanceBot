/*
	BalanceBot.h
	
	Created to hold miscellaneous definitions rather than have numbers in the code.
*/

#define		I2C_CLOCK_FREQ_400KHZ		0xC		// 12
#define		I2C_ADDRESS_MAX				0x7F	// 127

// These are 7-bit addresses, as they should be.  Some vendors shift the addr up one bit and 
// tack on the R/W bit, making an 8-bit addr.  This is why, in the literture, the nunchuck addr
// is often given as 0xA4, which is 0x52 << 1.
#define		I2C_ADDR_NUNCHUCK			0x52
#define		I2C_ADDR_MPU6050_1			0x68
#define		I2C_ADDR_MPU6050_2			0x69

#define		I2C_XMIT_END_SUCCESS		0x0
#define		I2C_XMIT_END_TOO_LONG		0x1
#define		I2C_XMIT_END_ADDR_NACK		0x2
#define		I2C_XMIT_END_DATA_NACK		0x3
#define		I2C_XMIT_END_OTHER_ERROR	0x4




#define		USART_BAUD_9600				9600

















