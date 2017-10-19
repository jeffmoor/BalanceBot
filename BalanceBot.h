/*
	BalanceBot.h
	
	Created to hold miscellaneous definitions rather than have numbers in the code.

	Note: This project assumes the robot code runs on a 16MHz, ATmega328.
*/

#define		I2C_CLOCK_FREQ_400KHZ		0xC		// 12
#define		I2C_ADDRESS_MAX				0x7F	// 127

// These are 7-bit addresses, as they should be.  Some vendors shift the addr up one bit and 
// tack on the R/W bit, making an 8-bit addr.  This is why, in the literture, the nunchuck addr
// is often given as 0xA4, which is 0x52 << 1.
#define		I2C_ADDR_NUNCHUCK					0x52
#define		I2C_ADDR_NUNCHUCK_BLK_REG1_ADDR		0xF0
#define		I2C_ADDR_NUNCHUCK_BLK_REG2_ADDR		0xFB
#define		I2C_ADDR_NUNCHUCK_BLK_REG1_DATA		0x55
#define		I2C_ADDR_NUNCHUCK_BLK_REG2_DATA		0x00
#define		I2C_ADDR_NUNCHUCK_WHT_REG1_ADDR		0x40
#define		I2C_ADDR_NUNCHUCK_WHT_REG1_DATA		0x00

#define		NUNCHUCK_REG_XY_VALUES				0x00

#define		I2C_ADDR_MPU6050_1					0x68
#define		I2C_ADDR_MPU6050_2					0x69

#define		MPU6050_REG_PWR_MGMT_1				0x6B
#define		MPU6050_REG_ID						0x75
#define		MPU6050_REG_GYRO_CONFIG				0x1B
#define		MPU6050_REG_ACCEL_CONFIG			0x1C
#define		MPU6050_REG_CONFIG					0x1A
#define		MPU6050_REG_PWR_MGMT_1_ACTIVATE_GYRO		0x00
#define		MPU6050_REG_GYRO_CONFIG_250DPS_FULL_SCALE	0x00
#define		MPU6050_REG_ACCEL_CONFIG_4G_FULL_SCALE		0x08
#define		MPU6050_REG_CONFIG_LOW_PASS_43HZ			0x03
#define		MPU6050_REG_BALANCE_VALUE			0x3F
#define		MPU6050_REG_RAW_GYRO_VALUES			0x43


#define		I2C_XMIT_END_SUCCESS		0x0
#define		I2C_XMIT_END_TOO_LONG		0x1
#define		I2C_XMIT_END_ADDR_NACK		0x2
#define		I2C_XMIT_END_DATA_NACK		0x3
#define		I2C_XMIT_END_OTHER_ERROR	0x4

#define		USART_BAUD_9600				9600


#define		ACC_CALIBRATION_VALUE		1000			// Enter the accelerometer calibration value

#define		PID_P_GAIN					15.0            // Gain setting for the P-controller (15.0)
#define		PID_I_GAIN					1.5				// Gain setting for the I-controller (1.5)
#define		PID_D_GAIN					30.0			// Gain setting for the D-controller (30.0)
#define		TURNING_SPEED				30.0			// Turning speed (30.0)
#define		MAX_TARGET_SPEED			150.0			// Max target speed (100.0)



void i2cRegisterWrite1(int iDeviceAddress, int iRegisterAddress, int iRegisterData);
void i2cRegisterReadStart(int iDeviceAddress, int iRegisterAddress, int iNumberOfBytes);










