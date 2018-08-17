/*
	BalanceBot.h
	
	Created to hold miscellaneous definitions rather than have numbers in the code.

	Note: This project assumes the robot code runs on a 16MHz, ATmega328.
*/
#define		FALSE						0
#define		TRUE						1

// Battery monitor values
// R2(22K) and R3(36K) form a 0.38 divider, decreasing the fully charged battery voltage from 12.6V (fully charged 3S, 11.1V pack)
// to ~4.78V.At 4.9mV / unit, this translates to a reading of 975 on analog pin A0.To prevent runing the bot at too low a battery
// voltage, and possibly damaging the pack, we shut down if the pack voltage falls below 10.5V, or roughly 4V out of the divider,
// or a reading of less than 813 on pin A0.
#define		MIN_BATTERY_A0				813
#define		DPIN_LED					13

// Bits
#define		BIT_1						B00000001
#define		BIT_2						B00000010
#define		BIT_3						B00000100
#define		BIT_4						B00001000

#define		CAL_LOOP_DELAY				3700


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

#define		NUNCHUCK_LOW_TRIGGER				88
#define		NUNCHUCK_HIGH_TRIGGER				168

#define		I2C_ADDR_MPU6050_1					0x68
#define		I2C_ADDR_MPU6050_2					0x69

#define		MPU6050_REG_PWR_MGMT_1				0x6B
#define		MPU6050_REG_WHO_AM_I				0x75
#define		MPU6050_REG_GYRO_CONFIG				0x1B
#define		MPU6050_REG_ACCEL_CONFIG			0x1C
#define		MPU6050_REG_CONFIG					0x1A
#define		MPU6050_REG_PWR_MGMT_1_ACTIVATE_GYRO		0x00
#define		MPU6050_REG_GYRO_CONFIG_250DPS_FULL_SCALE	0x00
#define		MPU6050_REG_ACCEL_CONFIG_4G_FULL_SCALE		0x08
#define		MPU6050_REG_CONFIG_LOW_PASS_43HZ			0x03
#define		MPU6050_REG_BALANCE_VALUE			0x3F
#define		MPU6050_REG_RAW_GYRO_VALUES			0x43
#define		MPU6050_ACCEL_4G_LSB				8192

#define		I2C_XMIT_END_SUCCESS				0x0
#define		I2C_XMIT_END_TOO_LONG				0x1
#define		I2C_XMIT_END_ADDR_NACK				0x2
#define		I2C_XMIT_END_DATA_NACK				0x3
#define		I2C_XMIT_END_OTHER_ERROR			0x4

#define		USART_BAUD_9600				9600

#define		AVERAGE_ELEMENT_COUNT		500

#define		ACC_BALANCE_VALUE			1000			// Enter the accelerometer calibration value

#define		PID_P_GAIN					15.0            // Gain setting for the P-controller (15.0)
#define		PID_I_GAIN					1.5				// Gain setting for the I-controller (1.5)
#define		PID_D_GAIN					30.0			// Gain setting for the D-controller (30.0)
#define		TURNING_SPEED				30.0			// Turning speed (30.0)
#define		MAX_TARGET_SPEED			150.0			// Max target speed (100.0)

#define		DEGREES_PER_RADIAN			57.2958
#define		TRIVIAL_ANGLE				0.5

void i2cRegisterWrite1(int iDeviceAddress, int iRegisterAddress, int iRegisterData);
void i2cRegisterReadStart(int iDeviceAddress, int iRegisterAddress, int iNumberOfBytes);










