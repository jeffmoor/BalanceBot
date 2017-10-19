/*
	BalanceBot.cpp

	Created to hold miscellaneous functions rather than have repeated code.

	Note: This project assumes the robot code runs on a 16MHz, ATmega328.
*/

#include <Wire.h>
#include "BalanceBot.h"


void i2cRegisterWrite1(int iDeviceAddress, int iRegisterAddress, int iRegisterData)
{
	Wire.beginTransmission(iDeviceAddress);			// Start communication with the specified device
	Wire.write(iRegisterAddress);                   // Write the address of the register o set
	Wire.write(iRegisterData);                      // Write the register data
	Wire.endTransmission();                         // End the transmission
}


void i2cRegisterReadStart(int iDeviceAddress, int iRegisterAddress, int iNumberOfBytes)
{
	Wire.beginTransmission(iDeviceAddress);
	Wire.write(iRegisterAddress);
	Wire.endTransmission();
	Wire.requestFrom(iDeviceAddress, iNumberOfBytes);	// Perform the "restart," repeating the device ID and the number of bytes requested
}









