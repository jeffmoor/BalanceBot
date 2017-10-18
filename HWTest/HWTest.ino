///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "BalanceBot.h"


byte		error, MPU_6050_found, nunchuck_found, lowByte, highByte;
int			iAddrI2C;
int			nDevices;

void setup()
{
	Wire.begin();
	TWBR = I2C_CLOCK_FREQ_400KHZ;					// Set the I2C clock speed to 400kHz (Note: nunchuck usually 100KHz)
	Serial.begin(USART_BAUD_9600);					// Start the serial port at 9600 kbps
}

void loop()
{
	Serial.println("Scanning I2C bus...");

	nDevices = 0;
	for (iAddrI2C = 1; iAddrI2C < I2C_ADDRESS_MAX; iAddrI2C++)		// Scan all I2C addresses
	{
		Wire.beginTransmission(iAddrI2C);
		error = Wire.endTransmission();

		if (error == I2C_XMIT_END_SUCCESS)											// Device found
		{
			nDevices++;											// Increment total number found

			// Print address of device found
			Serial.print("I2C device found at address 0x");
			if (iAddrI2C<16)
				Serial.print("0");
			Serial.println(iAddrI2C, HEX);
			
			switch (iAddrI2C)
			{
				case I2C_ADDR_NUNCHUCK:
					// Black nunchuck: 0xF0, 0x55 & 0xFB, 0x00  (Note: White is only one init pair: 0x40, 0x00)
					Serial.println("This could be a black nunchuck. Attempting initialization.");
					Wire.beginTransmission(I2C_ADDR_NUNCHUCK);
					Wire.write(0xF0);
					Wire.write(0x55);
					Wire.endTransmission();
					delay(20);
					Wire.beginTransmission(I2C_ADDR_NUNCHUCK);
					Wire.write(0xFB);
					Wire.write(0x00);
					Wire.endTransmission();
					delay(20);
					
					Serial.println("Sending joystick data request...");
					Wire.beginTransmission(I2C_ADDR_NUNCHUCK);
					Wire.write(0x00);
					Wire.endTransmission();
					
					Wire.requestFrom(I2C_ADDR_NUNCHUCK, 1);
					while (Wire.available() < 1);
					lowByte = Wire.read();
					if (lowByte > 100 && lowByte < 160) {
						Serial.print("Data response normal: ");
						Serial.println(lowByte);
						nunchuck_found = 1;
					}
					else {
						Serial.print("Data response is not normal: ");
						Serial.println(lowByte);
					}
					break;

				case I2C_ADDR_MPU6050_1:
				case I2C_ADDR_MPU6050_2:
					Serial.println("This could be a MPU-6050");
					Wire.beginTransmission(iAddrI2C);
					Wire.write(0x75);
					Wire.endTransmission();
					Serial.println("Send Who am I request...");
					Wire.requestFrom(iAddrI2C, 1);
					while (Wire.available() < 1);
					lowByte = Wire.read();
					
					if (lowByte == I2C_ADDR_MPU6050_1) {
						Serial.print("Who Am I responce is ok: 0x");
						Serial.println(lowByte, HEX);
					}
					else {
						Serial.print("Wrong Who Am I responce: 0x");
						if (lowByte<16)Serial.print("0");
						Serial.println(lowByte, HEX);
					}
					if (lowByte == I2C_ADDR_MPU6050_1 && iAddrI2C == I2C_ADDR_MPU6050_1) {
						MPU_6050_found = 1;
						Serial.println("Starting Gyro....");
						set_gyro_registers();
					}
					break;
				default:
					Serial.println("Unknown device.");
					break;

			}
		}
		else if (error == I2C_XMIT_END_OTHER_ERROR)
		{
			Serial.print("Unknown error at address 0x");
			if (iAddrI2C<16)
				Serial.print("0");
			Serial.println(iAddrI2C, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");
	if (MPU_6050_found) {
		Serial.print("Balance value: ");
		Wire.beginTransmission(I2C_ADDR_MPU6050_1);
		Wire.write(0x3F);
		Wire.endTransmission();
		Wire.requestFrom(I2C_ADDR_MPU6050_1, 2);
		Serial.println((Wire.read() << 8 | Wire.read())*-1);
		delay(20);
		Serial.println("Printing raw gyro values");
		for (iAddrI2C = 0; iAddrI2C < 20; iAddrI2C++) {
			Wire.beginTransmission(I2C_ADDR_MPU6050_1);
			Wire.write(0x43);
			Wire.endTransmission();
			Wire.requestFrom(I2C_ADDR_MPU6050_1, 6);
			while (Wire.available() < 6);
			Serial.print("Gyro X = ");
			Serial.print(Wire.read() << 8 | Wire.read());
			Serial.print(" Gyro Y = ");
			Serial.print(Wire.read() << 8 | Wire.read());
			Serial.print(" Gyro Z = ");
			Serial.println(Wire.read() << 8 | Wire.read());
		}
		Serial.println("");
	}
	else Serial.println("No MPU-6050 device found at address 0x68");

	if (nunchuck_found) {
		Serial.println("Printing raw Nunchuck values");
		for (iAddrI2C = 0; iAddrI2C < 20; iAddrI2C++) {
			Wire.beginTransmission(0x52);
			Wire.write(0x00);
			Wire.endTransmission();
			Wire.requestFrom(0x52, 2);
			while (Wire.available() < 2);
			Serial.print("Joystick X = ");
			Serial.print(Wire.read());
			Serial.print(" Joystick y = ");
			Serial.println(Wire.read());
			delay(100);
		}
	}
	else Serial.println("No Nunchuck device found at address 0x52");
	while (1);
}

void set_gyro_registers() {
	//Setup the MPU-6050
	Wire.beginTransmission(I2C_ADDR_MPU6050_1);                                     //Start communication with the address found during search.
	Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
	Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
	Wire.endTransmission();                                                   //End the transmission with the gyro.

	Wire.beginTransmission(I2C_ADDR_MPU6050_1);                                     //Start communication with the address found during search.
	Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
	Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
	Wire.endTransmission();                                                   //End the transmission with the gyro

	Wire.beginTransmission(I2C_ADDR_MPU6050_1);                                     //Start communication with the address found during search.
	Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
	Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
	Wire.endTransmission();                                                   //End the transmission with the gyro

	Wire.beginTransmission(I2C_ADDR_MPU6050_1);                                     //Start communication with the address found during search
	Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
	Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	Wire.endTransmission();                                                   //End the transmission with the gyro 
}









