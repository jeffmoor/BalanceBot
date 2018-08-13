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
	for (iAddrI2C = 1; iAddrI2C < I2C_ADDRESS_MAX; iAddrI2C++)	// Scan all 7-bit I2C addresses
	{
		Wire.beginTransmission(iAddrI2C);
		error = Wire.endTransmission();

		if (error == I2C_XMIT_END_SUCCESS)						// Device found
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
					// Black nunchuck, two init registers: 0xF0, 0x55 & 0xFB, 0x00  (Note: White is only one register init pair: 0x40, 0x00)
					Serial.println("This could be a black nunchuck. Attempting initialization.");
					i2cRegisterWrite1(I2C_ADDR_NUNCHUCK, I2C_ADDR_NUNCHUCK_BLK_REG1_ADDR, I2C_ADDR_NUNCHUCK_BLK_REG1_DATA);
					delay(20);
					i2cRegisterWrite1(I2C_ADDR_NUNCHUCK, I2C_ADDR_NUNCHUCK_BLK_REG2_ADDR, I2C_ADDR_NUNCHUCK_BLK_REG2_DATA);
					delay(20);
					
					Serial.println("Sending joystick data request...");

					i2cRegisterReadStart(I2C_ADDR_NUNCHUCK, 0x00, 1);
					
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
					Serial.println("This could be a MPU-6050 Accel/Gyro");

					Serial.println("Sending 'Who Am I' request...");
					i2cRegisterReadStart(iAddrI2C, MPU6050_REG_WHO_AM_I, 1);

					while (Wire.available() < 1);
					lowByte = Wire.read();
					
					if (lowByte == I2C_ADDR_MPU6050_1) {
						Serial.print("Who Am I responce is ok: 0x");
						Serial.println(lowByte, HEX);
					}
					else {
						Serial.print("Wrong Who Am I responce: 0x");
						if (lowByte<16)
							Serial.print("0");
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
	}		// Scan all addresses
	
	Serial.println("Done.\n");
	if (nDevices == 0)
		Serial.println("No I2C devices found.\n");
		
	if (MPU_6050_found) {
		Serial.print("Balance value: ");

		i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_BALANCE_VALUE, 2);

		Serial.println((Wire.read() << 8 | Wire.read())*-1);
		delay(20);
		Serial.println("Printing raw gyro values");
		for (iAddrI2C = 0; iAddrI2C < 20; iAddrI2C++) {

			i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_RAW_GYRO_VALUES, 6);
			
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

			i2cRegisterReadStart(I2C_ADDR_NUNCHUCK, NUNCHUCK_REG_XY_VALUES, 2);

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
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_PWR_MGMT_1, MPU6050_REG_PWR_MGMT_1_ACTIVATE_GYRO);
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_GYRO_CONFIG, MPU6050_REG_GYRO_CONFIG_250DPS_FULL_SCALE);
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_ACCEL_CONFIG, MPU6050_REG_ACCEL_CONFIG_4G_FULL_SCALE);
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_CONFIG, MPU6050_REG_CONFIG_LOW_PASS_43HZ);
}









