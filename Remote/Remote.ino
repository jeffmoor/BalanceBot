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
#include <Wire.h>	//Include the Wire.h library so we can communicate with the Nunchuck
#include "BalanceBot.h"


void setup() {
	Serial.begin(USART_BAUD_9600);                                    // Start the serial port at 9600 kbps
	Wire.begin();                                                     // Start the I2C as master
	TWBR = 12;                                                        // Set the I2C clock speed to 400kHz		(**  Inteface doc says 100KHz  **)
	Wire.begin();                                                     // Start the I2C bus as master
	delay(20);                                                        // Short delay

	Wire.beginTransmission(I2C_ADDR_NUNCHUCK);                        // Start communication with the Nunchuck
	Wire.write(I2C_ADDR_NUNCHUCK_BLK_REG1_ADDR);                      // We want to write to register (F0 hex)
	Wire.write(I2C_ADDR_NUNCHUCK_BLK_REG1_DATA);                      // Set the register bits as 01010101
	Wire.endTransmission();                                           // End the transmission
	delay(20);                                                        // Short delay

	Wire.beginTransmission(I2C_ADDR_NUNCHUCK);                        // Start communication with the Nunchuck
	Wire.write(I2C_ADDR_NUNCHUCK_BLK_REG2_ADDR);                      // We want to write to register (FB hex)
	Wire.write(I2C_ADDR_NUNCHUCK_BLK_REG2_DATA);                      // Set the register bits as 00000000
	Wire.endTransmission();                                           // End the transmission
	delay(20);                                                        // Short delay
}

void loop() {

	byte		received_data[6],
				send_byte;


	Wire.beginTransmission(I2C_ADDR_NUNCHUCK);                        // Start communication with the Nunchuck.
	Wire.write(NUNCHUCK_REG_XY_VALUES);                               // We want to start reading at register (00 hex)
	Wire.endTransmission();                                           // End the transmission
	Wire.requestFrom(I2C_ADDR_NUNCHUCK, 6);                           // Request 6 bytes from the Nunchuck
	
	for (byte i = 0; i < 6; i++)
		received_data[i] = Wire.read();
	
	send_byte = B00000000;                                            //Set the send_byte variable to 0
	
	// Stick center is 128 for x and y axes.  Test the read data, x in the first byte and y in the second byte.  Sufficiently low values
	// command the bot back or to the left.  Sufficiently high values command the bot forward or to the right.
	if (received_data[0] < NUNCHUCK_LOW_TRIGGER)
		send_byte |= BIT_1;						// x value is low, set bit 1 to command left
	else if (received_data[0] > NUNCHUCK_HIGH_TRIGGER)
		send_byte |= BIT_2;						// x value is high, set bit 2 to command right

	if (received_data[1] < NUNCHUCK_LOW_TRIGGER)
		send_byte |= BIT_3;						// y value is low, set bit 3 to command back
	else if (received_data[1] > NUNCHUCK_HIGH_TRIGGER)
		send_byte |= BIT_4;						// y value is high, set bit 4 to command forward

	if (send_byte)
		Serial.print((char)send_byte);                       //Send the send_byte variable if it's value is larger then 0

	delay(40);                                                        //Create a 40 millisecond loop delay
}



