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

#include <Wire.h>                                            //Include the Wire.h library so we can communicate with the gyro
#include "BalanceBot.h"

//
//		Declaring global variables
//
byte	start, received_byte, low_bat;

int		left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int		right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int		battery_voltage;
int		receive_counter;
int		gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long	gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float	angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float	pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float	pid_output_left, pid_output_right;


//
//		Setup
//
void setup()
{
	int		receive_counter;


	Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
	Wire.begin();                                                             //Start the I2C bus as master
	TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

																			  
	// To create a variable pulse for controlling the stepper motors, a timer is created that will execute
	// an interrupt procedure (TIMER2_COMPA_vect) every 20us
		//Make sure that the TCCR2A register is set to zero
	//Make sure that the TCCR2A register is set to zero
	//Set the interupt enable bit OCIE2A in the TIMSK2 register
	//Set the CS21 bit in the TCCRB register to set the prescaler to 8
	//The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
	//Set counter 2 to CTC (clear timer on compare) mode

	TCCR2A = 0;                                                               
	TCCR2B = 0;                                                               
	TIMSK2 |= (1 << OCIE2A);                                                  
	TCCR2B |= (1 << CS21);                                                    
	OCR2A = 39;                                                               
	TCCR2A |= (1 << WGM21);                                                   

	// By default the MPU-6050 sleeps. So we have to wake it up.
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_PWR_MGMT_1, MPU6050_REG_PWR_MGMT_1_ACTIVATE_GYRO);
	// Set the full scale of the gyro to +/- 250 degrees per second
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_GYRO_CONFIG, MPU6050_REG_GYRO_CONFIG_250DPS_FULL_SCALE);
	// Set the full scale of the accelerometer to +/- 4g.
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_ACCEL_CONFIG, MPU6050_REG_ACCEL_CONFIG_4G_FULL_SCALE);
	// Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_CONFIG, MPU6050_REG_CONFIG_LOW_PASS_43HZ);

	//Configure digital output port pins
	pinMode (2, OUTPUT);
	pinMode (3, OUTPUT);
	pinMode (4, OUTPUT);
	pinMode (5, OUTPUT);
	pinMode (13, OUTPUT);

	// Loop 500 times
	for (receive_counter = 0; receive_counter < 500; receive_counter++) {
		
		// Toggle the LED every 15 loops
		if (receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));

		i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_RAW_GYRO_VALUES, 4);

		// Get the yaw and pitch calibration values (read and combine 2 bytes each)
		gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();
		gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();

		delayMicroseconds(3700);
	}
	
	// Divide by 500 to get the average offsets
	gyro_pitch_calibration_value /= 500;
	gyro_yaw_calibration_value /= 500;

	//Set the loop_timer to the next end loop time
	loop_timer = micros() + 4000;
}

//
//		Main program loop
//
void loop()
{
	int		receive_counter;


	// If serial data available, retrieve it and reset counter
	if (Serial.available()) {                                                   
		received_byte = Serial.read();
		receive_counter = 0;
	}
	
	//The received byte will be valid for 25 program loops (100 milliseconds), then cleared
	if (receive_counter <= 25)
		receive_counter++;
	else
		received_byte = 0x00;

	// Test the battery voltage from pin A0.  The voltage at the pin will be the battery voltage minus
	// the diode's forward drop (0.7 volts), divided by the resistor network (0.4)	A fully charged 3S pack, 11.1 volt
	// LIPO battery is at 12.6 volts, fully charged, and 9.0 volts discharged.  At pin A0, this range is
	// 4.76 - 3.32 volts.  The ADC value at A0 for this range is roughly 975 - 780.
	battery_voltage = (analogRead(0) * 1.222) + 85;

	//If battery voltage is 8.0 - 10.5V, turn on the LED and set the low_bat variable to 1
	if (battery_voltage < 1050 && battery_voltage > 800) {
		digitalWrite(13, HIGH);
		low_bat = 1;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Angle calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_BALANCE_VALUE, 2);

	accelerometer_data_raw = Wire.read() << 8 | Wire.read();                      //Combine the two bytes to make one integer
	accelerometer_data_raw += ACC_CALIBRATION_VALUE;                          //Add the accelerometer calibration value
	if (accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
	if (accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;

	angle_acc = asin((float)accelerometer_data_raw / 8200.0)* 57.296;           //Calculate the current angle according to the accelerometer

	if (start == 0 && angle_acc > -0.5&& angle_acc < 0.5) {                     //If the accelerometer angle is almost 0
		angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
		start = 1;                                                              //Set the start variable to start the PID controller
	}

	i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_RAW_GYRO_VALUES, 4);
	
	gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();                           //Combine the two bytes to make one integer
	gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();                         //Combine the two bytes to make one integer

	gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
	angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		MPU-6050 offset compensation
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
																			  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
																			  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
																			  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
																			  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

	gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
																			  //Uncomment the following line to make the compensation active
																			  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

	angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		PID controller calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
																			  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
																			  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
																			  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
	pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
	if (pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015;

	pid_i_mem += PID_I_GAIN * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
	if (pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
	else if (pid_i_mem < -400)pid_i_mem = -400;
	//Calculate the PID output value
	pid_output = PID_P_GAIN * pid_error_temp + pid_i_mem + PID_D_GAIN * (pid_error_temp - pid_last_d_error);
	if (pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
	else if (pid_output < -400)pid_output = -400;

	pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

	if (pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

	if (angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1) {    //If the robot tips over or the start variable is zero or the battery is empty
		pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
		pid_i_mem = 0;                                                          //Reset the I-controller memory
		start = 0;                                                              //Set the start variable to 0
		self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Control calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
	pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor

	if (received_byte & B00000001) {                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
		pid_output_left += TURNING_SPEED;                                       //Increase the left motor speed
		pid_output_right -= TURNING_SPEED;                                      //Decrease the right motor speed
	}
	if (received_byte & B00000010) {                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
		pid_output_left -= TURNING_SPEED;                                       //Decrease the left motor speed
		pid_output_right += TURNING_SPEED;                                      //Increase the right motor speed
	}

	if (received_byte & B00000100) {                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
		if (pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
		if (pid_output > MAX_TARGET_SPEED * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
	}
	if (received_byte & B00001000) {                                            //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
		if (pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
		if (pid_output < MAX_TARGET_SPEED)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
	}

	if (!(received_byte & B00001100)) {                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
		if (pid_setpoint > 0.5)pid_setpoint -= 0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
		else if (pid_setpoint < -0.5)pid_setpoint += 0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
		else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
	}

	//The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
	if (pid_setpoint == 0) {                                                    //If the setpoint is zero degrees
		if (pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
		if (pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Motor pulse calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
	if (pid_output_left > 0)pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
	else if (pid_output_left < 0)pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

	if (pid_output_right > 0)pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
	else if (pid_output_right < 0)pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

	//Calculate the needed pulse time for the left and right stepper motor controllers
	if (pid_output_left > 0)left_motor = 400 - pid_output_left;
	else if (pid_output_left < 0)left_motor = -400 - pid_output_left;
	else left_motor = 0;

	if (pid_output_right > 0)right_motor = 400 - pid_output_right;
	else if (pid_output_right < 0)right_motor = -400 - pid_output_right;
	else right_motor = 0;

	//Copy the pulse time to the throttle variables so the interrupt subroutine can use them
	throttle_left_motor = left_motor;
	throttle_right_motor = right_motor;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Loop time timer
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
	//is created by setting the loop_timer variable to +4000 microseconds every loop.
	while (loop_timer > micros());
	loop_timer += 4000;
}



//
//		Interrupt routine  TIMER2_COMPA_vect
//
ISR(TIMER2_COMPA_vect) {
	//Left motor pulse calculations
	throttle_counter_left_motor++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
	if (throttle_counter_left_motor > throttle_left_motor_memory) {             //If the number of loops is larger then the throttle_left_motor_memory variable
		throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
		throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
		if (throttle_left_motor_memory < 0) {                                     //If the throttle_left_motor_memory is negative
			PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
			throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
		}
		else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
	}
	else if (throttle_counter_left_motor == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
	else if (throttle_counter_left_motor == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 

																			   //right motor pulse calculations
	throttle_counter_right_motor++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
	if (throttle_counter_right_motor > throttle_right_motor_memory) {           //If the number of loops is larger then the throttle_right_motor_memory variable
		throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
		throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
		if (throttle_right_motor_memory < 0) {                                    //If the throttle_right_motor_memory is negative
			PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
			throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
		}
		else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
	}
	else if (throttle_counter_right_motor == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
	else if (throttle_counter_right_motor == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
}


























