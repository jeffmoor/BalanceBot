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
//		Declaring "global" variables
//
// These are used by the interrupt
volatile static int		iMotorLeftThrottle,
						iMotorLeftThrottleCount,
						iMotorLeftThrottleMem,
						iMotorRightThrottle,
						iMotorRightThrottleCount,	
						iMotorRightThrottleMem;

// These are passed to the loop function by the setup function
static long				gyro_yaw_cal_value,
						gyro_pitch_cal_value;

static unsigned long	gulLoopTimer;


//
//		Setup Function
//
void setup()
{
	int		iCount;


	gyro_yaw_cal_value = 0;
	gyro_pitch_cal_value = 0;
	
	Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
	Wire.begin();                                                             //Start the I2C bus as master
	TWBR = 12;                                                                //Set the I2C clock speed to 400kHz
																				  
	// To create a variable pulse for controlling the stepper motors, a timer is created that will execute
	// an interrupt procedure (TIMER2_COMPA_vect) every 20us.  We use 8-bit Timer Counter 2
	TCCR2A = 0;						// Clear control registers A and B                                                               
	TCCR2B = 0;                                                               
	TIMSK2 |= (1 << OCIE2A);		// Set interrupt mask reg to enable the interrupt when compare matches                                                  
	TCCR2B |= (1 << CS21);			// Set prescaler to 8                                                    
	OCR2A = 39;						// Set compare register to 39 (~20us)                                                               
	TCCR2A |= (1 << WGM21);         // Set to clear timer on compare match                                          

	// By default the MPU-6050 sleeps. So we have to wake it up.
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_PWR_MGMT_1, MPU6050_REG_PWR_MGMT_1_ACTIVATE_GYRO);
	
	// Set the full scale of the gyro to +/- 250 degrees per second
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_GYRO_CONFIG, MPU6050_REG_GYRO_CONFIG_250DPS_FULL_SCALE);
	
	// Set the full scale of the accelerometer to +/- 4g.
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_ACCEL_CONFIG, MPU6050_REG_ACCEL_CONFIG_4G_FULL_SCALE);
	
	// Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_CONFIG, MPU6050_REG_CONFIG_LOW_PASS_43HZ);

	//Configure digital output port pins
	pinMode (2, OUTPUT);		// Pins 2 & 3 are the left stepper motor drive
	pinMode (3, OUTPUT);
	pinMode (4, OUTPUT);		// Pins 4 & 5 are the right stepper motor drive
	pinMode (5, OUTPUT);
	pinMode (13, OUTPUT);		// Pin 13 is the LED

	// Read and sum the pitch and yaw values over some number of loops. When done, divide by number of loops to get average value.
	for (iCount = 0; iCount < AVERAGE_ELEMENT_COUNT; iCount++) {
		
		// Toggle the LED every 15 loops (flash to show calibration in progress)
		if (iCount % 15 == 0)
			digitalWrite(13, !digitalRead(13));

		i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_RAW_GYRO_VALUES, 4);

		// Get the yaw and pitch calibration values (read and combine 2 bytes each)
		gyro_yaw_cal_value += Wire.read() << 8 | Wire.read();		// Gyro x-axis
		gyro_pitch_cal_value += Wire.read() << 8 | Wire.read();		// Gyro y-axis

		delayMicroseconds(CAL_LOOP_DELAY);
	}

	gyro_pitch_cal_value /= AVERAGE_ELEMENT_COUNT;
	gyro_yaw_cal_value /= AVERAGE_ELEMENT_COUNT;

	gulLoopTimer = micros() + LOOP_TIME_MS;		// Used as a delay so the loop only runs every 4 ms (250 Hz sample/refresh rate)
}



//
//		Main program loop
//
void loop()
{
	byte	bActive,
			bLowBatt,
			bCmdNunchuck;

	int		iCount = 0;

	int		fGyroPitchRaw,
			fGyroYawRaw,
			iAccelRaw;

	int		iMotorLeft,
			iMotorRight;

	int		iBatteryVoltage;

	float	fGyroAngle,
			fAccelAngle,
			angle,
			self_balance_pid_setpoint;

	float	pid_error_temp,
			pid_i_mem,
			pid_setpoint,
			gyro_input,
			pid_output,
			pid_last_d_error;

	float	pid_output_left,
			pid_output_right;


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Test Battery	(see definition of MIN_BATTERY_A0)
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// If battery is too low, turn on the battery warning LED and set the bLowBatt flag
	iBatteryVoltage = analogRead(0);
	if ((iBatteryVoltage < BATTERY_MIN_A0) && (iBatteryVoltage > BATTERY_MIN_CONNECTED)) {
		digitalWrite(DPIN_LED, HIGH);
		bLowBatt = TRUE;
	}

	// If serial data available from the nunchuck, retrieve it and reset counter.  Since the program loop time is ~4ms, keeping the
	// last data around for 25 loops will be ~100ms.  It is then cleared.  This aids in keeping the robot's movement smooth.
	if (Serial.available()) {
		bCmdNunchuck = Serial.read();
		iCount = 0;
		 
	} else if (iCount < 25)
		iCount++;
	else
		bCmdNunchuck = 0x00;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Angle Calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Read the z-axis accelerometer
	i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_BALANCE_VALUE, 2);

	iAccelRaw = Wire.read() << 8 | Wire.read();					// Combine the two bytes to make one integer
	iAccelRaw += ACC_BALANCE_VALUE;								// Add the accelerometer calibration value
	
	// Ensure the data value is between +/-8192, so asin function gets argument between +/-1
	// Calculate the current angle, in degrees, according to the accelerometer
	iAccelRaw = max(min(MPU6050_ACCEL_4G_LSB, iAccelRaw), -MPU6050_ACCEL_4G_LSB);
	fAccelAngle = asin((float)iAccelRaw / MPU6050_ACCEL_4G_LSB) * DEGREES_PER_RADIAN;

	// If the accelerometer angle is almost 0, set the gyro angle to this small value, and set the robot to active
	if ((bActive == FALSE) && (fAccelAngle > -TRIVIAL_ANGLE) && (fAccelAngle < TRIVIAL_ANGLE)) {
		fGyroAngle = fAccelAngle;													//Load the accelerometer angle in the fGyroAngle variable
		bActive = TRUE;																//Set the start variable to start the PID controller
	}

	// Read the yaw and pitch angles from the gyro
	i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_RAW_GYRO_VALUES, 4);
	fGyroYawRaw = Wire.read() << 8 | Wire.read();
	fGyroPitchRaw = Wire.read() << 8 | Wire.read();
	
	fGyroPitchRaw -= gyro_pitch_cal_value;								// Subtract the gyro pitch calibration value

	// Because the main loop is running at 250Hz, we know the raw gyro value is the approximate movement over the last 4ms.  As set, if the gyro
	// were moving at 1 deg/s, the gyro would output 131, but we are measuring per 4ms, so we multiply by 0.000031 (=1/(250*131)) to give us the
	// traveled angle in the last 4ms.  We then add this to the last position to get our new position.
	fGyroAngle += fGyroPitchRaw * 0.000031;								

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		MPU-6050 Offset Compensation
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Not every gyro is mounted 100% level with the axis of the robot.  As a result the robot will not rotate at the exact same spot and
	// start to make larger and larger circles.  To compensate for this behavior a VERY SMALL compensation angle is needed when the robot
	// is rotating.  Try 0.0000003 or -0.0000003 first to see if there is any improvement.

	fGyroYawRaw -= gyro_yaw_cal_value;									//Add the gyro calibration value

	// fGyroAngle -= fGyroYawRaw * 0.0000003;		// Uncomment to activate rotation compensation

	fGyroAngle = fGyroAngle * 0.9996 + fAccelAngle * 0.0004;						// Correct the drift of the gyro angle with the accelerometer angle

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		PID Calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The balancing robot is angle driven.  First the difference between the desired angel (setpoint) and actual angle (process value)
	// is calculated.  The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the
	// time.  The (pid_setpoint - pid_output * 0.015) part functions as a brake function.

	pid_error_temp = fGyroAngle - self_balance_pid_setpoint - pid_setpoint;

	if (pid_output > 10 || pid_output < -10)
		pid_error_temp += pid_output * 0.015;

	pid_i_mem += PID_I_GAIN * pid_error_temp;                  //Calculate the I-controller value and add it to the pid_i_mem variable

	if (pid_i_mem > 400)
		pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
	else if (pid_i_mem < -400)
		pid_i_mem = -400;

	//Calculate the PID output value
	pid_output = PID_P_GAIN * pid_error_temp + pid_i_mem + PID_D_GAIN * (pid_error_temp - pid_last_d_error);

	if (pid_output > 400)
		pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
	else if (pid_output < -400)
		pid_output = -400;

	pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

	if (pid_output < 5 && pid_output > -5)
		pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

	if (fGyroAngle > 30 || fGyroAngle < -30 || bActive == 0 || bLowBatt == 1) {    //If the robot tips over or the start variable is zero or the battery is empty
		pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
		pid_i_mem = 0;                                                          //Reset the I-controller memory
		bActive = 0;                                                              //Set the start variable to 0
		self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Nunchuck Control Calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	pid_output_left = pid_output;						// Copy the controller output to the pid_output_left variable for the left motor
	pid_output_right = pid_output;						// Copy the controller output to the pid_output_right variable for the right motor

	if (bCmdNunchuck & BIT_1) {						// Bit 1 set: Turn left
		pid_output_left -= TURNING_SPEED;				// Decrease the left motor speed
		pid_output_right += TURNING_SPEED;				// Increase the right motor speed
	}

	if (bCmdNunchuck & BIT_2) {						// Bit 2 set: Turn right
		pid_output_left += TURNING_SPEED;				// Increase the left motor speed
		pid_output_right -= TURNING_SPEED;				// Decrease the right motor speed
	}

	if (bCmdNunchuck & BIT_3) {						// Bit 3 set: Move forward (slowly change the setpoint angle so the robot starts leaning forward)
		if (pid_setpoint > -2.5)
			pid_setpoint -= 0.05;
		if (pid_output > -MAX_TARGET_SPEED)
			pid_setpoint -= 0.005;
	}

	if (bCmdNunchuck & BIT_4) {						// Bit 4 set: Move backward (lowly change the setpoint angle so the robot starts leaning backward)
		if (pid_setpoint < 2.5)
			pid_setpoint += 0.05;
		if (pid_output < MAX_TARGET_SPEED)
			pid_setpoint += 0.005;
	}

	if (!(bCmdNunchuck & B00001100)) {						//Slowly reduce the setpoint to zero if no foreward or backward command is given
		if (pid_setpoint > 0.5)
			pid_setpoint -= 0.05;				//If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
		else if (pid_setpoint < -0.5)
			pid_setpoint += 0.05;			//If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
		else
			pid_setpoint = 0;								//If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
	}

	//The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
	if (pid_setpoint == 0) {						//If the setpoint is zero degrees
		if (pid_output < 0)
			self_balance_pid_setpoint += 0.0015;						//Increase the self_balance_pid_setpoint if the robot is still moving forewards
		if (pid_output > 0)
			self_balance_pid_setpoint -= 0.0015;						//Decrease the self_balance_pid_setpoint if the robot is still moving backwards
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Motor pulse calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
	if (pid_output_left > 0)
		pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
	else if (pid_output_left < 0)
		pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

	if (pid_output_right > 0)pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
	else if (pid_output_right < 0)pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

	//Calculate the needed pulse time for the left and right stepper motor controllers
	if (pid_output_left > 0)
		iMotorLeft = 400 - pid_output_left;
	else if (pid_output_left < 0)
		iMotorLeft = -400 - pid_output_left;
	else
		iMotorLeft = 0;

	if (pid_output_right > 0)
		iMotorRight = 400 - pid_output_right;
	else if (pid_output_right < 0)
		iMotorRight = -400 - pid_output_right;
	else
		iMotorRight = 0;

	//Copy the pulse time to the throttle variables so the interrupt subroutine can use them
	iMotorLeftThrottle = iMotorLeft;
	iMotorRightThrottle = iMotorRight;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Loop timer
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The angle calculations are tuned for a loop time of 4 milliseconds (250 Hz). To make sure every loop is
	// exactly 4 milliseconds a wait loop is entered to delay until 4 ms has passed since the end of the last loop.
	while (gulLoopTimer > micros());
	gulLoopTimer += LOOP_TIME_MS;
}



//
//		Interrupt routine  TIMER2_COMPA_vect
//
ISR(TIMER2_COMPA_vect)
{
	//Left motor pulse calculations
	iMotorLeftThrottleCount++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
	if (iMotorLeftThrottleCount > iMotorLeftThrottleMem) {             //If the number of loops is larger then the throttle_left_motor_memory variable
		iMotorLeftThrottleCount = 0;                                        //Reset the throttle_counter_left_motor variable
		iMotorLeftThrottleMem = iMotorLeftThrottle;                       //Load the next throttle_left_motor variable
		if (iMotorLeftThrottleMem < 0) {                                     //If the throttle_left_motor_memory is negative
			PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
			iMotorLeftThrottleMem *= -1;                                     //Invert the throttle_left_motor_memory variable
		}
		else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
	}
	else if (iMotorLeftThrottleCount == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
	else if (iMotorLeftThrottleCount == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 

	// Right motor pulse calculations
	iMotorRightThrottleCount++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
	if (iMotorRightThrottleCount > iMotorRightThrottleMem) {           //If the number of loops is larger then the throttle_right_motor_memory variable
		iMotorRightThrottleCount = 0;                                       //Reset the throttle_counter_right_motor variable
		iMotorRightThrottleMem = iMotorRightThrottle;                     //Load the next throttle_right_motor variable
		if (iMotorRightThrottleMem < 0) {                                    //If the throttle_right_motor_memory is negative
			PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
			iMotorRightThrottleMem *= -1;                                    //Invert the throttle_right_motor_memory variable
		}
		else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
	}
	else if (iMotorRightThrottleCount == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
	else if (iMotorRightThrottleCount == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
}




