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
volatile static int		giMotorLeftThrottle,
						giMotorLeftThrottleCount,
						giMotorLeftThrottleMem,
						giMotorRightThrottle,
						giMotorRightThrottleCount,	
						giMotorRightThrottleMem;

// These are passed to the loop function by the setup function
static long				glGyroYawCal,
						glGyroPitchCal;

static unsigned long	gulLoopTimer;			// Used to keep the loop frequency to 250 Hz (every 4ms)


// These are tracked from loop to loop
static byte				gbActive,				// Track if the robot is active or not
						gbLowBatt,
						gbCmdNunchuck;			// Track the last received command from the nunchuck

static int				giCmdLoopCount;			// Number of loops for which the last command has been active

static float			gfCurrentAngle,
						gfSelfBalancePidSetpoint,
						gfPidIGainMem,
						gfDesiredAngle,
						gfPidOutput,
						gfLastLoopPidError,
						gfPidOutputLeft,
						gfPidOutputRight;

//
//		Setup Function
//
void setup()
{
	glGyroYawCal = 0;
	glGyroPitchCal = 0;

	Serial.begin(9600);                                                       //Start the serial port at 9600 kbps
	Wire.begin();                                                             //Start the I2C bus as master
	TWBR = 12;                                                                //Set the I2C clock speed to 400kHz

																			  // To create a variable pulse for controlling the stepper motors, use 8-bit Counter 2 to execute
																			  // an interrupt procedure (TIMER2_COMPA_vect) every 20us.
	TCCR2A = 0;						// Clear control registers A and B                                                               
	TCCR2B = 0;
	TIMSK2 |= (1 << OCIE2A);		// Set interrupt mask reg to enable the interrupt when compare matches                                                  
	TCCR2B |= (1 << CS21);			// Set prescaler to 8                                                    
	OCR2A = 39;						// Set compare register to 39 (~20us)                                                               
	TCCR2A |= (1 << WGM21);         // Set to clear timer on compare match                                          

									// Wake up the MPU6050 and configure it.  Set the gyro full-scale to +/- 250 deg/s,
									// the accelerometer to +/- 4g, and the digital low pass filter to ~43 Hz.
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_PWR_MGMT_1, MPU6050_REG_PWR_MGMT_1_ACTIVATE_GYRO);
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_GYRO_CONFIG, MPU6050_REG_GYRO_CONFIG_250DPS_FULL_SCALE);
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_ACCEL_CONFIG, MPU6050_REG_ACCEL_CONFIG_4G_FULL_SCALE);
	i2cRegisterWrite1(I2C_ADDR_MPU6050_1, MPU6050_REG_CONFIG, MPU6050_REG_CONFIG_LOW_PASS_43HZ);

	//Configure digital output port pins
	pinMode(2, OUTPUT);		// Pins 2 & 3 are the left stepper motor drive
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);		// Pins 4 & 5 are the right stepper motor drive
	pinMode(5, OUTPUT);
	pinMode(13, OUTPUT);		// Pin 13 is the LED

								// Read and sum the pitch and yaw values over some number of loops. When done, divide by number of loops to get average value.
	for (int iCnt = 0; iCnt < CAL_AVERAGE_COUNT; iCnt++) {

		// Toggle the LED every 15 loops (flash to show calibration in progress)
		if (iCnt % 15 == 0)
			digitalWrite(13, !digitalRead(13));

		i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_RAW_GYRO_VALUES, 4);

		// Get the yaw and pitch calibration values (read and combine 2 bytes each)
		glGyroYawCal += Wire.read() << 8 | Wire.read();		// Gyro x-axis
		glGyroPitchCal += Wire.read() << 8 | Wire.read();		// Gyro y-axis

		delayMicroseconds(CAL_LOOP_DELAY);
	}

	glGyroPitchCal /= CAL_AVERAGE_COUNT;
	glGyroYawCal /= CAL_AVERAGE_COUNT;

	// Initialize the loop time counter, setting the sample/refresh rate at 4ms (250 Hz)
	gulLoopTimer = micros() + LOOP_TIME_MS;
}


//
//		Main program loop
//
void loop()
{
	int		iGyroPitchRaw, iGyroYawRaw, iAccelRaw,
			iMotorLeft, iMotorRight,
			iBatteryVoltage;

	float	fAccelAngle;

	float	fPidErrorTemp;


	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Test Battery	(see definitions of BATTERY_MIN_A0 and BATTERY_MIN_CONNECTED)
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// If battery is too low, turn on the battery warning LED and set the gbLowBatt flag
	iBatteryVoltage = analogRead(0);
	if ((iBatteryVoltage < BATTERY_MIN_A0) && (iBatteryVoltage > BATTERY_MIN_CONNECTED)) {
		digitalWrite(DPIN_LED, HIGH);
		gbLowBatt = TRUE;
	}

	// If serial data available from the nunchuck, retrieve it and reset counter.  Since the program loop time is ~4ms, keeping the
	// last data around for 25 loops will be ~100ms.  It is then cleared.  This aids in keeping the robot's movement smooth.
	if (Serial.available()) {
		gbCmdNunchuck = Serial.read();
		giCmdLoopCount = 0;
		 
	} else if (giCmdLoopCount < 25)
		giCmdLoopCount++;
	else
		gbCmdNunchuck = 0x00;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Angle Calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Read the z-axis accelerometer and subtract the calibration value found in the HWTest run
	i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_BALANCE_VALUE, 2);
	iAccelRaw = Wire.read() << 8 | Wire.read();
	iAccelRaw -= ACC_BALANCE_VALUE;
	
	// Ensure the raw, adjusted value is between +/-8192, so asin function gets an argument between +/-1
	iAccelRaw = max(min(MPU6050_ACCEL_4G_LSB, iAccelRaw), -MPU6050_ACCEL_4G_LSB);
	
	// Calculate the current angle, in degrees, according to the accelerometer
	fAccelAngle = asin((float)iAccelRaw / MPU6050_ACCEL_4G_LSB) * DEGREES_PER_RADIAN;

	// If the robot has been off, and is now being stood up, initialize the current position measurement using the
	// accelerometer when the angle gets to within 0.5 degrees of vertical.  Once initialized, it will henceforth be
	// updated using measurememnts from the gyro.
	if ((gbActive == FALSE) && (fAccelAngle > -TRIVIAL_ANGLE) && (fAccelAngle < TRIVIAL_ANGLE)) {
		gfCurrentAngle = fAccelAngle;
		gbActive = TRUE;
	}

	//  Should the other calculations be omitted from the first loop???

	// Read the yaw and pitch angles from the gyro and subtract the calibration values found in Setup()
	i2cRegisterReadStart(I2C_ADDR_MPU6050_1, MPU6050_REG_RAW_GYRO_VALUES, 4);
	iGyroYawRaw = Wire.read() << 8 | Wire.read();
	iGyroYawRaw -= glGyroYawCal;
	iGyroPitchRaw = Wire.read() << 8 | Wire.read();
	iGyroPitchRaw -= glGyroPitchCal;

	// Because the main loop is running at 250Hz, we know the raw gyro value is the approximate movement over the last 4ms.  As set, if the gyro
	// were moving at 1 deg/s, the gyro would output 131, but we are measuring per 4ms, so we multiply by 0.000031 (=1/(250*131)) to give us the
	// traveled angle in the last 4ms.  We then add this to the last position to get our new position.
	gfCurrentAngle += iGyroPitchRaw * 0.0000305;

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		MPU-6050 Offset Compensation
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Not every gyro is mounted 100% level with the axis of the robot.  As a result, if turning "on a dime" the robot will start to make
	// larger and larger circles.  To compensate for this behavior a VERY SMALL compensation angle is needed when the robot is rotating.
	// Try 0.0000003 or -0.0000003 first to see if there is any improvement.

	// If the MPU6050 isn't mounted exactly perpendicular to the world, the pitch angle can vary as the robot goes around a circle,
	// cusing it to speed up and make larger circles.  This line combats this by introducing a very small correction to the angle based
	// on the yaw angle. Uncomment this only if this correction is needed.
	//gfCurrentAngle -= iGyroYawRaw * 0.0000003;

	gfCurrentAngle = gfCurrentAngle * 0.9996 + fAccelAngle * 0.0004;		// Correct the drift of the gyro angle with the accelerometer angle

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		PID Calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The balancing robot is angle driven.  First the difference between the desired angel (setpoint) and actual angle (process value)
	// is calculated.  The gfSelfBalancePidSetpoint variable is automatically changed to make sure that the robot stays balanced all the
	// time.  The (gfDesiredAngle - gfPidOutput * 0.015) part functions as a brake function.

	fPidErrorTemp = gfCurrentAngle - gfSelfBalancePidSetpoint - gfDesiredAngle;

	// Parachute brake. If the gfPidOutput is high, lean back the opposite direction a bit.
	if (gfPidOutput > 10 || gfPidOutput < -10)
		fPidErrorTemp += gfPidOutput * 0.015;

	gfPidIGainMem += PID_I_GAIN * fPidErrorTemp;                  //Calculate the I-controller value and add it to the gfPidIGainMem variable

	if (gfPidIGainMem > 400)
		gfPidIGainMem = 400;                                       //Limit the I-controller to the maximum controller output
	else if (gfPidIGainMem < -400)
		gfPidIGainMem = -400;

	//Calculate the PID output value
	gfPidOutput = PID_P_GAIN * fPidErrorTemp + gfPidIGainMem + PID_D_GAIN * (fPidErrorTemp - gfLastLoopPidError);

	if (gfPidOutput > 400)
		gfPidOutput = 400;                                     //Limit the PI-controller to the maximum controller output
	else if (gfPidOutput < -400)
		gfPidOutput = -400;

	gfLastLoopPidError = fPidErrorTemp;                                        //Store the error for the next loop

	// Create a dead-band to stop the motors when the robot is balanced
	if (gfPidOutput < 5 && gfPidOutput > -5)
		gfPidOutput = 0;

	// If the robot tips over, the start variable is zero, or the battery is empty, shut it down
	if (gfCurrentAngle > 30 || gfCurrentAngle < -30 || gbActive == 0 || gbLowBatt == 1) {
		gfPidOutput = 0;											// Set the PID controller output to 0 so the motors stop moving
		gfPidIGainMem = 0;											// Reset the I-controller memory
		gbActive = FALSE;											// Set the start variable to 0
		gfSelfBalancePidSetpoint = 0;							// Reset the gfSelfBalancePidSetpoint variable
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Nunchuck Control Calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	gfPidOutputLeft = gfPidOutput;						// Copy the controller output to the gfPidOutputLeft variable for the left motor
	gfPidOutputRight = gfPidOutput;						// Copy the controller output to the gfPidOutputRight variable for the right motor

	if (gbCmdNunchuck & BIT_1) {						// Bit 1 set: Turn right
		gfPidOutputLeft += TURNING_SPEED;				// Increase the left motor speed
		gfPidOutputRight -= TURNING_SPEED;				// Decrease the right motor speed
	}

	if (gbCmdNunchuck & BIT_2) {						// Bit 2 set: Turn left
		gfPidOutputLeft -= TURNING_SPEED;				// Decrease the left motor speed
		gfPidOutputRight += TURNING_SPEED;				// Increase the right motor speed
	}

	if (gbCmdNunchuck & BIT_3) {						// Bit 3 set: Move forward (slowly change the setpoint angle so the robot starts leaning forward)
		if (gfDesiredAngle > -2.5)						// Shooting for an angle of -2.5 degrees by slowly deccrementing by 0.05 per loop.  This makes the movement more fluid.
			gfDesiredAngle -= 0.05;
		if (gfPidOutput > -MAX_TARGET_SPEED)				// Decrease the angle further, slowly, until the output greater than the maximum speed (shag carpet)
			gfDesiredAngle -= 0.005;
	}

	if (gbCmdNunchuck & BIT_4) {						// Bit 4 set: Move backward (slowly change the setpoint angle so the robot starts leaning backward)
		if (gfDesiredAngle < 2.5)
			gfDesiredAngle += 0.05;
		if (gfPidOutput < MAX_TARGET_SPEED)
			gfDesiredAngle += 0.005;
	}

	if (!(gbCmdNunchuck & B00001100)) {				// Neither bits 3 or 4: Slowly reduce the desired angle to zero
		if (gfDesiredAngle > 0.5)						// If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
			gfDesiredAngle -= 0.05;				
		else if (gfDesiredAngle < -0.5)
			gfDesiredAngle += 0.05;						// If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
		else
			gfDesiredAngle = 0;							// If the PID setpoint is between -0.5 and 0.5 degrees, set the setpoint to 0
	}

	// It's almost impossible to perfectly balance the robot and make it come to a complete standstill.  Also, a surface with a small angle
	// will cause the robot to move.   When the setpoint (gfDesiredAngle) is zero, the robot should come to a complete stop.  We monitor the
	// speed of the robot (gfPidOutput) and adapt the gfSelfBalancePidSetpoint to change the robot's position until it comes to a complete stop.
	if (gfDesiredAngle == 0) {
		if (gfPidOutput < 0)
			gfSelfBalancePidSetpoint += 0.0015;
		if (gfPidOutput > 0)
			gfSelfBalancePidSetpoint -= 0.0015;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		Motor pulse calculations
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// To compensate for the logarithmic behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
	if (gfPidOutputLeft > 0)
		gfPidOutputLeft = 405 - (1 / (gfPidOutputLeft + 9)) * 5500;
	else if (gfPidOutputLeft < 0)
		gfPidOutputLeft = -405 - (1 / (gfPidOutputLeft - 9)) * 5500;

	if (gfPidOutputRight > 0)
		gfPidOutputRight = 405 - (1 / (gfPidOutputRight + 9)) * 5500;
	else if (gfPidOutputRight < 0)
		gfPidOutputRight = -405 - (1 / (gfPidOutputRight - 9)) * 5500;

	//Calculate the needed pulse time for the left and right stepper motor controllers
	if (gfPidOutputLeft > 0)
		iMotorLeft = 400 - gfPidOutputLeft;
	else if (gfPidOutputLeft < 0)
		iMotorLeft = -400 - gfPidOutputLeft;
	else
		iMotorLeft = 0;

	if (gfPidOutputRight > 0)
		iMotorRight = 400 - gfPidOutputRight;
	else if (gfPidOutputRight < 0)
		iMotorRight = -400 - gfPidOutputRight;
	else
		iMotorRight = 0;

	//Copy the pulse time to the throttle variables so the interrupt subroutine can use them
	giMotorLeftThrottle = iMotorLeft;
	giMotorRightThrottle = iMotorRight;

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
	giMotorLeftThrottleCount++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
	if (giMotorLeftThrottleCount > giMotorLeftThrottleMem) {             //If the number of loops is larger then the throttle_left_motor_memory variable
		giMotorLeftThrottleCount = 0;                                        //Reset the throttle_counter_left_motor variable
		giMotorLeftThrottleMem = giMotorLeftThrottle;                       //Load the next throttle_left_motor variable
		if (giMotorLeftThrottleMem < 0) {                                     //If the throttle_left_motor_memory is negative
			PORTD &= 0b11110111;                                                  //Set output 3 low to reverse the direction of the stepper controller
			giMotorLeftThrottleMem *= -1;                                     //Invert the throttle_left_motor_memory variable
		}
		else PORTD |= 0b00001000;                                               //Set output 3 high for a forward direction of the stepper motor
	}
	else if (giMotorLeftThrottleCount == 1)PORTD |= 0b00000100;             //Set output 2 high to create a pulse for the stepper controller
	else if (giMotorLeftThrottleCount == 2)PORTD &= 0b11111011;             //Set output 2 low because the pulse only has to last for 20us 

	// Right motor pulse calculations
	giMotorRightThrottleCount++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
	if (giMotorRightThrottleCount > giMotorRightThrottleMem) {           //If the number of loops is larger then the throttle_right_motor_memory variable
		giMotorRightThrottleCount = 0;                                       //Reset the throttle_counter_right_motor variable
		giMotorRightThrottleMem = giMotorRightThrottle;                     //Load the next throttle_right_motor variable
		if (giMotorRightThrottleMem < 0) {                                    //If the throttle_right_motor_memory is negative
			PORTD |= 0b00100000;                                                  //Set output 5 low to reverse the direction of the stepper controller
			giMotorRightThrottleMem *= -1;                                    //Invert the throttle_right_motor_memory variable
		}
		else PORTD &= 0b11011111;                                               //Set output 5 high for a forward direction of the stepper motor
	}
	else if (giMotorRightThrottleCount == 1)PORTD |= 0b00010000;            //Set output 4 high to create a pulse for the stepper controller
	else if (giMotorRightThrottleCount == 2)PORTD &= 0b11101111;            //Set output 4 low because the pulse only has to last for 20us
}






