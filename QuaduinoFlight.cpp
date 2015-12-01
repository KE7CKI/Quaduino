#include "QuaduinoFlight.h"
#include "QuaduinoGlobal.h"
#include <Arduino.h>

// =============================================================
// MOTOR CONTROL INITIAL SETUP
// =============================================================

QuaduinoFlight::QuaduinoFlight() {

}

void QuaduinoFlight::init() {
	motorsActive = false;

	minPulseWidth = 1100;
	maxPulseWidth = 1600;
	gravityHover = 200;
	frontMotorOffset = 11;
	rearMotorOffset = 5;
	rightMotorOffset = 7;
	leftMotorOffset = 9;

	consKp = 0.3;
	consKi = 0.003;
	consKd = 0.192;

	aggKp = 0.93;
	aggKi = 0.1;
	aggKd = 0.2;

	angleToSwap = 15;

        writeMotor(FRONT, 0);
        writeMotor(RIGHT, 0);
        writeMotor(REAR, 0);
        writeMotor(LEFT, 0);

	yawPID = &PID(&yawInput, &yawOutput, &yawSetpoint, 4.0, 0.0, 0.0, DIRECT);
	pitchPID = &PID(&pitchInput, &pitchOutput, &pitchSetpoint, 4.0, 0.0, 0.0, DIRECT);
	rollPID = &PID(&rollInput, &rollOutput, &rollSetpoint, 4.0, 0.0, 0.0, DIRECT);

	setupPidControl();
}

// =============================================================
// PID CONTROL INITIAL SETUP
// =============================================================

void QuaduinoFlight::setupPidControl() {
    // A1510 continuous-rotation motor notes:
    //   write() effective range: [37,79]
    //   writeMilliseconds() effective range: [1100,1600]
    //   hover speed is ~1500 with myQuad
    //   PID output range is +/- 200, based on hover speed
  
    (*pitchPID).SetOutputLimits(-200, 200);
    (*pitchPID).SetSampleTime(100); // 10
    (*pitchPID).SetMode(AUTOMATIC);
    pitchTuningActive = true;
  
    (*rollPID).SetOutputLimits(-200, 200);
    (*rollPID).SetSampleTime(10);
    (*rollPID).SetMode(AUTOMATIC);
    rollTuningActive = true;
  
    (*yawPID).SetOutputLimits(-200, 200);
    (*yawPID).SetSampleTime(10);
    (*yawPID).SetMode(AUTOMATIC);
    yawTuningActive = true;
  
    //altitude_tuning_active = false;
  
    yawSetpoint = 0.0; // no rotation
    rollSetpoint = 0.0; // level
    pitchSetpoint = 0.0; // level
    altitudeSetpoint = 0.0; // no change
}

// =============================================================
// WRITE VALUE TO A MOTOR
// =============================================================
void QuaduinoFlight::writeMotor(uint8_t m, uint8_t val) {
  /*
  switch(m) {
    case FRONT: analogWrite(MOTOR1, val); break;
    case RIGHT: analogWrite(MOTOR2, val); break;
    case REAR: analogWrite(MOTOR3, val); break;
    case LEFT: analogWrite(MOTOR4, val); break;
    default: break;
  }
  */
  analogWrite(m, val);
}

// =============================================================
// SET ALL MOTORS TO FULL THROTTLE
// =============================================================

void QuaduinoFlight::runMotorsMax() {
    // set all motors to maximum pulse width (highest speed)
    writeMotor(FRONT, maxPulseWidth);
    writeMotor(RIGHT, maxPulseWidth);
    writeMotor(REAR, maxPulseWidth);
    writeMotor(LEFT, maxPulseWidth);
}


// =============================================================
// SET ALL MOTORS TO ZERO THROTTLE
// =============================================================

void QuaduinoFlight::runMotorsMin() {
    // set all motors to minimum pulse width (lowest speed)
    writeMotor(FRONT, minPulseWidth);
    writeMotor(RIGHT, minPulseWidth);
    writeMotor(REAR, minPulseWidth);
    writeMotor(LEFT, minPulseWidth);
}


// =============================================================
// RUN MOTORS TO FULL THROTTLE, THEN ZERO THROTTLE
// =============================================================

void QuaduinoFlight::runMotorsFullrange() {
    runMotorsMax();
    delay(500);
    runMotorsMin();
    delay(500);
}

void QuaduinoFlight::runMotorsFullrangeGradual() {
	int range = minPulseWidth;
	bool increase = true;

	runMotorsMin();

	while(range <= maxPulseWidth && range >= minPulseWidth) {
		if(increase) {
			range++;
		} else if (increase && range == maxPulseWidth) {
			range--;
			increase = false;
		} else if(!increase) {
			range--;
		} else if(!increase && range == minPulseWidth) {
			runMotorsAt(0);
			break;
		}

		runMotorsAt(range);
	}
}


// =============================================================
// ADJUST PITCH USING PID/STEERING
// =============================================================

void QuaduinoFlight::adjustPitch(float pitchAngle) {
    if (pitchTuningActive) {
        // use orientation's "pitch" angle in degrees (passed as argument)
        pitchInput = pitchAngle;
        pitchSetpoint = ((rcvPitch - 180) / 9.0f);
    
        // determine appropriate PID tunings
		double temp = pitchInput - pitchSetpoint;
        if (abs(temp) > angleToSwap) {
            // far away from target value, so use aggressive tunings
            (*pitchPID).SetTunings(aggKp, aggKi, aggKd);
        } else {
            // close to target value, so use conservative tunings
            (*pitchPID).SetTunings(consKp, consKi, consKd);
        }
    
        // run PID computation
        (*pitchPID).Compute();
    }

    // assign new motor speed settings
    rearMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + rearMotorOffset + yawAdjustFR - pitchOutput;
    frontMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + frontMotorOffset + yawAdjustFR + pitchOutput;
}


// =============================================================
// ADJUST ROLL USING PID/STEERING
// =============================================================

void QuaduinoFlight::adjustRoll(float rollAngle) {
    if (rollTuningActive) {
        // use orientation's "roll" angle in degrees (passed as argument)
        rollInput = rollAngle;
        rollSetpoint = (-(rcvRoll - 180) / 9.0f);

        // determine appropriate PID tunings
		double temp = rollInput - rollSetpoint;
        if (abs(temp) > angleToSwap) {
            // far away from target value, so use aggressive tunings
            (*rollPID).SetTunings(aggKp, aggKi, aggKd);
        } else {
            // close to target value, so use conservative tunings
            (*rollPID).SetTunings(consKp, consKi, consKd);
        }

        // run PID computation
        (*rollPID).Compute();
    }
    // assign new motor speed settings
    leftMotorValue = minPulseWidth + gravityHover + altitudeSetpoint - rollOutput + leftMotorOffset + yawAdjustLR;
    rightMotorValue = minPulseWidth + gravityHover + altitudeSetpoint + rollOutput + rightMotorOffset + yawAdjustLR;
}


// =============================================================
// ADJUST YAW USING PID/STEERING (NOT IMPLEMENTED YET)
// =============================================================

void QuaduinoFlight::adjustYaw(float yawAngle) {
    if (yawTuningActive) {
        // use orientation's "yaw" angle in degrees (passed as argument)
        yawInput = yawAngle;
        (*yawPID).Compute();
    }
}


void QuaduinoFlight::setRecvPitch(uint16_t recvPitch) {
	rcvPitch = recvPitch;
}

void QuaduinoFlight::setRecvRoll(uint16_t recvRoll) {
	rcvRoll = recvRoll;
}

void QuaduinoFlight::setRecvYaw(uint16_t recvYaw) {
	rcvYaw = recvYaw;
}

void QuaduinoFlight::setRecvThrottle(uint16_t recvThrottle) {
	rcvThrottle = recvThrottle;
}

void QuaduinoFlight::setRecvActive(uint16_t recvActive) {
	rcvActive = recvActive;
}

void QuaduinoFlight::incMotorOffset(char motor, float inc) {
	switch(motor) {
	case 'f': frontMotorOffset += inc; break;
	case 'r': rightMotorOffset += inc;break;
	case 'b': rearMotorOffset += inc;break;
	case 'l': leftMotorOffset += inc;break;
	}
}

void QuaduinoFlight::setRollTuningActive(bool act) {
	rollTuningActive = act;
}

void QuaduinoFlight::setPitchTuningActive(bool act) {
	pitchTuningActive = act;
}

void QuaduinoFlight::setYawTuningActive(bool act) {
	yawTuningActive = act;
}


void QuaduinoFlight::incTuningParam(char param, float inc) {
	switch(param) {
	case 'y': consKp += inc; break;
	case 'u': consKi += inc; break;
	case 'i': consKd += inc; break;
	case 'h': consKp -= inc; break;
	case 'j': consKi -= inc; break;
	case 'k': consKd -= inc; break;
	case 'o': aggKp += inc; break;
	case 'p': aggKi += inc; break;
	case '[': aggKd += inc; break;
	case 'l': aggKp -= inc; break;
	case ';': aggKi -= inc; break;
	case '\'': aggKd -= inc; break;
	default: break;
	}
}

bool QuaduinoFlight::getRollTuningActive() {
	return rollTuningActive;
}

bool QuaduinoFlight::getPitchTuningActive() {
	return pitchTuningActive;
}

bool QuaduinoFlight::getYawTuningActive() {
	return yawTuningActive;
}

uint8_t QuaduinoFlight::getFrontMotorValue() {
	return frontMotorValue;
}

uint8_t QuaduinoFlight::getRightMotorValue() {
	return rightMotorValue;
}

uint8_t QuaduinoFlight::getRearMotorValue() {
	return rearMotorValue;
}

uint8_t QuaduinoFlight::getLeftMotorValue() {
	return leftMotorValue;
}




/*
#ifndef Flight_h
#define Flight_h

#include "Arduino.h"
#include <Serial.h>
#include <PID_v1.h>

// The Motor class contains data and operations (functions) for one motor.
// In the Motor class: "static Motor motors[4];" creates an array of 4 motors.
//
// Each motor has a "pin" on the Arduino, that outputs a PWM signal to the motor
// to control the motor speed.
// Each motor has its own "value" which is the speed (PWM) given to the motor.
//
// The four motors share ("static") information:
//    minPWM - The minimum PWM value needed to spin the motor.
//	  hoover - The approximate PWM value needed for the quad to take off.
//    active - Must be true for motors to turn -- a safety lock.
//
//
//	The Motor class has operations that can be performed on each motor:
//		Off () - Turn off the motor.
//		Set () - Set the speed (PWM) for the motor (only if Active is true).
//
// The four motors share some common operations:
//		Rev () - Turn on each of the motor to maximum speed and then turn them off. The motors 
//			will be turned on and off in order: Front, Back, Left, and Right. This command is used
//			test the motors and their wiring. If the motors do not turn on in the right order,
//			they are wired to wrong connections. If they do not sound like they are rev-ing
//			up to a high speed, there is either a loose connection, partially broken wire,
//			or a bad Mosfet. If the motors do not turn off, you have a bad Mosfet.
//		Active (status) - Turn off all the motors. If status is false, lock the motors so they 
//			cannot be turn off until Active is called with a true status.

class Motor {
  private:
  
  public:
	uint8_t pin;				// output pwm pin on Arduino
	uint8_t value;				// current motor pwm setting 	
	static uint8_t minPWM;		// minimum value to spin motor
	static uint8_t hover;		// nominal value to hover
	static bool active;			// active = false - motors stay off
	static Motor motors[4];		// four motors on quad copter
	
	Motor (uint8_t inPin);		// constructor
	void Off ();				// turn off the motor
	void Set (uint8_t pwm);		// set the motor speed
	static void Active(bool inActive); // set motors off; inActive = false, keep motors off
	void Rev();					// rev motors to max speed in order: Front, Back, Left, Right
}; //end of Motor class

// Names for the four motors
enum Motors_t {Front, Back, Left, Right};

// Names for the axis the the quad may be turn. Yes, Altitude is not really an axis.
enum Axis_t   {Yaw, Pitch, Roll, Altitude}; // 
	// Yaw      - Left-Right Direction copter's nose is facing
	// Picth    - Up-Down Direction copter's nose is facing
	// Roll     - Right-Left Wing higher than the other
	// Altitude - How high off the ground (not currently implemented)

// Names for the three PID control factors.	
enum Pid_t {Proportional, Integral, Derivative};

//
// The Flight class tracks and controls the overall fight of the quad.
// Only one Flight exists, because we only have one quad. Everything is accessed statically.
//
// attitude[4] - contains direction the quad is facing and its altitude
// desiredAttitude[4] - is direction and altitude we want the quad facing.
//      When the quad is level with the ground it will stay in one place.
//		When the quad tilts (e.g., pitches the nose up), the quad will move in that direction.
//		When the side motors turn faster or slower than the front/back motors, the quad yaws (turns).
//		So by setting an attitude other than level, the quad will move. You are now in control!
//
//	PID Pids[4] - one PID for each axis (Pitch, Yaw, Roll, and Altitude).
//
//	What is a PID? Good question!
//	PID stands for Proportional, Integral, and Derivative.
//  Ok, so what does that mean? Another good question!
//
//	PID is a very common control mechanism. In our case we are trying the control the Pitch,
//	Yaw, Roll, and Altitude of the quad; by setting the motor speeds.
//	So for example, by keeping the front motor at a constant speed and changing the speed
//	of the back motor, we control the pitch.
//	The keeping the front and back motor speeds constant, we simultaneously add (or subtract
//	speed to the side motors to adjust yaw. Then we can add speed to one side motor and 
//	simultaneously subtract the same speed from the other side motor, to adjust roll (left or
//	right wing higher than the other).
//
//	How does changing the speed of the side motors affect yaw? The front and back motors spin in
//	one direction (lets say clockwise). The two side motors spin in the other direction, lets say
//	counter clockwise. Imagine, you only had one motor, lets say it was spinning its propeller
//	clockwise. If the motor is attach to the quad and the quad is sitting on the floor, then
//	friction would keep the quad from spinning is the opposite direction of the propeller. But
//	once in the air, the quad would spin. On a regular helicopter there is a tail rotor
//	(propeller) to prevent the spinning. In the same way, if all four motors on the quad spun
//	propellers in the same direction, the quad would spin in the opposite direction. By putting
//	two motors in one direction and the two motors in the other direction, they counter each
//	others force to spin the quad. When we want the quad to turn (spin just a little), we speed
//	up (or slow down) the motors going in one direction.
//
//	Ok, so what is PID and why are we using PID? PID is the control mechanism that tells us
//	how much power to apply to each motor. If we want to front and back motor to apply the
//	same amount of lift (so the quad remains level), why not give them both the same amount
//	of power?
//		- Remember you slapped the (heavy) battery on the bottom of the quad. Probably not
//		    exactly in the middle, so one motor need to lift more to balance thing out. Also
//			the wings and motors are not all glued on exactly straight.
//		- Motors were selected to give a lot of power at an affordable price. Some motors will
//			spin faster with the same power.
//		- ...
//	So we have to set the front motor based on the throttle, then adjust/twiddle with back
//	motor to keep thing balance. With four motors to twiddle with, a person could not keep
//	up. So PID does the twiddling.
//
//	PID is algorithm, a computer formula. PID has two inputs: the desired output and the
//	current output (e.g., Pitch). PID then produces the Input (motor speed) that might
//	make the Current Output become the Desired Output. Might? PID does not get it right the
//	first time, but it keeps trying.
//
//	Why doesn't PID get it right the first time? Imagine your car is stopped. You want to know
//	how hard to push on the gas pedal to make the car go 60 miles per hour. Well first, no matter
//	how hard you press on the gas, your car is not going to be going 60 mph after one second. So
//	PID would have to give the car a lot of gas to get the car going and accelerating. AS we get
//	closer to 60, we have to ease of the gas so we don't keep on accellarating to 70, 80, ...
//
//	The Proportional part of PID, increases the Input based on how far we hare from the 
//	desired Output. If we are far away, add a lot; so we get there faster. If we are close,
//	add just a little; so we don't zoom past our goal.
//	
//	The Integral part of PID, increases the Input based on how far and how long we have been
//	away from the desired Output. Proportional is the main factor driving us toward the
//	desired Output. But if Proportional is not getting us there fast enough, Integral
//	gives the extra juice to get us there faster.
//
//	The Derivative part of PID, controls the Input based on how fast the Output is changing.
//	The Derivative smooths out changes.
//
//	The quad uses Proportion and Integral. It does not use Derivative.
//
// --------------------------------------------------------------------
//
//	Begin () - Initializes the Flight controls (turn off the motors).
//
//	SetThrottle(speed) - Set the speed of the front motor (and gives a quick 
//		adjust to the other motors).
//
//	Update(yaw, pitch, roll) - Does the PID magic and updates the motor speeds.
//
//	Light (light, value) - Turns on light to a specified brightness.
//
//	Lights (lights) - Turn on the lights in a binary pattern. Used for debug output.
//
//

class Flight {
  private:
  
  public:
	static double  attitude[4];			// current direction (and hieght) of copter
	static double  desiredAttitude[4];	// setpoint direction
	static double  previousAttitude[4];	// last pass direction
	static PID     Pids[4];				// PIDs for each Axis_t
	static double  motorPWM[4];			// motor speed CHANGE outputed by PID
	
	static unsigned long now;			// current time
	static unsigned long previousTime;	// time we last updated the values
  
	Flight();	// default constructor
	
	// user copter throttle setting
	static void SetThrottle(uint8_t inThrottle);	// user copter throttle setting
	
	// Update, new attitude each time sensors are read
	static void Update (double yaw, double pitch, double roll, double altitude = 0);
	
	// Initialize Flight (turn off motors)
	static void Begin();
	
	// Set one light's value
	static void Light (uint8_t light, uint8_t value);
	
	// Set all the lights (probably used for debugging)
	static void Lights (uint8_t lights);
	
}; // end of Flight class

//The Arduino Digital Output pins for each of the LEDs
#define WhiteLED   (9)
#define BlueLED    (44)
#define GreenLED   (45)
#define RedLED     (46)
#define ArduinoLED (13)
	// ArduinoLED on the Quad

#endif
*/
