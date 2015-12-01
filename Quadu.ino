/******************************************************************************

myQuad - Intel Ultimate Engineering Experience
         Firmware revision: 20130522-2320

BASIC "+" QUADCOPTER DESIGN
===========================

1. Rigid frame with four motors equidistant from center at N/S/E/W positions.
2. Electronics and power source (battery) mounted firmly in center of frame.
3. Sensors built-in to determine orientation, heading, velocity, and position.
4. Microcontroller software controls individual motor speed to hover/fly quad.

ARDUINO MYQUAD IMPLEMENTATION
=============================

Power-on initial setup:
  1. Initialize info/control link. (Serial/nRF24LU1+)
  2. Read stored calibrations settings. (EEPROM)
  3. Initialize motion sensors. (MPU-9150, AK8975)
  4. Initialize motor control. (Servo, PID)

Main full-time run loop:
  1. Read and process commands from info/control link.
  2. Read orientation/motion/position data from sensors.
  3. If motors enabled, adjust motor speed:
      a. Assign appropriate PID algorithm tuning.
      b. Compute PID algorithm updates.
      c. Set new motor speed values.
  4. Write status to info/control link if status changed.

 *******************************************************************************/

// main Arduino library
//#include <Arduino.h>	//included in myQuadEEPROM.h

// EEPROM library
#include <EEPROM.h>	//included in myQuadEEPROM.cpp

// sensor library files
#include <Wire.h>		//included in I2CDev.h
#include "I2Cdev.h"
//#include "MPU9150Lib.h"	//included in myQuadSensors.h
//#include "dmpKey.h"	//included in inv_mpu_dmp_motion_driver.cpp
//#include "dmpmap.h"	//included in inv_mpu_dmp_motion_driver.cpp
//#include "inv_mpu.h"	//included in MPU9150Lib.cpp
//#include "inv_mpu_dmp_motion_driver.h"	//included in MPU9150Lib.cpp

// motor control library files
#include "PID_v1.h"

//#define DEBUG_HeaderSketch
//#define DEBUG_MotorSketch
//#define DEBUG_DataSketch
//#define DEBUG_RadioDataSketch
#define DEBUG_FullFunctionSketch

// internal code organization for simplicity
#include "QuaduinoGlobal.h"
#include "QuaduinoEEPROM.h"	//included in MPU9150Lib.h
#include "QuaduinoSensors.h"
#include "QuaduinoFlight.h"
#include "QuaduinoCommunication.h"

void setupPins();

#ifdef DEBUG_HeaderSketch
int LEDCount;
#endif

// =============================================================
// INITIAL SETUP
// =============================================================
void setup()
{
    Serial.write("Beginning setup");
	setupPins();

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin(); 

	commDevice.init();
	sensorPackage.init();
	flightController.init();

#ifdef DEBUG_HeaderSketch
	LEDCount = 0;
#endif

     Serial.println("Setup complete.");
}

void setupPins() {
	pinMode(BLU_LED, OUTPUT);
	pinMode(GRN_LED, OUTPUT);
	pinMode(RED_LED, OUTPUT);
	pinMode(WHT_LED, OUTPUT);

	pinMode(SYSOFF, OUTPUT);
	pinMode(VBAT, INPUT);

	pinMode(BAT_EN1, OUTPUT);
	pinMode(BAT_EN2, OUTPUT);
	pinMode(BAT_CHG, INPUT);
	
	pinMode(MOTOR4, OUTPUT);
	pinMode(MOTOR3, OUTPUT);
	pinMode(MOTOR2, OUTPUT);
	pinMode(MOTOR1, OUTPUT);

	pinMode(RADIO_ASSOC, OUTPUT);
	pinMode(RADIO_RSSI, OUTPUT);

	pinMode(MPU_INT, INPUT);
	pinMode(ALT_INT, INPUT);
}

// =============================================================
// MAIN PROGRAM LOOP
// =============================================================

uint8_t frameCount;
uint32_t lastMil;
uint32_t lastMic;
uint32_t benchMic;

void loop()
{
  #ifdef DEBUG_HeaderSketch
	//Header Sketch:
	for(uint8_t i = 0; i < 7; i++) { 
		digitalWrite(i, LOW);
		analogWrite(i, 0);
	}

	switch(LEDCount) {
	case 0: digitalWrite(0, HIGH); analogWrite(A0, 1023); break;
	case 1: digitalWrite(1, HIGH); analogWrite(A1, 1023); break;
	case 2: digitalWrite(2, HIGH); analogWrite(A2, 1023); break;
	case 3: digitalWrite(3, HIGH); analogWrite(A3, 1023); break;
	case 4: digitalWrite(4, HIGH); analogWrite(A4, 1023); break;
	case 5: digitalWrite(5, HIGH); analogWrite(A5, 1023); break;
	case 6: digitalWrite(6, HIGH); analogWrite(A6, 1023); break;
	case 7: digitalWrite(7, HIGH); analogWrite(A7, 1023); break;
	default: break;
	}

	LEDCount++;
	delay(100);
#elif defined DEBUG_MotorSketch
	//Motor Sketch:
        
        for(uint8_t val = 0; val < 255; val++) {
          flightController.writeMotor(MOTOR1, val);
          delay(5);
        }
        for(uint8_t val = 255; val > 0; val--) {
          flightController.writeMotor(MOTOR1, val);
          delay(5);
        }	
#elif defined DEBUG_DataSketch
	//Data Sketch:
	
	sensorPackage.updateMPU();
	sensorPackage.updateBarometer();
	sensorPackage.updateBattery();

	if(frameCount++ % 3 == 0) {
		commDevice.serialSendQuadStatus(sensorPackage, flightController, 0);
	}

	commDevice.parseSerialInput(0);

#elif defined DEBUG_RadioDataSketch
	//Radio Data Sketch:

	if (frameCount++ % 3 == 0) {
              commDevice.serialSendQuadStatus(sensorPackage, flightController, 1);
        }
	
	commDevice.parseSerialInput(1);
	
#elif defined DEBUG_FullFunctionSketch
	//Full-function sketch:

    // read and parse any available serial data
    commDevice.parseSerialInput(1);
	
	sensorPackage.updateMPU();
	sensorPackage.updateBarometer();
	sensorPackage.updateBattery();

    if (frameCount++ % 3 == 0) {
		commDevice.serialSendQuadStatus(sensorPackage, flightController, 1);
    }

    // if MPU programming failed, don't try to do anything,
    // because it will result in absolutely no ability to fly
    /*
	if (sensors_ready) {
        // read orientation from MPU/DMP
        
		//if (dmp_data_ready()) {
            // BENCHMARK TIME FOR THIS LOOP, 20120626-2141: ~3500 microseconds
            //benchMic = micros();
            //update_mpu();
            //Serial.println(micros() - benchMic);
        //}

        // read temperature/pressure/altitude from barometer
        // (internally reads new data in correct order using appropriate
        // non-blocking delay between commands/reads, as BMA085 requires)
        //update_barometer();

        // run this every 10ms, a.k.a. 100Hz
        if (micros() - lastMic >= 10004) {
            // BENCHMARK TIME FOR THIS LOOP, 20120626-2141: ~5500 microseconds
            // (benchmark includes serial_send_motor_speeds() call)
            //benchMic = micros();
            lastMic = micros();
            
            // update battery level
            update_battery();

            // adjust motors if active
            if (motors_active) {
                // feed PID balancing algorithms with newest orientation data
                // yaw MUST be run before pitch/roll due to motor speed adjustments
                adjust_yaw(yawPitchRoll[0] * 180.0f/PI);
                adjust_pitch(yawPitchRoll[1] * 180.0f/PI);
                adjust_roll(yawPitchRoll[2] * 180.0f/PI);

                // update motor speeds to new adjusted values
                rear_motor.writeMicroseconds(rear_motor_value);
                front_motor.writeMicroseconds(front_motor_value);
                left_motor.writeMicroseconds(left_motor_value);
                right_motor.writeMicroseconds(right_motor_value);
            }

             // send orientation/altitude/motor/battery status
           // (sends every other frame, or ~50Hz)
            //Serial.println(micros() - benchMic);
        }
		
    }
    */
#endif
}
