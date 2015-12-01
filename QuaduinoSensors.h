#ifndef _QuaduinoSensors_
#define _QuaduinoSensors_

#include "MPU9150Lib.h"

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output
#define MPU_UPDATE_RATE  (50)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:
#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 


class QuaduinoSensors {
private:
	MPU9150Lib MPU;
	MPUVector3 yawPitchRoll; 

	// calculated altitude output (relative to initial zero value)
	float currentAltitude;
	float initialAltitude;
	uint8_t initialAltitudeSet;

	// battery level (scaled to 0-100, filtered using Gauss-Siedel)
	int16_t battery;
	int16_t battery0;
	int16_t batteryRaw;

	// magnetic heading
	float magHeading;

	// set true if all sensors available and programmed correctly
	bool areSensorsReady;
public:
	QuaduinoSensors();

	void init();

	bool updateMPU();
	bool updateBarometer();
	bool updateBattery();

	float getAltitude();
	float getMagneticHeading();
	int16_t getBattery();

	float getYaw();
	float getPitch();
	float getRoll();
};

#endif
