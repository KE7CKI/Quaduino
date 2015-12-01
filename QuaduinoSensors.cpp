#include "QuaduinoSensors.h"
#include "QuaduinoGlobal.h"

// =============================================================
// MPU-9150 INITIALIZATION
// =============================================================
QuaduinoSensors::QuaduinoSensors() {
	
}

void QuaduinoSensors::init() {
	MPU = MPU9150Lib();
        
	magHeading = 0.0;
	areSensorsReady = false;

	if(MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG)) {
		areSensorsReady = true;
                Serial.println("... sensors complete.");
	}
}

// =============================================================
// READ OUTPUT FROM MPU9150
// =============================================================
bool QuaduinoSensors::updateMPU() {
	if (MPU.read()) {
		//MPU.printAngles(MPU.m_dmpEulerPose);
		//Serial.println();
		yawPitchRoll[0] = MPU.m_dmpEulerPose[0] * RAD_TO_DEGREE; //X (pitch) rotation on the MPU9150 breakout - goes from 0 to 90 to 180 to -90 back to 0
		if(yawPitchRoll[0] < 0) {
			yawPitchRoll[0] = map(yawPitchRoll[0], -180, -0.1, 180, 359.9);
		}

		yawPitchRoll[1] = MPU.m_dmpEulerPose[1] * RAD_TO_DEGREE; //Y (roll) rotation on the MPU9150 breakout - goes from 0 to 90 to 0 to -90 back to 0
		yawPitchRoll[1] = map(yawPitchRoll[1], 90, -90, 180, 0);

		yawPitchRoll[2] = MPU.m_dmpEulerPose[2] * RAD_TO_DEGREE; //Z (Yaw) rotation on the MPU9150 breakout - goes from 0 to 90 to 180 to -90 back to 0
		if(yawPitchRoll[2] < 0) {
			yawPitchRoll[2] = map(yawPitchRoll[2], -180, -0.1, 180, 359.9);
		}

		magHeading = atan2((float)MPU.m_calMag[1], (float)MPU.m_calMag[0]) * 180.0/PI + 180;
                
                digitalWrite(WHT_LED, HIGH);
		return true;
	}
        digitalWrite(WHT_LED, LOW);
	return false;
}

// =============================================================
// REQUEST/READ TEMPERATURE/PRESSURE/ALTITUDE FROM BAROMETER
// =============================================================
bool QuaduinoSensors::updateBarometer() {
	/*
	switch (bmaState) {
        case 0:
            // request temperature
            bma.setControl(BMA085_MODE_TEMPERATURE);
            bmaMicros = micros();
            bmaState = 1;
            break;
        case 1:
            // wait appropriate time for conversion (4.5ms delay)
            if (micros() - bmaMicros < bma.getMeasureDelayMicroseconds()) break;

            // read calibrated temperature value in degrees Celsius
            bmaTemperature = bma.getTemperatureC();

            // request pressure (3x oversampling mode, high detail, 23.5ms delay)
            bma.setControl(BMA085_MODE_PRESSURE_3);
            bmaMicros = micros();
            bmaState = 2;
            break;
        case 2:
            // wait appropriate time for conversion (23.5ms delay)
            if (micros() - bmaMicros < bma.getMeasureDelayMicroseconds()) break;

            // read calibrated pressure value in Pascals (Pa)
            bmaPressure = bma.getPressure();

            // calculate absolute altitude in meters based on known pressure
            // (may pass a second "sea level pressure" parameter here,
            // otherwise uses the standard value of 101325 Pa)
            bmaAltitudeRaw = bma.getAltitude(bmaPressure);
            bmaState = 0;

            // filter/smooth altitude with Gauss-Siedel method
            // (constant closer to 0 for smooth/slow, closer to 1 for rough/fast)
            bmaAltitude0 = bmaAltitude;
            bmaAltitude = bmaAltitude0 + (0.2f * (bmaAltitudeRaw - bmaAltitude0));

            // read 5x to settle and then assign as "initial" no-movement altitude offset
            if (initialAltitudeSet < 50) {
                initialAltitudeSet++;
            } else if (initialAltitudeSet == 50) {
                initialAltitude = bmaAltitude;
                initialAltitudeSet++;
            }

            // calculate estimated altitude offset by estimated zero point
            currentaltitude = bmaAltitude - initialAltitude;

            break;
	}
	*/
	return false;
}

// =============================================================
// READ BATTERY LEVEL
// =============================================================
bool QuaduinoSensors::updateBattery() {
    // battery is connected to Arduino analog pin 0 through a voltage divider
    // fully charged should be about 705, charge required about 400

    // 710 = 8.38v, fully charged
    // 505 = 5.76v, abnormally dead
    // 380 = 4.33v, USB only w/o battery

    // change of 125 = change of 1.43
    // volts = measurement * 0.01144
    // change of 330 = change of 4.05
    // volts = measurement * 0.01227
    // change of 205 = change of 2.62
    // volts = measurement * 0.0128

    // read and scale 565-710 usable range to 0-100
    batteryRaw = (analogRead(0) - 565) * 100 / 145;
    battery0 = battery;
    battery = battery0 + (0.1f * (batteryRaw - battery0));
    if (battery < 0) battery = 0;
    else if (battery > 255) battery = 255;
}

float QuaduinoSensors::getAltitude() {
	return currentAltitude;
}

float QuaduinoSensors::getMagneticHeading() {
	return magHeading;
}

int16_t QuaduinoSensors::getBattery() {
	return battery;
}

float QuaduinoSensors::getYaw() {
	return yawPitchRoll[2];
}

float QuaduinoSensors::getPitch() {
	return  yawPitchRoll[0];
}

float QuaduinoSensors::getRoll() {
	return yawPitchRoll[1];
}
