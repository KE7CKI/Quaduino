#ifndef _myQuadEEPROM_
#define _myQuadEEPROM_

#include <Arduino.h>

#define CALSENS_DATA_VALID			0x15fc
#define CALSENS_DATA_VALID_LOW		0xfc // pattern to detect valid config - low byte
#define CALSENS_DATA_VALID_HIGH		0x15 // pattern to detect valid config - high byte

#define CALMOTOR_DATA_VALID			0xEA03
#define CALMOTOR_DATA_VALID_LOW		0x03 // pattern to detect valid config - low byte
#define CALMOTOR_DATA_VALID_HIGH	0xEA // pattern to detect valid config - high byte

typedef struct {
	short valid;                        // should contain the valid pattern if a good config
	boolean magValid;                   // true if mag data valid
	short magMinX;                      // mag min x value
	short magMaxX;                      // mag max x value
	short magMinY;                      // mag min y value
	short magMaxY;                      // mag max y value
	short magMinZ;                      // mag min z value
	short magMaxZ;                      // mag max z value 
	boolean accelValid;                 // true if accel data valid
	short accelMinX;                    // mag min x value
	short accelMaxX;                    // mag max x value
	short accelMinY;                    // mag min y value
	short accelMaxY;                    // mag max y value
	short accelMinZ;                    // mag min z value
	short accelMaxZ;                    // mag max z value
} CALSENS_DATA;

typedef struct {
	short valid;
	boolean motorValid;
	short motor1Cal;
	short motor2Cal;
	short motor3Cal;
	short motor4Cal;
} CALMOTOR_DATA;

//  EEPROMErase() erases any current data in the EEPROM

void EEPROMErase();                

//  calSensWrite() writes new data to the EEPROM

void calSensWrite(CALSENS_DATA * calData); 

//  calSensRead() reads existing data and returns true if valid else false in not.

boolean calSensRead(CALSENS_DATA * calData);

//  motorCalWrite() writes new data to the EEPROM

void motorCalWrite(CALMOTOR_DATA * calData); 

//  motorCalRead() reads existing data and returns true if valid else false in not.

boolean motorCalRead(CALMOTOR_DATA * calData);

#endif
