#include <EEPROM.h>

#include "QuaduinoEEPROM.h"

void EEPROMErase()
{
    EEPROM.write(0, 0);                            // just destroy the valid byte
}


void calSensWrite(CALSENS_DATA *calSensData){
  byte *ptr = (byte *)calSensData;
  byte length = sizeof(CALSENS_DATA);

  calSensData->valid = CALSENS_DATA_VALID;
  
  for (byte i = 0; i < length; i++)
    EEPROM.write(i, *ptr++);
}

boolean calSensRead(CALSENS_DATA *calSensData){
  byte *ptr = (byte *)calSensData;
  byte length = sizeof(CALSENS_DATA);

  calSensData->magValid = false;
  calSensData->accelValid = false;

  if ((EEPROM.read(0) != CALSENS_DATA_VALID_LOW) ||   
      (EEPROM.read(1) != CALSENS_DATA_VALID_HIGH))
    return false;                                  // invalid data
    
  for (byte i = 0; i < length; i++)
    *ptr++ = EEPROM.read(i);
  return true;  
}

void motorCalWrite(CALMOTOR_DATA * calMotorData) {
	byte *ptr = (byte *)calMotorData;
	byte length = sizeof(calMotorData);

	calMotorData->valid = CALMOTOR_DATA_VALID;

	for(byte i = sizeof(CALSENS_DATA); i < length; i++) {
		EEPROM.write(i, *ptr++);
	}
}

boolean motorCalRead(CALMOTOR_DATA * calMotorData) {
	byte *ptr = (byte *)calMotorData;
	byte length = sizeof(calMotorData);

	calMotorData->motorValid = false;

	if ((EEPROM.read(sizeof(CALSENS_DATA)) != CALMOTOR_DATA_VALID_LOW) ||   
		(EEPROM.read(sizeof(CALSENS_DATA)+1) != CALMOTOR_DATA_VALID_HIGH))
		return false;                                  // invalid data
    
	for (byte i = sizeof(CALSENS_DATA); i < length; i++)
		*ptr++ = EEPROM.read(i);
	return true;  
}
