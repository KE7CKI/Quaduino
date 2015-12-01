#include "QuaduinoCommunication.h"
#include "QuaduinoSensors.h"
#include "QuaduinoFlight.h"
#include "QuaduinoGlobal.h"


QuaduinoCommunication::QuaduinoCommunication() {
  
}

void QuaduinoCommunication::init() {
	Serial.begin(57600);
	Serial.flush();
	Serial.println("... serial 0 complete.");
	Serial2.begin(38400);
	Serial2.flush();
	Serial.println("... serial 1 complete.");

	i = 0;
	count = 0;
	hasPacket = false;
}

void QuaduinoCommunication::parseSerialInput(uint8_t whichSerial) {
	 if(Serial.available() > 0 && whichSerial == 0) { // If there is data on serial0 and that is the serial line we're trying to listen on...
		 dataIn[i]= Serial.read();
	
		if(i < 31) {
			i++;
		} else {
			hasPacket = true;
			i = 0;
		}
	 } else if(Serial2.available() > 0 && whichSerial == 1) { // If there is data on serial1 and that is the serial we're trying to listen on..
		 dataIn[i]= Serial2.read();
	
		if(i < 31) {
			i++;
		} else {
			hasPacket = true;
			i = 0;
		}
	 }


	 if(hasPacket) {
		hasPacket = false;
		switch(dataIn[0]) {
			case 0: // Flight command packet
			flightController.setRecvRoll(dataIn[1]);
			flightController.setRecvPitch(dataIn[2]);
			flightController.setRecvThrottle(dataIn[3]);
			flightController.setRecvYaw(dataIn[4]);
			flightController.setRecvActive(dataIn[5]);
			break;
		case 1: // Motor calibration packet
			switch(dataIn[1]) {
			case 1: flightController.incMotorOffset('f', 1); break;
			case 2: flightController.incMotorOffset('f', -1); break;
			case 3: flightController.incMotorOffset('r', 1); break;
			case 4: flightController.incMotorOffset('r', -1); break;
			case 5: flightController.incMotorOffset('b', 1); break;
			case 6: flightController.incMotorOffset('b', -1); break;
			case 7: flightController.incMotorOffset('l', 1); break;
			case 8: flightController.incMotorOffset('l', -1); break;
			}
			break;
		case 2: // PID enable/calibration packet
			switch(dataIn[1]) {
			case ',': 
				//flightController.setYawTuningActive(!flightController.getYawTuningActive());
				//Serial.print("k,yaw,");
				//Serial.println(yaw_tuning_active ? 1 : 0);
				break;
			case '.': 
				flightController.setPitchTuningActive(!flightController.getPitchTuningActive());
				//Serial.print("k,pitch,");
				//Serial.println(pitch_tuning_active ? 1 : 0);
				break;
			case '/': 
				flightController.setRollTuningActive(!flightController.getRollTuningActive());
				//Serial.print("k,roll,");
				//Serial.println(roll_tuning_active ? 1 : 0);
				break;
			case 'y': flightController.incTuningParam('y', 0.01); break;
			case 'u': flightController.incTuningParam('u', 0.01); break;
			case 'i': flightController.incTuningParam('i', 0.001); break;
			case 'h': flightController.incTuningParam('h', -0.01); break;
			case 'j': flightController.incTuningParam('j', -0.01); break;
			case 'k': flightController.incTuningParam('k', -0.001); break;
			case 'o': flightController.incTuningParam('o', 0.01); break;
			case 'p': flightController.incTuningParam('p', 0.01); break;
			case '[': flightController.incTuningParam('[', 0.001); break;
			case 'l': flightController.incTuningParam('l', -0.01); break;
			case ';': flightController.incTuningParam(';', -0.01); break;
			case '\'': flightController.incTuningParam('\'', -0.001); break;
			}
			break;
		case 3: // EEPROM control (not enabled)
			switch(dataIn[1]) {
			case 'c':
				break;
			case 'v':
				break;
			default: break;
			}
			break;
		}
	}
}

void QuaduinoCommunication::serialSendQuadStatus(QuaduinoSensors &thisSensorPackage, QuaduinoFlight &thisFlightController, int8_t whichSerial) {
  dataOut[0] = 'g';
  dataOut[1] = map(thisSensorPackage.getPitch(), 0, 360, 0, 255); //max 360, min 0
  dataOut[2] = thisSensorPackage.getRoll(); //max 180 min 0
  dataOut[3] = map(thisSensorPackage.getYaw(), 0, 360, 0, 255); //max 360, min 0
  dataOut[4] = map(thisFlightController.getFrontMotorValue(), 1100, 1600, 0, 255); //max 1600, min 1100
  dataOut[5] = map(thisFlightController.getRightMotorValue(), 1100, 1600, 0, 255);
  dataOut[6] = map(thisFlightController.getRearMotorValue(), 1100, 1600, 0, 255);
  dataOut[7] = map(thisFlightController.getLeftMotorValue(), 1100, 1600, 0, 255);
  dataOut[8] = map(thisSensorPackage.getMagneticHeading(), 0, 360, 0, 255);
  dataOut[9] = thisSensorPackage.getBattery();
  dataOut[10] = 0; //first byte of altitude
  dataOut[11] = 0; //second byte of altitude
  dataOut[31] = 10;

  switch(whichSerial) {
  case 1: Serial2.write(dataOut, 32); break;
  case 0: Serial.write(dataOut, 32); break;// Without terminating with "break;" this continues onto the default case..
  default: Serial.write(dataOut, 32); break;
  }
}

void QuaduinoCommunication::isActive() {
	Serial.println("I am active.");
}

/*
void MyQuadCommunication::serialEvent() {
  dataIn[i]= Serial.read();
  Serial1.flush();
	
  if(i < 31) {
    //Serial.print(dataIn[i]);
    i++;
  } else {
    //Serial.println();
    hasPacket = true;
    i = 0;
  }
}

void serialEvent1() {
  dataIn[i]= Serial1.read();
  Serial1.flush();
	
  if(i < 31) {
    //Serial.print(dataIn[i]);
    i++;
  } else {
    //Serial.println();
    hasPacket = true;
    i = 0;
  }
}
*/
