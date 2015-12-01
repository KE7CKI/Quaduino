#ifndef _QuaduinoCommunication_
#define _QuaduinoCommunication_

#include <Arduino.h>
#include "QuaduinoSensors.h"
#include "QuaduinoFlight.h"

// keeps track of incoming data bytes
#define PACKET_SIZE 32

class QuaduinoCommunication {
private:
	uint8_t dataIn[PACKET_SIZE];
	uint8_t dataOut[PACKET_SIZE];

	uint8_t i, count;
	boolean hasPacket;
	uint8_t packetPos;
	uint8_t packetType;
public:
	
	QuaduinoCommunication();

	// =============================================================
	// INITIAL SERIAL SETUP
	// =============================================================
	void init();
	// =============================================================
	// PARSE INCOMING SERIAL DATA
	// =============================================================

	void parseSerialInput(uint8_t whichSerial);

	/*
	void serialEvent();
	void serialEvent1();
	*/

	// =============================================================
	// SEND QUAD INFO TO CONTROL CENTER
	// =============================================================
	//void serialSendQuadStatus(uint8_t whichSerial);
        void serialSendQuadStatus(QuaduinoSensors &thisSensorPackage, QuaduinoFlight &thisFlightController, int8_t whichSerial);
	
        static void isActive();
};

#endif
