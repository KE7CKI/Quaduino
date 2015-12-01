#ifndef _myQuadPins_
#define _myQuadPins_

#include "QuaduinoSensors.h"
#include "QuaduinoCommunication.h"
#include "QuaduinoFlight.h"

// Pin Definitions
#define BLU_LED		44
#define GRN_LED		45	// Used to indicate Right side of quad and MPU DMP Upload success
#define RED_LED		46	// Used to indicate Left side of quad and MPU Initialize success
#define WHT_LED		9	

#define SYSOFF		49
#define VBAT		62

#define BAT_EN1		30
#define BAT_EN2		31
#define BAT_CHG		32

#define MOTOR4		8	// Motor 4 output
#define MOTOR3		12	// Motor 3 output
#define MOTOR2		11	// Motor 2 output
#define MOTOR1		10	// Motor 1 output

#define RADIO_ASSOC	33	// Used to indicate the radio has associated itself with a dongle
#define RADIO_RSSI	34	// Radio Signal Strength Indicator

#define MPU_INT		18
#define ALT_INT		19

/* The following pins are defined by their respective classes. In general, you don't
 * need to define their pins. They are only placed here as a representation of
 * which pins are unavailable.
 *
#define SDA			20
#define SCL			21
// SDA/SDL are I2C pins and defined in I2Cdev
#define MISO		50
#define MOSI		51
#define SCK			52
// MOSI/MISO/SCK are SPI pins and are defined in SPI.h
#define Radio_TX	16
#define Radio_RX	17
// Radio_TX/Radio_RX are Serial
 *
 */

static QuaduinoSensors sensorPackage;
static QuaduinoCommunication commDevice;
static QuaduinoFlight flightController;

#endif
