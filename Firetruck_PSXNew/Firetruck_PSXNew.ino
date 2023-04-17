// jump to line 122 for relevant definitions
#include <DigitalIO.h>
#include <PsxControllerBitBang.h>
#include <L298NX2.h>
#include <avr/pgmspace.h>

typedef const __FlashStringHelper * FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *> (s)

const char buttonSelectName[] PROGMEM = "Select";
const char buttonL3Name[] PROGMEM = "L3";
const char buttonR3Name[] PROGMEM = "R3";
const char buttonStartName[] PROGMEM = "Start";
const char buttonUpName[] PROGMEM = "Up";
const char buttonRightName[] PROGMEM = "Right";
const char buttonDownName[] PROGMEM = "Down";
const char buttonLeftName[] PROGMEM = "Left";
const char buttonL2Name[] PROGMEM = "L2";
const char buttonR2Name[] PROGMEM = "R2";
const char buttonL1Name[] PROGMEM = "L1";
const char buttonR1Name[] PROGMEM = "R1";
const char buttonTriangleName[] PROGMEM = "Triangle";
const char buttonCircleName[] PROGMEM = "Circle";
const char buttonCrossName[] PROGMEM = "Cross";
const char buttonSquareName[] PROGMEM = "Square";

const char* const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {  // PSX_BUTTONS_NO=16, as defined in PsxNewLib.h
	buttonSelectName,
	buttonL3Name,
	buttonR3Name,
	buttonStartName,
	buttonUpName,
	buttonRightName,
	buttonDownName,
	buttonLeftName,
	buttonL2Name,
	buttonR2Name,
	buttonL1Name,
	buttonR1Name,
	buttonTriangleName,
	buttonCircleName,
	buttonCrossName,
	buttonSquareName
};

byte psxButtonToIndex (PsxButtons psxButtons) {
	byte i;

	for (i = 0; i < PSX_BUTTONS_NO; ++i) {
		if (psxButtons & 0x01) {
			break;
		}

		psxButtons >>= 1U;
	}

	return i;
}

FlashStr getButtonName (PsxButtons psxButton) {
	FlashStr ret = F("");
	
	byte b = psxButtonToIndex (psxButton);
	if (b < PSX_BUTTONS_NO) {
		PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(psxButtonNames[b])));
		ret = PSTR_TO_F (bName);
	}

	return ret;
}

void dumpButtons (PsxButtons psxButtons) {
	static PsxButtons lastB = 0;

	if (psxButtons != lastB) {
		lastB = psxButtons;     // Save it before we alter it
		
		Serial.print (F("Pressed: "));

		for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
			byte b = psxButtonToIndex (psxButtons);
			if (b < PSX_BUTTONS_NO) {
				PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(psxButtonNames[b])));
				Serial.print (PSTR_TO_F (bName));
			}

			psxButtons &= ~(1 << b);

			if (psxButtons != 0) {
				Serial.print (F(", "));
			}
		}

		Serial.println ();
	}
}

void dumpAnalog (const char *str, const byte x, const byte y) {
	Serial.print (str);
	Serial.print (F(" analog: x = "));
	Serial.print (x);
	Serial.print (F(", y = "));
	Serial.println (y);
}

const char ctrlTypeUnknown[] PROGMEM = "Unknown";
const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

const char* const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
	ctrlTypeUnknown,
	ctrlTypeDualShock,
	ctrlTypeDsWireless,
	ctrlTypeGuitHero,
	ctrlTypeOutOfBounds
};



// PWM pins are 3, 5, 6, 9, 10, 11
// need 4 PWM pins (2 motor, 1 pump, )

// define pins for PS2 controller
const byte PIN_PS2_ATT = 4;
const byte PIN_PS2_CMD = 3;
const byte PIN_PS2_DAT = 2;
const byte PIN_PS2_CLK = 5;

const byte PIN_BUTTONPRESS = A0;
const byte PIN_HAVECONTROLLER = A1;

// define pins for motor driver
const uint8_t PIN_L298_ENA;
const uint8_t PIN_L298_IN1;
const uint8_t PIN_L298_IN2;

const uint8_t PIN_L298_ENB;
const uint8_t PIN_L298_IN3;
const uint8_t PIN_L298_IN4;

// initialise PS2 controller
PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;
boolean haveController = false;

// initialise motor driver
L298NX2 motors(PIN_L298_ENA,
               PIN_L298_IN1,
               PIN_L298_IN2,
               PIN_L298_ENB,
               PIN_L298_IN3,
               PIN_L298_IN4);

void setup () {
	fastPinMode (PIN_BUTTONPRESS, OUTPUT);
	fastPinMode (PIN_HAVECONTROLLER, OUTPUT);
	pinMode (13, OUTPUT); // emulating L298N using UQ Innovate PCB (PWM effectively)
	digitalWrite(13, LOW);  //start with no movement
	psx.begin();

	delay (300);

	Serial.begin (115200);
	Serial.println (F("Ready!"));
}
 
void loop () {
	
  psx.read();

  if (psx.buttonPressed(PSB_R2)) {
    motors.forward();
    digitalWrite(13, HIGH); // allow flow to motors
    Serial.println(F("Moving FORWARDS"));
  } else if (psx.buttonPressed(PSB_L2)) {
    motors.backward();
    digitalWrite(13, LOW);  // stop flow to motors
    Serial.println(F("Moving BACKWARDS"));
  } else {
    motors.stop();
    digitalWrite(13, LOW);
    Serial.println(F("Not moving"));
  }

  
  // uncomment to debug controller
  delay (1000 / 30); // 60Hz polling rate

}

void PSXInfoLoop () {
  static byte slx, sly, srx, sry;
	
	fastDigitalWrite (PIN_HAVECONTROLLER, haveController);
	
	if (!haveController) {  // if the controller isn't connected
		if (psx.begin ()) {   // if the controller can be initialised
			Serial.println (F("Controller found!"));
			delay (300);
			if (!psx.enterConfigMode ()) {
				Serial.println (F("Cannot enter config mode"));
			} else {
				PsxControllerType ctype = psx.getControllerType ();
				PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte> (ctype) : PSCTRL_MAX])));
				Serial.print (F("Controller Type is: "));
				Serial.println (PSTR_TO_F (cname));

				if (!psx.enableAnalogSticks ()) {
					Serial.println (F("Cannot enable analog sticks"));
				}
				
				//~ if (!psx.setAnalogMode (false)) {
				  //~ Serial.println (F("Cannot disable analog mode"));
				//~ }
				//~ delay (10);
				
				if (!psx.enableAnalogButtons ()) {
					Serial.println (F("Cannot enable analog buttons"));
				}
				
				if (!psx.exitConfigMode ()) {
					Serial.println (F("Cannot exit config mode"));
				}
			}
			
			haveController = true;
		}
	} else {                // the controller is connected
		if (!psx.read ()) {   // the controller can't be read
			Serial.println (F("Controller lost :("));
			haveController = false;
		} else {              // the controller is responding
			fastDigitalWrite (PIN_BUTTONPRESS, !!psx.getButtonWord ());
			dumpButtons (psx.getButtonWord ());

			byte lx, ly;
			psx.getLeftAnalog (lx, ly);
			if (lx != slx || ly != sly) {
				dumpAnalog ("Left", lx, ly);
				slx = lx;
				sly = ly;
			}

			byte rx, ry;
			psx.getRightAnalog (rx, ry);
			if (rx != srx || ry != sry) {
				dumpAnalog ("Right", rx, ry);
				srx = rx;
				sry = ry;
			}
		}
	}
}