#include <L298NX2.h>
#include <Servo.h>
#include <PsxControllerBitBang.h>
#include <DigitalIO.h>
#include <avr/pgmspace.h>

typedef const __FlashStringHelper * FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *> (s)

// These can be changed freely when using the bitbanged protocol
// Pin in order from left to right (when looking at the connector)
const byte PIN_PS2_DAT = 2;
const byte PIN_PS2_CMD = 3;
// ground to GND PIN
// power to 3.3V PIN
const byte PIN_PS2_ATT = 4;
const byte PIN_PS2_CLK = 5;


// Drive Functionality
int IN1 = 10;
int IN2 = 11;
int IN3 = 12;
int IN4 = 13;

L298NX2 driveMotor(IN1, IN2, IN3, IN4);


// Fluid Functionality
const uint8_t PIN_PUMP_SPEED = 8;  // PWM pin
static uint8_t PUMP_MODE = 0;
const  uint8_t PUMP_STEP = 39;  // approximately 15% increment of 255
static uint8_t PUMP_SPEED = PUMP_STEP * 2;

//int pump = 8;
boolean pumpOn = false;

Servo panServo; //pan servo is pin 6
Servo tiltServo; //tilt servo is pin 7

int panAngle = 90; // Facing straight
int tiltAngle = 150; // Facing 60 degrees up


//PS2 STUFF
const byte PIN_BUTTONPRESS = A0;
const byte PIN_HAVECONTROLLER = A1;

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

const char* const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {
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


PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK> psx;

boolean haveController = false;
 
void setup () {
	fastPinMode (PIN_BUTTONPRESS, OUTPUT);
	fastPinMode (PIN_HAVECONTROLLER, OUTPUT);

  pinMode(PIN_PUMP_SPEED, OUTPUT);
	
	delay (300);

	Serial.begin (115200);
	Serial.println (F("Ready!"));
	//psx.begin();

	driveMotor.stop();

	panServo.attach(6);
	tiltServo.attach(7);

	panServo.write(panAngle);
	tiltServo.write(tiltAngle);
}
 
 int speed = 3;

void nozzleControl() {
  // When PSB_PAD_LEFT is pressed, the nozzle moves using the servo pan and tilt. 

  //Pan Control
  if (psx.buttonPressed(PSB_PAD_LEFT) && (panAngle < 180)) {
    Serial.println("Pan Left");
    panAngle += speed;
    panServo.write(panAngle);

  } else if (psx.buttonPressed(PSB_PAD_RIGHT) && (panAngle > 0)) {
    Serial.println("Pan Right");
    panAngle -= speed;
    panServo.write(panAngle);
  }

  //Tilt Control
  if (psx.buttonPressed(PSB_PAD_UP) && (tiltAngle < 180)) {
    Serial.println("Tilt Up");
    tiltAngle += speed;
    tiltServo.write(tiltAngle);

  } else if (psx.buttonPressed(PSB_PAD_DOWN) && (tiltAngle > 110)) {
    Serial.println("Tilt Down");
    tiltAngle -= speed;
    tiltServo.write(tiltAngle);
  }
}

void pumpControl() {
  // 5 speed modes
  if (psx.buttonJustPressed(PSB_R1) && (PUMP_MODE < 4)) {
    //Serial.println();
    PUMP_MODE++;
    PUMP_SPEED += PUMP_STEP;
  } else if (psx.buttonJustPressed(PSB_L1) && (PUMP_MODE > 0)) {
    PUMP_MODE--;
    //Serial.println(PUMP_SPEED);
    PUMP_SPEED -= PUMP_STEP;
  } else {
	;
  }

  // toggle pump using PWM signal
  if (psx.buttonJustPressed(PSB_SQUARE)) {
    Serial.println("Pressing Square, PUMP ON OR OFF");
    pumpOn = !pumpOn;
  }

  if (pumpOn == 1) {
    digitalWrite(PIN_PUMP_SPEED, PUMP_SPEED);
  } else {
    digitalWrite(PIN_PUMP_SPEED, 0);
  }
  Serial.print(F("Pump | status: "));
  Serial.print(pumpOn);
  Serial.print(F(" | mode: "));
  Serial.print(PUMP_MODE);
  Serial.print(F(" | speed: "));
  Serial.println(PUMP_SPEED);

}



void motorControl() {

  if (psx.buttonPressed (PSB_R2)) {
    Serial.println("Pressing R2");
    driveMotor.forward();

  } else if (psx.buttonPressed (PSB_L2)) {
    Serial.println("Pressing L2");
    driveMotor.backward();

  } else {
    driveMotor.stop();
  }
}

void testing () {
	
	fastDigitalWrite (PIN_HAVECONTROLLER, haveController);
	
	if (!haveController) {
		if (psx.begin ()) {
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
	} else {
		if (!psx.read ()) {
			Serial.println (F("Controller lost :("));
			haveController = false;
		} else {
        motorControl();
        nozzleControl();  
        pumpControl();
    }
	}
}


void loop() {
  testing();
  //motorControl();
  //nozzleControl();  
  //pumpControl();

  delay(1000 / 60); // 60 FPS
}

