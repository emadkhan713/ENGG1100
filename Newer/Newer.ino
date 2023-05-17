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
const uint8_t PIN_PUMP_SPEED = 9;  // PWM pin
static uint8_t PUMP_MODE = 0;
//const  uint8_t PUMP_STEP = 39;  // approximately 15% increment of 255
//const  uint8_t PUMP_STEP = 64; // approximately 25% increment of 255
//static uint8_t PUMP_SPEED = 64; // approximately 25% of 255


//int pump = 9;
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

	//driveMotor.stop();

	panServo.attach(6);
	tiltServo.attach(7);

	panServo.write(panAngle);
	tiltServo.write(tiltAngle);
}
 
 int servo_speed = 3;

void nozzleControl() {
  // When PSB_PAD_LEFT is pressed, the nozzle moves using the servo pan and tilt. 

  //Pan Control
  if (psx.buttonPressed(PSB_PAD_LEFT) && (panAngle < 180)) {
    Serial.println("Pressing Left: Pan Left");
    panAngle += servo_speed;
    panServo.write(panAngle);

  } else if (psx.buttonPressed(PSB_PAD_RIGHT) && (panAngle > 0)) {
    Serial.println("Pressing Right: Pan Right");
    panAngle -= servo_speed;
    panServo.write(panAngle);
  }

  //Tilt Control
  if (psx.buttonPressed(PSB_PAD_DOWN) && (tiltAngle < 180)) {
    Serial.println("Pressing DOWN: Tilt Up");
    tiltAngle += servo_speed;
    tiltServo.write(tiltAngle);

  } else if (psx.buttonPressed(PSB_PAD_UP) && (tiltAngle > 110)) {
    Serial.println("Pressing UP: Tilt Down");
    tiltAngle -= servo_speed;
    tiltServo.write(tiltAngle);
  }
}

void pumpControl() {
  // 4 speed modes
  if (psx.buttonJustPressed(PSB_R1) && (PUMP_MODE < 3)) {
    //Serial.println();
    PUMP_MODE++;
  } else if (psx.buttonJustPressed(PSB_L1) && (PUMP_MODE > 0)) {
    PUMP_MODE--;
    //Serial.println(PUMP_SPEED);
  }

  // toggle pump using PWM signal
  if (psx.buttonJustPressed(PSB_SQUARE)) {
    Serial.println("Pressing Square: PUMP ON / OFF");
    pumpOn = !pumpOn;
  }

  if (pumpOn) {
	if (PUMP_MODE == 0) {
		analogWrite(PIN_PUMP_SPEED, 64); // 25% of 255
	} 
	else if (PUMP_MODE == 1) {
		analogWrite(PIN_PUMP_SPEED, 127); // 50% of 255
	} 
	else if (PUMP_MODE == 2) {
		analogWrite(PIN_PUMP_SPEED, 191); // 75% of 255
	}
	else {
		analogWrite(PIN_PUMP_SPEED, 255); // 100% of 255
	}
  } else {
    digitalWrite(PIN_PUMP_SPEED, 0);
  }
  /*
  Serial.print(F("Pump | status: "));
  Serial.print(pumpOn);
  Serial.print(F(" | mode: "));
  Serial.print(PUMP_MODE);
  Serial.print(F(" | speed: "));
  Serial.println(PUMP_SPEED);
  */

}



void motorControl() {

  if (psx.buttonPressed (PSB_R2)) {
    Serial.println("Pressing R2: Forward");
    driveMotor.forward();

  } else if (psx.buttonPressed (PSB_L2)) {
    Serial.println("Pressing L2: Backward");
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

