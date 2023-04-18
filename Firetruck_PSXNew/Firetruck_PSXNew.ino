// jump to line 125 for relevant definitions
#include <DigitalIO.h>
#include <Servo.h>
#include <PsxControllerBitBang.h>
#include <avr/pgmspace.h>

typedef const __FlashStringHelper* FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper*>(s)

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

byte psxButtonToIndex(PsxButtons psxButtons) {
  byte i;

  for (i = 0; i < PSX_BUTTONS_NO; ++i) {
    if (psxButtons & 0x01) {
      break;
    }

    psxButtons >>= 1U;
  }

  return i;
}

FlashStr getButtonName(PsxButtons psxButton) {
  FlashStr ret = F("");

  byte b = psxButtonToIndex(psxButton);
  if (b < PSX_BUTTONS_NO) {
    PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
    ret = PSTR_TO_F(bName);
  }

  return ret;
}

void dumpButtons(PsxButtons psxButtons) {
  static PsxButtons lastB = 0;

  if (psxButtons != lastB) {
    lastB = psxButtons;  // Save it before we alter it

    Serial.print(F("Pressed: "));

    for (byte i = 0; i < PSX_BUTTONS_NO; ++i) {
      byte b = psxButtonToIndex(psxButtons);
      if (b < PSX_BUTTONS_NO) {
        PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(psxButtonNames[b])));
        Serial.print(PSTR_TO_F(bName));
      }

      psxButtons &= ~(1 << b);

      if (psxButtons != 0) {
        Serial.print(F(", "));
      }
    }

    Serial.println();
  }
}

void dumpAnalog(const char* str, const byte x, const byte y) {
  Serial.print(str);
  Serial.print(F(" analog: x = "));
  Serial.print(x);
  Serial.print(F(", y = "));
  Serial.println(y);
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

const byte PIN_BUTTONPRESS = A0;
const byte PIN_HAVECONTROLLER = A1;



// PWM pins are 3, 5, 6, 9, 10, 11
// needed: 1 motor, 1 pump

const byte PIN_PS2_ATT = 4;
const byte PIN_PS2_CMD = 3;
const byte PIN_PS2_DAT = 2;
const byte PIN_PS2_CLK = 5;

PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK>
  psx;
boolean haveController = false;

const uint8_t PIN_MOTOR_DIRECTION = 12;
const uint8_t PIN_MOTOR_SPEED = 10;

const uint8_t PIN_PUMP_ENABLE = 8;
const uint8_t PIN_PUMP_SPEED = 9;  // PWM pin

const uint8_t PIN_SERVO_H = A2;
const uint8_t PIN_SERVO_V = A3;

const uint8_t PUMP_STEP = 39;      // approximately 15% increment of 255
static uint8_t PWM_PUMP_MODE = 0;
static uint8_t PWM_PUMP_SPEED = 39*2;

Servo horizontal;
Servo vertical;

static uint8_t ANGLE_SERVO_H = 90;
static uint8_t ANGLE_SERVO_V = 135;

void setup() {
  fastPinMode(PIN_BUTTONPRESS, OUTPUT);
  fastPinMode(PIN_HAVECONTROLLER, OUTPUT);

  pinMode(PIN_MOTOR_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_SPEED, OUTPUT);
  pinMode(PIN_PUMP_MODE, OUTPUT);
  pinMode(PIN_PUMP_SPEED, OUTPUT);
  
  digitalWrite(PIN_MOTOR_DIRECTION, LOW);
  digitalWrite(PIN_MOTOR_SPEED, 0);
  digitalWrite(PIN_PUMP_MODE, PWM_PUMP_MODE);
  analogWrite(PIN_PUMP_SPEED, PWM_PUMP_SPEED);

  horizontal.attach(PIN_SERVO_H);
  vertical.attach(PIN_SERVO_V);

  psx.begin();

  delay(300);

  Serial.begin(115200);
  Serial.println(F("Ready!"));
}

void loop() {
  psx.read();
  MotorControl();
  PumpSpeed();
  NozzleControl();
  //PsxInfoLoop();
  delay(1000 / 60);  // 60Hz polling rate
}

void MotorControl() {
  if (psx.buttonPressed(PSB_R2)) {
    digitalWrite(PIN_MOTOR_DIRECTION, LOW);
    digitalWrite(PIN_MOTOR_SPEED, 255);
  } else if (psx.buttonPressed(PSB_L2)) {
    digitalWrite(PIN_MOTOR_DIRECTION, HIGH);
    digitalWrite(PIN_MOTOR_SPEED, 255);
  } else {
    digitalWrite(PIN_MOTOR_SPEED, 0);
  }
}

void PumpSpeed() {
  // 5 speed modes
  if (psx.buttonJustPressed(PSB_R1) && (PWM_PUMP_MODE < 4)) {
    PWM_PUMP_MODE++;
    PWM_PUMP_SPEED += PUMP_STEP;
  }
  if (psx.buttonJustPressed(PSB_L1) && (PWM_PUMP_MODE > 0)) {
    PWM_PUMP_MODE--;
    PWM_PUMP_SPEED -= PUMP_STEP;
  }

  /* Serial.print(F("Pump info | mode: "));
  Serial.print(PWM_PUMP_MODE);
  Serial.print(F(" | speed: "));
  Serial.println(PWM_PUMP_SPEED); */
}

void NozzleControl() {
  // horizontal servo
  if (psx.buttonPressed(PSB_PAD_LEFT) && (ANGLE_SERVO_H > 0)) {
    ANGLE_SERVO_H--;
    horizontal.write(ANGLE_SERVO_H)
  }
  if (psx.buttonPressed(PSB_PAD_RIGHT) && (ANGLE_SERVO_H < 180)) {
    ANGLE_SERVO_H++;
    horizontal.write(ANGLE_SERVO_H)
  }

  // vertical servo
  if (psx.buttonPressed(PSB_PAD_UP) && (ANGLE_SERVO_V > 90)) {
    ANGLE_SERVO_V--;
    vertical.write(ANGLE_SERVO_V)
  }
  if (psx.buttonPressed(PSB_PAD_DOWN) && (ANGLE_SERVO_V < 180)) {
    ANGLE_SERVO_V++;
    vertical.write(ANGLE_SERVO_V)
  }

  /* Serial.print(F("Servo info | H: "));
  Serial.print(ANGLE_SERVO_H);
  Serial.print(F(" | V: "));
  Serial.println(ANGLE_SERVO_V); */
}

void PSXInfoLoop() {
  static byte slx, sly, srx, sry;

  fastDigitalWrite(PIN_HAVECONTROLLER, haveController);

  if (!haveController) {  // if the controller isn't connected
    if (psx.begin()) {    // if the controller can be initialised
      Serial.println(F("Controller found!"));
      delay(300);
      if (!psx.enterConfigMode()) {
        Serial.println(F("Cannot enter config mode"));
      } else {
        PsxControllerType ctype = psx.getControllerType();
        PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P>(pgm_read_ptr(&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte>(ctype) : PSCTRL_MAX])));
        Serial.print(F("Controller Type is: "));
        Serial.println(PSTR_TO_F(cname));

        if (!psx.enableAnalogSticks()) {
          Serial.println(F("Cannot enable analog sticks"));
        }

        //~ if (!psx.setAnalogMode (false)) {
        //~ Serial.println (F("Cannot disable analog mode"));
        //~ }
        //~ delay (10);

        if (!psx.enableAnalogButtons()) {
          Serial.println(F("Cannot enable analog buttons"));
        }

        if (!psx.exitConfigMode()) {
          Serial.println(F("Cannot exit config mode"));
        }
      }

      haveController = true;
    }
  } else {              // the controller is connected
    if (!psx.read()) {  // the controller can't be read
      Serial.println(F("Controller lost :("));
      haveController = false;
    } else {  // the controller is responding
      fastDigitalWrite(PIN_BUTTONPRESS, !!psx.getButtonWord());
      dumpButtons(psx.getButtonWord());

      byte lx, ly;
      psx.getLeftAnalog(lx, ly);
      if (lx != slx || ly != sly) {
        dumpAnalog("Left", lx, ly);
        slx = lx;
        sly = ly;
      }

      byte rx, ry;
      psx.getRightAnalog(rx, ry);
      if (rx != srx || ry != sry) {
        dumpAnalog("Right", rx, ry);
        srx = rx;
        sry = ry;
      }
    }
  }
}