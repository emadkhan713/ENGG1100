#include <DigitalIO.h>
#include <Servo.h>
#include <PsxControllerBitBang.h>
#include <avr/pgmspace.h>

/*

// PWM pins are 3, 5, 6, 9, 10, 11
// require PWM: pump speed

const uint8_t PIN_PS2_ATT = 4;
const uint8_t PIN_PS2_CMD = 3;
const uint8_t PIN_PS2_DAT = 2;
const uint8_t PIN_PS2_CLK = 5;

PsxControllerBitBang<PIN_PS2_ATT, PIN_PS2_CMD, PIN_PS2_DAT, PIN_PS2_CLK>
  psx;
boolean haveController = false;

const uint8_t PIN_MOTOR_DIRECTION = 11;
const uint8_t PIN_MOTOR_SPEED = 10;

*/

const uint8_t PIN_PUMP_SPEED = 9;  // PWM pin
const uint8_t PIN_SERVO_H = 7;
const uint8_t PIN_SERVO_V = 8;

static bool PUMP_ENABLED = false;
static uint8_t PUMP_MODE = 0;
const  uint8_t PUMP_STEP = 39;  // approximately 15% increment of 255
static uint8_t PUMP_SPEED = PUMP_STEP * 2;

Servo horizontal;
Servo vertical;

static uint8_t ANGLE_SERVO_H = 90;  // facing straight by default
static uint8_t ANGLE_SERVO_V = 150;  // facing 60deg up by default

void setup() {
  /*
  pinMode(PIN_MOTOR_DIRECTION, OUTPUT);
  pinMode(PIN_MOTOR_SPEED, OUTPUT);
  */
  pinMode(PIN_PUMP_SPEED, OUTPUT);

  /*
  digitalWrite(PIN_MOTOR_DIRECTION, LOW);
  digitalWrite(PIN_MOTOR_SPEED, 0);
  */
  analogWrite(PIN_PUMP_SPEED, 0);

  horizontal.attach(PIN_SERVO_H);
  vertical.attach(PIN_SERVO_V);

  //psx.begin();

  delay(2000);

  Serial.begin(115200);
  Serial.println(F("Ready!"));
}

void loop() {
  /*
  psx.read();
  MotorControl();
  PumpControl();
  NozzleControl(); 
  delay(1000 / 30);  // 30Hz polling rate
  */
  PumpTest();
  ServoTest();
}

void PumpTest() {
  static uint8_t  NUMBER_OF_STEPS = 5
  static uint16_t WAIT_TIME_MS = 2000
  
  for (int i = 0; i < NUMBER_OF_STEPS; i++) {
    analogWrite(PIN_PUMP_SPEED, PUMP_SPEED + i*PUMP_STEP); 
    delay(WAIT_TIME_MS)
  }

  for (int i = NUMBER_OF_STEPS; i > 0; i--) {
    analogWrite(PIN_PUMP_SPEED, PUMP_SPEED + i*PUMP_STEP);
    delay(WAIT_TIME_MS)
  }
  
}

void ServoTest() {
  static uint8_t HALF_PI = 90
  
  for (int i = 0; i < 2*HALF_PI; i++) {
    horizontal.write(i);
    delay(1000 / 30)
  }

  for (int j = HALF_PI; j < 2*HALF_PI; j++) {
    vertical.write(j)
    delay(1000 / 30)
  }

}

void MotorControl() {
  if (psx.buttonPressed(PSB_R2)) {
    digitalWrite(PIN_MOTOR_DIRECTION, LOW);   //forward
    digitalWrite(PIN_MOTOR_SPEED, 255);
  } else if (psx.buttonPressed(PSB_L2)) {
    digitalWrite(PIN_MOTOR_DIRECTION, HIGH);  //backward
    digitalWrite(PIN_MOTOR_SPEED, 255);
  } else {
    digitalWrite(PIN_MOTOR_SPEED, 0);
  }
}

void PumpControl() {
  // 5 speed modes
  if (psx.buttonJustPressed(PSB_R1) && (PWM_PUMP_MODE < 4)) {
    PWM_PUMP_MODE++;
    PUMP_SPEED += PUMP_STEP;
  } else if (psx.buttonJustPressed(PSB_L1) && (PWM_PUMP_MODE > 0)) {
    PWM_PUMP_MODE--;
    PUMP_SPEED -= PUMP_STEP;
  } else {
	;
  }

  // toggle pump using PWM signal
  if (psx.buttonJustPressed(PSB_SQUARE) && !PUMP_ENABLED) {
    PUMP_ENABLED = true;
  } else if (psx.buttonJustPressed(PSB_SQUARE) && PUMP_ENABLED) {
    PUMP_ENABLED = false;
  } else {
    ;
  }

  if (PUMP_ENABLED) {
    analogWrite(PIN_PUMP_SPEED, PUMP_SPEED);
  } else {
    analogWrite(PIN_PUMP_SPEED, 0);
  }

  /* Serial.print(F("Pump | status: "));
  Serial.print(PUMP_ENABLED);
  Serial.print(F(" | mode: "));
  Serial.print(PWM_PUMP_MODE);
  Serial.print(F(" | speed: "));
  Serial.println(PUMP_SPEED); */
}

void NozzleControl() {
  // horizontal servo
  if (psx.buttonPressed(PSB_PAD_LEFT) && (ANGLE_SERVO_H > 0)) {
    ANGLE_SERVO_H--;
    horizontal.write(ANGLE_SERVO_H);
  }
  if (psx.buttonPressed(PSB_PAD_RIGHT) && (ANGLE_SERVO_H < 180)) {
    ANGLE_SERVO_H++;
    horizontal.write(ANGLE_SERVO_H);
  }

  // vertical servo
  if (psx.buttonPressed(PSB_PAD_UP) && (ANGLE_SERVO_V > 90)) {
    ANGLE_SERVO_V--;
    vertical.write(ANGLE_SERVO_V);
  }
  if (psx.buttonPressed(PSB_PAD_DOWN) && (ANGLE_SERVO_V < 180)) {
    ANGLE_SERVO_V++;
    vertical.write(ANGLE_SERVO_V);
  }

  /* Serial.print(F("Servo | H: "));
  Serial.print(ANGLE_SERVO_H);
  Serial.print(F(" | V: "));
  Serial.println(ANGLE_SERVO_V); */
}