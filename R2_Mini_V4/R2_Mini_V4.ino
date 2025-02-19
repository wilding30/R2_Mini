#include <Wire.h>
#include <math.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_PWMServoDriver.h>
#include <L298NX2.h>

// Set up Xbox controllder
// any xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;

// LED pin to indicate BT connectivity
const int ledPin = 2;
bool ledFlag = false;
bool ledState = false;

// Vairables for motor drive control
// Motor Pin definition
const unsigned int EN_A = 4;
const unsigned int IN1_A = 16;
const unsigned int IN2_A = 17;

const unsigned int IN1_B = 18;
const unsigned int IN2_B = 19;
const unsigned int EN_B = 5;

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

const int deadzonexy = 30; //original value is 30, this will give the motors enough power to move, add eliminate centering jitter
float uprightdrivespeed = 0.30; // Upright 2 legged drive speed multiplier, when in 2 leg mode the drive speed is reduced by multplying it by this factor
int maxDriveSpeed = 255;
int resetDriveSpeed = maxDriveSpeed;
int x_axis = 0;                     // value from joystick
int y_axis = 0;                     // value from joystick
int rawLeft;
int rawRight;
int motorLeft;
int motorRight;

// Initialise body servos
Adafruit_PWMServoDriver pwmBody = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Variables to adust for front arm open and close limits for servo movement
int frontArm1Pin = 14;
int frontArm1Min = 200;
int frontArm1Max = 400;
int triggerLeft = frontArm1Min;
int frontArm2Pin = 15;
int frontArm2Min = 200;
int frontArm2Max = 400;
int triggerRight = frontArm2Min;

void setup() {
  // Begin serial monitor
  Serial.begin(115200);

  // Startup the Xbox controller
  Serial.println("Starting NimBLE Client");

  // set up onboard LED pin
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Start controller
  xboxController.begin();

  //Start the main drive motors
  motors.setSpeed(0);

  //Start the body PCA9685 servo controller
  pwmBody.begin();
  pwmBody.setOscillatorFrequency(27000000);
  pwmBody.setPWMFreq(SERVO_FREQ);
  ServoBodySetup();

}

void loop() {
  xboxController.onLoop();
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } else {
      ledFlag = true;
      motordrivers();
      ServoBodyMain();
    }
  } else {
    Serial.println("not connected");
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }
  ledCheck();
  //ledFlag = false;
  delay(50);
}

void ledCheck()
{
  if (ledFlag != ledState)
  {
    if (ledFlag = true)
    {
      digitalWrite(ledPin, HIGH);
      ledState = true;
    } else
    {
      digitalWrite(ledPin, LOW);
      ledState = false;
    }
  }
}

void readXboxValues() {
  if (xboxController.xboxNotif.btnA) {
    Serial.println("A is pressed");
  }
  if (xboxController.xboxNotif.btnB) {
    Serial.println("B is pressed");
  }
  if (xboxController.xboxNotif.btnX) {
    Serial.println("X is pressed");
  }
  if (xboxController.xboxNotif.btnY) {
    Serial.println("Y is pressed");
  }
  if (xboxController.xboxNotif.btnLB) {
    Serial.println("LB is pressed");
  }
  if (xboxController.xboxNotif.btnRB) {
    Serial.println("RB is pressed");
  }
  if (xboxController.xboxNotif.btnStart) {
    Serial.println("Start is pressed");
  }
  if (xboxController.xboxNotif.btnShare) {
    Serial.println("Share is pressed");
  }
  if (xboxController.xboxNotif.btnXbox) {
    Serial.println("Xbox is pressed");
  }
  if (xboxController.xboxNotif.btnLS) {
    Serial.println("LS is pressed");
  }
  if (xboxController.xboxNotif.btnRS) {
    Serial.println("RS is pressed");
  }
  if (xboxController.xboxNotif.btnDirUp) {
    Serial.println("DirUp is pressed");
  }
  if (xboxController.xboxNotif.btnDirDown) {
    Serial.println("DirDown is pressed");
  }
  if (xboxController.xboxNotif.btnDirLeft) {
    Serial.println("DirLeft is pressed");
  }
  if (xboxController.xboxNotif.btnDirRight) {
    Serial.println("DirRight is pressed");
  }
  Serial.print("Left trigger is ");
  Serial.println((float)xboxController.xboxNotif.trigLT / 1024);
  Serial.println("Right trigger is ");
  Serial.println((float)xboxController.xboxNotif.trigRT / 1024);
  Serial.println("Left stick x value is ");
  Serial.println((float)xboxController.xboxNotif.joyLHori / joystickMax);
  Serial.println("Left stick y value is ");
  Serial.println((float)xboxController.xboxNotif.joyLVert / joystickMax);
  Serial.println("Right stick x value is ");
  Serial.println((float)xboxController.xboxNotif.joyRHori / joystickMax);
  Serial.println("Right stick y value is ");
  Serial.println((float)xboxController.xboxNotif.joyRVert / joystickMax);
}

void motordrivers() {     // for the main drive wheels

  //////////////////////////read in the values from bluetooth///////////////////
  x_axis = map(((float)xboxController.xboxNotif.joyLHori / joystickMax) * 1000, 0, 1000, -maxDriveSpeed, maxDriveSpeed);
  y_axis = map(((float)xboxController.xboxNotif.joyLVert / joystickMax) * 1000, 1000, 0, -maxDriveSpeed, maxDriveSpeed);

  rawLeft = y_axis - x_axis;
  rawRight = y_axis + x_axis;

  rawLeft = constrain(rawLeft, -maxDriveSpeed, maxDriveSpeed); // constrain the PWM to max 255
  rawRight = constrain(rawRight, -maxDriveSpeed, maxDriveSpeed);
  if (rawLeft < deadzonexy && rawLeft > -deadzonexy) {
    motorLeft = 0;
  } else {
    motorLeft = -abs(rawLeft);
  }
  if (rawRight < deadzonexy && rawRight > -deadzonexy) {
    motorRight = 0;
  } else {
    motorRight = -abs(rawRight);
  }
  
  motors.setSpeedA(motorLeft);
  motors.setSpeedB(motorRight);

  if (rawLeft > deadzonexy) {
    motors.forwardA();
  } else if (rawLeft < -deadzonexy) {
    motors.backwardA();
  } else {
    motors.stopA();
  }
  if (rawRight > deadzonexy) {
    motors.forwardB();
  } else if (rawRight < -deadzonexy) {
    motors.backwardB();
  } else {
    motors.stopB();
  }
}

void ServoBodySetup() {
  pwmBody.setPWM(frontArm1Pin, 0, SERVOMIN); //front arm 1 position 0
  pwmBody.setPWM(frontArm2Pin, 0, SERVOMIN); //front arm 2 position 1
  //pwmBody.setPWM(liftPin, 0, liftUp); //centre leg lift position 3
  //pwmBody.setPWM(tiltPin, 0, tiltMin);// Body Tilt position 4
}

void ServoBodyMain() {
  triggerLeft = map((float)xboxController.xboxNotif.trigLT, 0, 1024, frontArm1Min, frontArm1Max);
  triggerRight = map((float)xboxController.xboxNotif.trigRT, 0, 1024, frontArm2Min, frontArm2Max);
  triggerLeft = constrain(triggerLeft, SERVOMIN, SERVOMAX);
  triggerRight = constrain(triggerRight, SERVOMIN, SERVOMAX);
  Serial.print("Left pwm is ");
  Serial.println(triggerLeft);
  Serial.println("Right pwm is ");
  Serial.println(triggerRight);
  pwmBody.setPWM(14, 0, triggerLeft); //front arm 1
  pwmBody.setPWM(frontArm2Pin, 0, triggerRight); //front arm 2
}