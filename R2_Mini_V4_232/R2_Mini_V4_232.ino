#include <Wire.h>
#include <math.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_PWMServoDriver.h>

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

// Motor vairables
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

// Initialise Servo control boards
Adafruit_PWMServoDriver pwmBody = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwmDome = Adafruit_PWMServoDriver(0x41);

// Initialise body servos
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Variables to adust for front arm open and close limits for servo movement
int frontArm1Pin = 14;
int frontArm1Min = 155;
int frontArm1Max = 275;
int triggerLeft = frontArm1Min;
int frontArm2Pin = 15;
int frontArm2Min = 155;
int frontArm2Max = 275;
int triggerRight = frontArm2Min;

// Variables for 232 and dome spin
bool stand = false;
int tiltPin = 10;
int tiltMin = 200;
int tiltMax = 400;
bool tilt = true;
int liftPin = 11;
int liftMin = 200;
int liftMax = 400;
bool lift = false;
int domePin = 7;
int domeMin = 150;
int domeMax = 400;
int domeMid = 270;
int domeDeadZone = 10;
int pos = 0;    // variable to store the servo position

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
  // Motor A
  pinMode(EN_A, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  analogWrite(EN_A, 0);
  digitalWrite(IN1_A, LOW);
  digitalWrite(IN2_A, LOW);

  // Motor B
  pinMode(EN_B, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);
  analogWrite(EN_B, 0);
  digitalWrite(IN1_B, LOW);
  digitalWrite(IN2_B, LOW);

  //Start the body PCA9685 servo controller
  pwmBody.begin();
  pwmBody.setOscillatorFrequency(27000000);
  pwmBody.setPWMFreq(SERVO_FREQ);
  ServoBodySetup();

  //Start the dome PCA9685 servo controller
  pwmDome.begin();
  pwmDome.setOscillatorFrequency(27000000);
  pwmDome.setPWMFreq(1600);
  ServoDomeSetup();

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
      if (xboxController.xboxNotif.btnDirUp) {
        Serial.println("DirUp is pressed");
        stand = true;
        ChangePos();
      }
      if (xboxController.xboxNotif.btnDirDown) {
        stand = false;
        ChangePos();
      }
    }
  } else {
    Serial.println("not connected");
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
    ledFlag = false;
  }
  ledCheck();
  //ledFlag = false;
  delay(50);
  ServoDomeMain();
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
  motorLeft = abs(rawLeft);
  motorRight = abs(rawRight);

  if (motorLeft < deadzonexy) {
    motorLeft = 0;
  }
  if (motorRight < deadzonexy) {
    motorRight = 0;
  }
  
  analogWrite(EN_A, motorLeft);
  analogWrite(EN_B, motorRight);

  if (rawLeft > 0) {
    digitalWrite(IN1_A, HIGH);
    digitalWrite(IN2_A, LOW);
  } else if (rawLeft < 0) {
    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, HIGH);
  } else {
    digitalWrite(IN1_A, LOW);
    digitalWrite(IN2_A, LOW);
  }
  if (rawRight > 0) {
    digitalWrite(IN1_B, HIGH);
    digitalWrite(IN2_B, LOW);
  } else if (rawRight < -deadzonexy) {
    digitalWrite(IN1_B, LOW);
    digitalWrite(IN2_B, HIGH);
  } else {
    digitalWrite(IN1_B, LOW);
    digitalWrite(IN2_B, LOW);
  }

  Serial.println("Right stick x value is ");
  pos = map((float)xboxController.xboxNotif.joyRHori / joystickMax * 1000, 0, 1000, domeMin, domeMax);
  Serial.print(pos);
  if (abs(pos - domeMid) > domeDeadZone) {
    pwmBody.setPWM(domePin, 0, pos);
  } else {
    pwmBody.setPWM(domePin, 0, domeMid);
  }
  
}

void ServoBodySetup() {
  pwmBody.setPWM(frontArm1Pin, 0, SERVOMIN); //front arm 1 position 0
  pwmBody.setPWM(frontArm2Pin, 0, SERVOMIN); //front arm 2 position 1
  pwmBody.setPWM(liftPin, 0, liftMin); //centre leg lift position 3
  pwmBody.setPWM(tiltPin, 0, tiltMin);// Body Tilt position 4
  pwmBody.setPWM(domePin, 0, domeMid);
}

void ServoDomeSetup() {
  pwmBody.setPWM(0, 0, 0); //test LED position 0
}

void ServoDomeMain() {
  pwmBody.setPWM(0, 0, 4096); //test LED on
  delay(1000);
  pwmBody.setPWM(0, 0, 0); //test LED off  
}

void ServoBodyMain() {
  triggerLeft = map((float)xboxController.xboxNotif.trigLT, 0, 1024, frontArm1Min, frontArm1Max);
  triggerRight = map((float)xboxController.xboxNotif.trigRT, 0, 1024, frontArm2Min, frontArm2Max);
  triggerLeft = constrain(triggerLeft, SERVOMIN, SERVOMAX);
  triggerRight = constrain(triggerRight, SERVOMIN, SERVOMAX);
  /*Serial.print("Left pwm is ");
  Serial.println(triggerLeft);
  Serial.println("Right pwm is ");
  Serial.println(triggerRight);*/
  pwmBody.setPWM(frontArm1Pin, 0, triggerLeft); //front arm 1
  pwmBody.setPWM(frontArm2Pin, 0, triggerRight); //front arm 2
}

void ChangePos() {
  if (stand = true) {
    if (tilt = true) {
      for (pos = tiltMin; pos <= tiltMax; pos += 1) {
          pwmBody.setPWM(tiltPin, 0, pos);
          delay(10);
          tilt = false;
      }
      return;
    }
    if (lift = false) {
      for (pos = liftMin; pos <= liftMax; pos += 1) {
          pwmBody.setPWM(liftPin, 0, pos);
          delay(10);
          lift = true;
      }
      return;
    }
  } else {
    if (lift = true) {
      for (pos = liftMax; pos >= liftMin; pos -= 1) {
          pwmBody.setPWM(liftPin, 0, pos);
          delay(10);
          lift = false;
      }
      return;
    }
    if (tilt = false) {
      for (pos = tiltMax; pos >= tiltMin; pos -= 1) {
          pwmBody.setPWM(tiltPin, 0, pos);
          delay(10);
          tilt = true;
      }
      return;
    }
  }
}