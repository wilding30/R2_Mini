#include <Wire.h>
#include <math.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_PWMServoDriver.h>
//#include <FS_MX1508.h>

// Set up Xbox controllder
// any xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;

// Vairables for motor drive control
#define IN1 16 //Left Motor Driver forward
#define IN2 17 //Left Motor Driver backward
#define IN3 18 //Right Motor Driver forward
#define IN4 19 //Right Motor Driver backward
const int deadzonexy = 40; //original value is 30, this will give the motors enough power to move, add eliminate centering jitter
float uprightdrivespeed = 0.30; // Upright 2 legged drive speed multiplier, when in 2 leg mode the drive speed is reduced by multplying it by this factor
int maxDriveSpeed = 200;
int resetDriveSpeed = maxDriveSpeed;
int x_axis = 0;                     // value from joystick
int y_axis = 0;                     // value from joystick
int rawLeft;
int rawRight;
float diff;                       // value for differential steering

void setup() {
  // Begin serial monitor
  Serial.begin(115200);

  // Startup the Xbox controller
  Serial.println("Starting NimBLE Client");
  xboxController.begin();

  //Start the main drive motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(IN1,0);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,0);

}

void loop() {
  xboxController.onLoop();
  if (xboxController.isConnected()) {
    if (xboxController.isWaitingForFirstNotification()) {
      Serial.println("waiting for first notification");
    } else {
      motordrivers();
      //readXboxValues();
    }
  } else {
    Serial.println("not connected");
    if (xboxController.getCountFailedConnection() > 2) {
      ESP.restart();
    }
  }
  delay(50);
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

  rawLeft = y_axis + x_axis;
  rawRight = y_axis - x_axis;

  Serial.println("X axis = ");
  Serial.println(x_axis);
  Serial.println("Y axis = ");
  Serial.println(y_axis);

  /*diff = abs(abs(x_axis) - abs(y_axis));
  rawLeft = rawLeft < 0 ? rawLeft - diff : rawLeft + diff;
  rawRight = rawRight < 0 ? rawRight - diff : rawRight + diff;*/

  rawLeft = constrain(rawLeft, -maxDriveSpeed, maxDriveSpeed); // constrain the PWM to max 255
  rawRight = constrain(rawRight, -maxDriveSpeed, maxDriveSpeed);
  Serial.println("Left motor = ");
  Serial.print(rawLeft);
  Serial.println("Right motor = ");
  Serial.print(rawRight);

  if (rawLeft > deadzonexy && rawRight > deadzonexy) {
    analogWrite(IN1,rawRight);
    analogWrite(IN3,rawLeft);
  } else if (rawLeft < -deadzonexy && rawRight < -deadzonexy) {
    analogWrite(IN2,-rawRight);
    analogWrite(IN4,-rawLeft);
  } else {
    analogWrite(IN1,0);
    analogWrite(IN2,0);
    analogWrite(IN3,0);
    analogWrite(IN4,0);
  }
  
  /*if (rawLeft > deadzonexy) {
    analogWrite(IN3,rawLeft);
    analogWrite(IN4,0);
  } else if (rawLeft < -deadzonexy) {
    analogWrite(IN3,0);
    analogWrite(IN4,-rawLeft);
  } else {
    analogWrite(IN3,0);
    analogWrite(IN4,0);
  }
  if (rawRight > deadzonexy) {
    analogWrite(IN1,rawRight);
    analogWrite(IN2,0);
  } else if (rawRight < -deadzonexy) {
    analogWrite(IN1,0);
    analogWrite(IN2,-rawRight);
  } else {
    analogWrite(IN1,0);
    analogWrite(IN2,0);
  }*/
}