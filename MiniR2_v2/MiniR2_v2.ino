#include <Wire.h>
#include <math.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <Adafruit_PWMServoDriver.h>
//#include <FS_MX1508.h>

// Set up Xbox controllder
// any xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;

//Call boards for i2c
Adafruit_PWMServoDriver pwmDome = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwmBody = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
//#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
//#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Variables to adust for front arm open and close limits for servo movement
int frontArm1Pin = 0;
int frontArm1Min = 30;
int frontArm1Max = 170;
int triggerLeft = frontArm1Min;
int frontArm2Pin = 1;
int frontArm2Min = 30;
int frontArm2Max = 170;
int triggerRight = frontArm2Min;

// Variables to adust for Centre leg lift and Tilt servo mechanism
int tiltMin = 133;
int tiltMax = 40;
int tiltPin = 5;
int liftUp = 125;
int liftDown = 28;
int liftPin = 4;
bool state232 = false;
int delay232 = 10;

// Vairables for motor drive control
#define IN1 16 //Left Motor Driver forward
#define IN2 17 //Left Motor Driver backward
#define IN3 18 //Right Motor Driver forward
#define IN4 19 //Right Motor Driver backward
const int deadzonexy = 30; //original value is 30, this will give the motors enough power to move, add eliminate centering jitter
float uprightdrivespeed = 0.30; // Upright 2 legged drive speed multiplier, when in 2 leg mode the drive speed is reduced by multplying it by this factor
int maxDriveSpeed = 170;
int resetDriveSpeed = maxDriveSpeed;
int x_axis = 0;                     // value from joystick
int y_axis = 0;                     // value from joystick
int rawLeft;
int rawRight;
float diff;                       // value for differential steering
//uint32_t bitsResolution = 8;
//uint32_t pwmFrequency = 1000;

//Define the motor driver details on the MX1508
//MX1508 motorA(IN1, IN2, FAST_DECAY);
//MX1508 motorB(IN3, IN4, FAST_DECAY);

int soundbutton = 0;           //value for the sound player, can be used to trigger lights
int lift = 0;
int tilt = 0;
int periscope = 0;

int ledPin = 13;           // pin for the LED in the dome to be turned on and off
int ledinterval = 200;    // delay to blink the LED lights in the dome
int ledinterval1 = 200;
int ledinterval2 = 200;
int ledinterval3 = 200;
int ledinterval4 = 200;
int ledState = LOW;             // ledState used to set the LED
int ledState1 = LOW;
int ledState2 = LOW;
int ledState3 = LOW;
int ledState4 = LOW;

// float output1 = 0;              // Output from first channel
// float output2 = 0;              // Output from second channel
// float output3 = 0;              // Output for Dome motor
// float output4 = 0;              // Front arm servo


// Variables to adust for periscope lift servo
int periUp = 25;
int periDown = 135;

// values to adust for the LED random number blink, min and max delays
int ledOnMin = 200; // value in milliseconds
int ledOnMax = 500;
int ledOffMin = 50; // value in milliseconds
int ledOffMax = 150;

// Holo position values
int holoCentreVal = 300;

// Dome lights
int hololedinterval1 = 200;    // delay to blink the LED lights in the dome
int hololedinterval2 = 200;    // delay to blink the LED lights in the dome
int hololedinterval3 = 200;    // delay to blink the LED lights in the dome
int periledinterval = 500;    // delay to blink the LED lights in the Periscope
int hololedState1 = LOW;       // ledState used to set the LED
int hololedState2 = LOW;       // ledState used to set the LED
int hololedState3 = LOW;       // ledState used to set the LED
int periledState = LOW;       // ledState used to set the LED
int hololedonmin = 300;
int hololedonmax = 1000;
int hololedoffmin = 100; // value in milliseconds
int hololedoffmax = 500;
int periledonmin = 300; //periscope LED on off values
int periledonmax = 600;
int periledoffmin = 300; //
int periledoffmax = 600;

bool debug = true;
static unsigned long lastMilli = 0;
unsigned long currentMillis; // time current
unsigned long previousMillis = 0;        // will store last time LED was updated


void setup() {
  // Begin serial monitor
  Serial.begin(115200);

  // Startup the Xbox controller
  Serial.println("Starting NimBLE Client");
  xboxController.begin();

  //Start the body PCA9685 servo controller
  pwmBody.begin();
  pwmBody.setOscillatorFrequency(27000000);
  pwmBody.setPWMFreq(SERVO_FREQ);
  //ServoBodySetup();

  //Start the dome PCA9685 servo controller
  pwmDome.begin();
  pwmDome.setOscillatorFrequency(27000000);
  pwmDome.setPWMFreq(SERVO_FREQ);
  //ServoDomeSetup();

  //Start the main drive motors
  //motorA.setResolution(bitsResolution);
  //motorB.setResolution(bitsResolution);
  //motorA.setFrequency(pwmFrequency);
  //motorB.setFrequency(pwmFrequency);
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
      FrontArmServo();
      motordrivers();
      if (xboxController.xboxNotif.btnY) {
        Serial.println("Y is pressed");
        tiltmechanism();
      }
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

void ServoBodySetup() {
  pwmBody.setPWM(frontArm1Pin, 0, frontArm1Min); //front arm 1 position 0
  pwmBody.setPWM(frontArm2Pin, 0, frontArm2Min); //front arm 2 position 1
  pwmBody.setPWM(2, 0, 0); //
  pwmBody.setPWM(liftPin, 0, liftUp); //centre leg lift position 3
  pwmBody.setPWM(tiltPin, 0, tiltMin);// Body Tilt position 4
  pwmBody.setPWM(3, 0, 0);
  pwmBody.setPWM(6, 0, 0); //
  pwmBody.setPWM(7, 0, 0); //
  pwmBody.setPWM(8, 0, 0); //
  pwmBody.setPWM(9, 0, 0); //
  pwmBody.setPWM(10, 0, 0); //
  pwmBody.setPWM(11, 0, 0); //
  pwmBody.setPWM(12, 0, 0); //
  pwmBody.setPWM(13, 0, 0); //
  pwmBody.setPWM(14, 0, 0); //
  pwmBody.setPWM(15, 0, 300);  //dome rotation centred for servo motor
}

void ServoDomeSetup() {
  pwmDome.setPWM(0, 4096, 0); //Dome lights
  pwmDome.setPWM(1, 4096, 0); //Dome lights
  pwmDome.setPWM(2, 4096, 0); //Dome lights
  pwmDome.setPWM(3, 4096, 0); //Dome lights
  pwmDome.setPWM(4, 0, holoCentreVal); // Holo Servo 1
  pwmDome.setPWM(5, 0, holoCentreVal); // Holo Servo 2
  pwmDome.setPWM(6, 0, holoCentreVal); // Holo Servo 3
  pwmDome.setPWM(7, 4096, 0); // Dome lights
  pwmDome.setPWM(8, 0, 0); //
  pwmDome.setPWM(9, 0, 0); //
  pwmDome.setPWM(10, 0, periDown); //Periscope
  pwmDome.setPWM(11, 4096, 0); //Periscope LED
  pwmDome.setPWM(12, 0, 0); //
  pwmDome.setPWM(13, 4096, 0); // holo light 1
  pwmDome.setPWM(14, 4096, 0); // holo light 2
  pwmDome.setPWM(15, 4096, 0); // holo light 3
}

void FrontArmServo() {
  triggerLeft = int(((float)xboxController.xboxNotif.trigLT / 1024) * 100);
  triggerLeft = constrain(triggerLeft, 0, 99);
  triggerLeft = map(triggerLeft, 0, 99, frontArm1Min, frontArm1Max);
  triggerLeft = map(triggerLeft, frontArm1Min, frontArm1Max, SERVOMIN, SERVOMAX);
  pwmBody.setPWM(frontArm1Pin, 0, triggerLeft);

  triggerRight = int(((float)xboxController.xboxNotif.trigRT / 1024) * 100);
  triggerRight = constrain(triggerRight, 0, 99);
  triggerRight = map(triggerRight, 0, 99, frontArm2Min, frontArm2Max);
  triggerRight = map(triggerRight, frontArm2Min, frontArm2Max, SERVOMIN, SERVOMAX);
  pwmBody.setPWM(frontArm2Pin, 0, triggerRight);
}

void tiltmechanism() {

  analogWrite(IN3,0);
  analogWrite(IN4,0);
  if (state232 = false) {
    for (int i = liftUp; liftDown; i--) {
      pwmBody.setPWM(liftPin, 0, i);
      Serial.println("Leg Down = " + i);
      delay(delay232);
    }
    for (int i = tiltMin; tiltMax; i++) {
      pwmBody.setPWM(tiltPin, 0, i);
      Serial.println("Tilt to Max = " + i);
      delay(delay232);
    }
    state232 = true;
    maxDriveSpeed = maxDriveSpeed * uprightdrivespeed;
  } else if (state232 = false) {
    for (int i = liftDown; liftUp; i++) {
      pwmBody.setPWM(liftPin, 0, i);
      Serial.println("Leg Up = " + i);
      delay(delay232);
    }
    for (int i = tiltMin; tiltMax; i--) {
      pwmBody.setPWM(tiltPin, 0, i);
      Serial.println("Tilt to Min = " + i);
      delay(delay232);
    }
    state232 = false;
    maxDriveSpeed = resetDriveSpeed;
  }
}

void motordrivers() {     // for the main drive wheels

  //////////////////////////read in the values from bluetooth///////////////////
  x_axis = map(((float)xboxController.xboxNotif.joyLHori / joystickMax) * 1000, 0, 1000, -maxDriveSpeed, maxDriveSpeed);
  y_axis = map(((float)xboxController.xboxNotif.joyLHori / joystickMax) * 1000, 1000, 0, -maxDriveSpeed, maxDriveSpeed);

  rawLeft = y_axis + x_axis;
  rawRight = y_axis - x_axis;

  diff = abs(abs(x_axis) - abs(y_axis));
  rawLeft = rawLeft < 0 ? rawLeft - diff : rawLeft + diff;
  rawRight = rawRight < 0 ? rawRight - diff : rawRight + diff;

  rawLeft = constrain(rawLeft, -maxDriveSpeed, maxDriveSpeed); // constrain the PWM to max 255
  rawRight = constrain(rawRight, -maxDriveSpeed, maxDriveSpeed);
  Serial.println("Left motor = " + rawLeft);
  Serial.println("Right motor = " + rawRight);

  if (rawLeft > deadzonexy) {
    analogWrite(IN2,0);
    analogWrite(IN1,rawLeft);
  } else if (rawLeft < -deadzonexy) {
    analogWrite(IN1,0);
    analogWrite(IN2,-rawLeft);
  } else {
    analogWrite(IN1,0);
    analogWrite(IN2,0);
  }
  if (rawRight > deadzonexy) {
    analogWrite(IN4,0);
    analogWrite(IN3,rawRight);
  } else if (rawRight < -deadzonexy) {
    analogWrite(IN3,0);
    analogWrite(IN4,-rawRight);
  } else {
    analogWrite(IN3,0);
    analogWrite(IN4,0);
  }
}