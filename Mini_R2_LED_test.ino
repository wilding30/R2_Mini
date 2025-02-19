#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

//Call boards for i2c
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

const int baudrate = 115200;

struct ledType1{ int pin; int onMin; int onMax; int offMin; int offMax; bool state; int interval; unsigned long millis; int pwmMin; int pwmMax; };

ledType1 PSI_F = {0, 2000, 5000, 500, 1500, false, 200, 0, 10, 1000};
ledType1 PSI_R = {1, 2000, 5000, 500, 1500, false, 200, 0, 10, 1000};
ledType1 HP_F = {10, 2000, 5000, 500, 1500, false, 200, 0, 0, 1000};
ledType1 HP_T = {11, 2000, 5000, 2000, 7000, false, 200, 0, 0, 1000};
ledType1 HP_R = {12, 2000, 5000, 2000, 7000, false, 200, 0, 0, 1000};

struct ledType2{ int pin; int pwmMin1; int pwmMin2; int pwmMax1; int pwmMax2; int pwmCurrent; int pwmTarget; };

ledType2 LP_F_1 = {5, 100, 300, 700, 1000, 0, 1000};
ledType2 LP_F_2 = {6, 100, 300, 700, 1000, 0, 1000};
ledType2 LP_R_1 = {7, 25, 150, 350, 400, 0, 400};
ledType2 LP_R_2 = {8, 300, 600, 1500, 1800, 0, 1400};
ledType2 LP_R_3 = {9, 25, 150, 350, 400, 0, 400};

////////////////////Variables that can be changed to cusomise////////////////////////////////////////////////////////////////////////////////////////////
// values to adust for the LED random number blink, min and max delays

const int holoCentreVal = 333; // 330 slow clockwise, 350 slow counter-clockwise
const int ledLP_incr = 1;

unsigned long currentMillis; // time current

/////////////////////////////////// SETUP ////////////////////////////////////////////////////
void setup() {
  Serial.begin(baudrate);                                                 //Used only for debugging on arduino serial monitor
  Serial.println("Mini Droid!");

  pwm2.begin();
  pwm2.setPWMFreq(50);  // standard for analog servos

  //Set the servos to their start positions
  servoSetup2(); // view function and end of code

  delay(1000);
}

// start of loop ///////////////////////////////////////////////////////////////////////
void loop() {

  currentMillis = millis();
  ledOnOff(PSI_F);
  ledOnOff(PSI_R);
  ledOnOff(HP_F);
  ledOnOff(HP_T);
  ledOnOff(HP_R);
  ledFade(LP_F_1);
  ledFade(LP_F_2);
  ledFade(LP_R_1);
  ledFade(LP_R_2);
  ledFade(LP_R_3);
}
//end of loop ///////////////////////////////////////////////////////////////////////////////////////////

// Startup routine

void servoSetup2() {
  pwm2.setPWM(PSI_F.pin, 0, 0); // PSI Rear 
  pwm2.setPWM(PSI_R.pin, 0, 0); // PSI Front
  pwm2.setPWM(HP_F.pin, 0, 0); //
  pwm2.setPWM(HP_T.pin, 0, 0); //
  pwm2.setPWM(HP_R.pin, 0, 0); //
  pwm2.setPWM(LP_F_1.pin, 0, 0); // Logic Paneil - Front Top
  pwm2.setPWM(LP_F_2.pin, 0, 0); // Logic Paneil - Front Bottom
  pwm2.setPWM(LP_R_1.pin, 0, 0); // Logic Paneil - Rear 1
  pwm2.setPWM(LP_R_2.pin, 0, 0); // Logic Paneil - Rear 2 
  pwm2.setPWM(LP_R_3.pin, 0, 0); //Logic Paneil - Rear 3
  pwm2.setPWM(13, 0, 335); // Holo Servo Rear
  pwm2.setPWM(14, 0, 333); // Holo Servo Top
  pwm2.setPWM(15, 0, 327); // Holo Servo Front
}

void ledOnOff (ledType1 &led) {
  if ((currentMillis - led.millis) >= led.interval) {
    led.millis = millis();
    if (led.state == false) {
      led.state = true;
      led.interval = random(led.onMin, led.onMax);       //set a random time dealy when the sound is being played
      pwm2.setPWM(led.pin, 0, led.pwmMax);
    }
    else {
      led.state = false;
      led.interval = random(led.offMin, led.offMax);       //set a random time dealy when the sound is being played
      pwm2.setPWM(led.pin, 0, led.pwmMin);
    }
  }
}

void ledFade(ledType2 &led) {
  if (led.pwmCurrent < led.pwmTarget) {
    led.pwmCurrent = min(led.pwmCurrent + ledLP_incr, led.pwmTarget);
  } else if (led.pwmCurrent > led.pwmTarget) {
    led.pwmCurrent = max(led.pwmCurrent - ledLP_incr, led.pwmTarget);
  } else if (led.pwmTarget > led.pwmMin2) {
    led.pwmTarget = random(led.pwmMin1, led.pwmMin2);
  } else {
    led.pwmTarget = random(led.pwmMax1, led.pwmMax2);
  }
  pwm2.setPWM(led.pin, 0, led.pwmCurrent);
}