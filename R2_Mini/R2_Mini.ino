/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald

 modified for the ESP32 on March 2017
 by John Bennett

 see http://www.arduino.cc/en/Tutorial/Sweep for a description of the original code

 * Different servos require different pulse widths to vary servo angle, but the range is 
 * an approximately 500-2500 microsecond pulse every 20ms (50Hz). In general, hobbyist servos
 * sweep 180 degrees, so the lowest number in the published range for a particular servo
 * represents an angle of 0 degrees, the middle of the range represents 90 degrees, and the top
 * of the range represents 180 degrees. So for example, if the range is 1000us to 2000us,
 * 1000us would equal an angle of 0, 1500us would equal 90 degrees, and 2000us would equal 1800
 * degrees.
 * 
 * Circuit: (using an ESP32 Thing from Sparkfun)
 * Servo motors have three wires: power, ground, and signal. The power wire is typically red,
 * the ground wire is typically black or brown, and the signal wire is typically yellow,
 * orange or white. Since the ESP32 can supply limited current at only 3.3V, and servos draw
 * considerable power, we will connect servo power to the VBat pin of the ESP32 (located
 * near the USB connector). THIS IS ONLY APPROPRIATE FOR SMALL SERVOS. 
 * 
 * We could also connect servo power to a separate external
 * power source (as long as we connect all of the grounds (ESP32, servo, and external power).
 * In this example, we just connect ESP32 ground to servo ground. The servo signal pins
 * connect to any available GPIO pins on the ESP32 (in this example, we use pin 18.
 * 
 * In this example, we assume a Tower Pro MG995 large servo connected to an external power source.
 * The published min and max for this servo is 1000 and 2000, respectively, so the defaults are fine.
 * These values actually drive the servos a little past 0 and 180, so
 * if you are particular, adjust the min and max values to match your needs.
 */

#include <ESP32Servo.h>

Servo myservo1;  // create servo object to control a servo
Servo myservo2;
// 16 servo objects can be created on the ESP32

int pos1 = 0;    // variable to store the servo position
int pos2 = 0;
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
// Possible PWM GPIO pins on the ESP32-S3: 0(used by on-board button),1-21,35-45,47,48(used by on-board LED)
// Possible PWM GPIO pins on the ESP32-C3: 0(used by on-board button),1-7,8(used by on-board LED),9-10,18-21
//#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
int servoPin1 = 12;
int servoPin2 = 14;
//#elif defined(CONFIG_IDF_TARGET_ESP32C3)
//int servoPin = 7;
//#else
//int servoPin = 18;
//#endif

void setup() {
  Serial.begin(115200);
	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo1.setPeriodHertz(50);    // standard 50 hz servo
	myservo1.attach(servoPin1, 1000, 2000); // attaches the servo on pin 18 to the servo object
	myservo2.setPeriodHertz(50);    // standard 50 hz servo
	myservo2.attach(servoPin2, 1000, 2000); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
  Serial.println("Setting servo1");
  myservo1.write(0);
  Serial.println("Setting servo2");
  myservo2.write(180);
}

void loop() {
  Serial.println("Enter data:");
  while (Serial.available() == 0) {}     //wait for data available
  String readStr = Serial.readString();  //read until timeout
  readStr.trim();
  String servNum = readStr.substring(0,1);
  Serial.println(servNum);                        // remove any \r \n whitespace at the end of the String
  if (servNum = "1") {
    String position = readStr.substring(2);
    int posint = position.toInt();
    MoveServo(1, posint, 15);
  }
  else if (servNum = "2") {
    String position = readStr.substring(2);
    int posint = position.toInt();
    MoveServo(2, posint, 15);
  }
  else {
    Serial.println("Unknown command");
  }
	/*for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		myservo1.write(pos);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
	}
  delay(5000);
	for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
		myservo1.write(pos);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
	}*/
  //delay(5000);
}

void MoveServo(int serv, int newPos, int delTime) {
  newPos = min(newPos, 180);
  newPos = max(newPos, 0);
  if (serv = 1) {
    if (newPos > pos1) {
      for (int i = pos1; i <= newPos; i += 1) {
        myservo1.write(i);
        Serial.println(i);
        delay(delTime);
      }
    }
    else if (newPos < pos1) {
      for (int i = pos1; i >= newPos; i -= 1) {
        myservo1.write(i);
        Serial.println(i);
        delay(delTime);
      }
    }
    pos1 = newPos;
    Serial.println("new servo 1 position is");
    Serial.println(pos1);
  }
  else if (serv = 2) {
    if (newPos > pos2) {
      for (int i = pos2; i <= newPos; i += 1) {
        myservo2.write(i);
        Serial.println(i);
        delay(delTime);
      }
    }
    else if (newPos < pos2) {
      for (int i = pos2; i >= newPos; i -= 1) {
        myservo2.write(i);
        Serial.println(i);
        delay(delTime);
      }
    }
    pos2 = newPos;
    Serial.println("new servo 2 position is");
    Serial.println(pos2);
  }
}