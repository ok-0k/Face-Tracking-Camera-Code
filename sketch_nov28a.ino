
#include <Servo.h>

const int PAN_PIN = 9;   
const int TILT_PIN = 10; 
const int SERVO_MIN = 10;
const int SERVO_MAX = 170;
const int START_POS_PAN = 140; 
const int START_POS_TILT = 140;

Servo panServo;
Servo tiltServo;

String inputString = "";  
bool stringComplete = false;  

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);
  panServo.write(START_POS_PAN);
  tiltServo.write(START_POS_TILT);
  delay(500);
}

void loop() {
  if (stringComplete) {
    int commaIndex = inputString.indexOf(',');

    if (commaIndex > 0) {
      String panStr = inputString.substring(0, commaIndex);
      String tiltStr = inputString.substring(commaIndex + 1);
      int panAngle = panStr.toInt();
      int tiltAngle = tiltStr.toInt();
      panAngle = constrain(panAngle, SERVO_MIN, SERVO_MAX);
      tiltAngle = constrain(tiltAngle, SERVO_MIN, SERVO_MAX);
      panServo.write(panAngle);
      tiltServo.write(tiltAngle);
    }
    inputString = "";
    stringComplete = false;
  }
}
void serialEvent() {
  while (Serial.available()) {

    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}