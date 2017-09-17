#include <Wire.h>
//#include <Adafruit_MCP23017.h>
//#include <Encoder.h>
//#include <PID_v1.h>
#include <BricktronicsShield.h>
#include <BricktronicsButton.h>
#include <BricktronicsLight.h>
#include <BricktronicsMotor.h>

BricktronicsButton button(BricktronicsShield::SENSOR_1);
BricktronicsLight light(BricktronicsShield::SENSOR_2);
BricktronicsMotor motorA(BricktronicsShield::MOTOR_1);
BricktronicsMotor motorB(BricktronicsShield::MOTOR_2);

void setup() {
  Serial.begin(9600);
  BricktronicsShield::begin();
  button.begin();
  light.begin();
  motorA.begin();
  motorB.begin();
}

String readLine() {
  String line;
  while(true) {
    char in = Serial.read();
    if(in == -1) {
      continue;
    }
    if(in == '\n') {
      return line;
    }
    line += in;
  }
}

void loop() {

  if(Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if(command == "GetButton") {
      Serial.println(button.isPressed());
    } else if(command == "GetLight") {
      Serial.println(light.scaledValue());
    } else if(command == "SetFloodlightOn") {
      light.setFloodlightAlways(true);
    } else if(command == "SetFloodlightOff") {
      light.setFloodlightAlways(false);
    } else if(command.substring(0,8) == "SetMotor") {
      String motorPort = command.substring(8,9);
      int motorSpeed = command.substring(9,13).toInt();
      if(motorPort == "A") {
        motorA.setFixedDrive(motorSpeed);
      } else if(motorPort == "B") {
        motorB.setFixedDrive(motorSpeed);
      }
    } else if(command == "StopMotors") {
      motorA.setFixedDrive(0);
      motorB.setFixedDrive(0);
    }
  }
}
