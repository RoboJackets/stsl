#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <BricktronicsShield.h>
#include <BricktronicsButton.h>
#include <BricktronicsLight.h>

BricktronicsButton button(BricktronicsShield::SENSOR_1);
BricktronicsLight light(BricktronicsShield::SENSOR_2);

void setup() {
  Serial.begin(9600);
  BricktronicsShield::begin();
  button.begin();
  light.begin();
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
//    String command = readLine();
    String command = Serial.readStringUntil('\n');
    if(command == "GetButton") {
      Serial.println(button.isPressed());
    } else if(command == "GetLight") {
      Serial.println(light.scaledValue());
    } else if(command == "SetFloodlightOn") {
      light.setFloodlightAlways(true);
    } else if(command == "SetFloodlightOff") {
      light.setFloodlightAlways(false);
    }
  }
}
