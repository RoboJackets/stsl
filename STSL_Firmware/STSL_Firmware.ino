#include <BricktronicsShield.h>
#include <BricktronicsButton.h>

BricktronicsButton button(BricktronicsShield::SENSOR_1);

void setup() {
  Serial.begin(9600);
  BricktronicsShield::begin();
  button.begin();
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
    }
  }
}
