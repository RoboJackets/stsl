#include <WiFi.h>

const char * ssid = "RJ_TRAINII_00";
const char * password = "robojackets";

int led = 13;

WiFiServer server(80);

void setup() {
  pinMode(led, OUTPUT);

  WiFi.softAP(ssid, password);
}

String readLine(WiFiClient &client) {
  String line = "";
  while(client.connected()) {
    if(client.available()) {
      char c = client.read();
      if(c == '\n') {
        return line;
      } else {
        line += c;
      }
    }
  }
  /* If the client disconnects before we get a newline character,
   * just return whatever we've got so far.
   */
  return line;
}

void loop() {
  WiFiClient client = server.available();
  if(client) {
    while(client.connected()) {
      String command = readLine(client);
      if(command == "SetOnBoardLEDOn") {
        digitalWrite(led, HIGH);
      } else if(command == "SetOnBoardLEDOff") {
        digitalWrite(led, LOW);
      }
    }
  }
}

