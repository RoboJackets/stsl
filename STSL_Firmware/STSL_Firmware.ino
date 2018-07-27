#include <WiFi.h>

const char * ssid = "RJ_TRAINII_00";
const char * password = "robojackets";
IPAddress huzzahIP(10,10,10,1);
IPAddress networkMask(255,255,255,0);
uint16_t port = 80;

int led = 13;

WiFiServer server(port);

void setup() {
  Serial.begin(115200);
  Serial.println("Setting up...");
  
  pinMode(led, OUTPUT);

  Serial.println("Pins ready.");

  Serial.println("Enabling AP.");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("Wait 100 ms for AP_START...");
  delay(100);
  Serial.println("Configurnig AP.");
  WiFi.softAPConfig(huzzahIP, huzzahIP, networkMask);

  Serial.println("WiFi ready.");

  server.begin();
  Serial.println("Server ready.");
  
  Serial.print("Waiting for connections at ");
  Serial.print(WiFi.softAPIP());
  Serial.print(":");
  Serial.println(port);
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
    Serial.println("Client connected");
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

