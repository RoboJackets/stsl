#include <WiFi.h>
#include <Adafruit_APDS9960.h>

const char * ssid = "RJ_TRAINII_00";
const char * password = "robojackets";
IPAddress huzzahIP(10,10,10,1);
IPAddress networkMask(255,255,255,0);
uint16_t port = 80;

int led = 13;
int ledState = HIGH;
unsigned long ledTime;

int lineSensorCenterPin = A2;
int lineSensorOffsetPin = A3;

// NOTE: These will change when the new board design comes in
int usTriggerPin = A0;
int usEchoPin = 33;

WiFiServer server(port);

Adafruit_APDS9960 apds;

void setup() {
  Serial.begin(115200);
  Serial.println("Setting up...");
  
  pinMode(led, OUTPUT);

  pinMode(lineSensorCenterPin, INPUT);
  pinMode(lineSensorOffsetPin, INPUT);
  
  pinMode(usTriggerPin, OUTPUT);
  pinMode(usEchoPin, INPUT);

  Serial.println("Pins ready.");

  Serial.println("Setting up ADPS 9960");
  if(apds.begin()) {
    apds.enableProximity(true);
    apds.enableGesture(true);
    apds.enableColor(true);
    Serial.println("ADPS 9960 Ready.");
  } else {
    Serial.println("ADPS 9960 Failed to initialize.");
  }

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

  digitalWrite(led, HIGH);
  ledTime = millis();
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

void writeString(WiFiClient &client, String &str) {
  size_t totalBytesSent = 0;
  const char* buf = str.c_str();
  size_t bufSize = str.length();
  while(totalBytesSent < str.length()) {
    size_t bytesSent = client.write(buf, bufSize);
    buf += bytesSent;
    bufSize -= bytesSent;
    totalBytesSent += bytesSent;
  }
}

double getUltrasonicDistance() {
  digitalWrite(usTriggerPin, LOW);
  delayMicroseconds(2);
  // Send the ping
  digitalWrite(usTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(usTriggerPin, LOW);
  // Measure how long the echo pin was held high (pulse width)
  return pulseIn(usEchoPin, HIGH) / 58.0;
}

void loop() {
  WiFiClient client = server.available();
  if(client) {
    Serial.println("Client connected");
    digitalWrite(led, HIGH);
    while(client.connected()) {
      String command = readLine(client);
      if(command == "GetGesture") {
        uint8_t gesture = apds.readGesture();
        String response = "";
        if(gesture == APDS9960_DOWN) response = "DOWN\n";
        else if(gesture == APDS9960_UP) response = "UP\n";
        else if(gesture == APDS9960_LEFT) response = "LEFT\n";
        else if(gesture == APDS9960_RIGHT) response = "RIGHT\n";
        else response = "NONE\n";
        writeString(client, response);
      }
      else if(command == "GetColor") {
        while(!apds.colorDataReady()) {
          delay(5);
        }
        uint16_t r, g, b, c;
        apds.getColorData(&r, &g, &b, &c);
        String response = String(r) + " " + String(g) + " " + String(b) + " " + String(c) + "\n";
        writeString(client, response);
      }
      else if(command == "GetProximity") {
        // TODO scale proximity readings to real world units
        writeString(client, String(apds.readProximity()) + "\n");
      }
      else if(command == "GetLineCenter") {
        writeString(client, String(analogRead(lineSensorCenterPin)) + "\n");
      }
      else if(command == "GetLineOffset") {
        writeString(client, String(analogRead(lineSensorOffsetPin)) + "\n");
      } else if(command == "GetUltrasonic") {
        writeString(client, String(getUltrasonicDistance()) + "\n");
      }
    }
  } else {
    unsigned long now = millis();
    if((now - ledTime) > 500) {
      ledState = !ledState;
      digitalWrite(led, ledState);
      ledTime = now;
    }
  }
}

