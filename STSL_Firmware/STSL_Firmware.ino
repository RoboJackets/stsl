#include <WiFi.h>
#include <Adafruit_APDS9960.h>
#include <esp32-hal-ledc.h>

const char * ssid = "RJ_TRAINII_00";
const char * password = "robojackets";
IPAddress huzzah_ip(10,10,10,1);
IPAddress network_mask(255,255,255,0);
uint16_t port = 80;

int led = 13;
int led_state = HIGH;
unsigned long led_time;

int line_center_pin = A2;
int line_offset_pin = A3;

// NOTE: These will change when the new board design comes in
int us_trigger_pin = A0;
int us_echo_pin = 33;

int left_a_pin = A1;
int left_b_pin = A12;
int right_a_pin = A10;
int right_b_pin = A8;
int lift_a_pin = A7;
int lift_b_pin = A6;

int left_a_channel = 1;
int left_b_channel = 2;
int right_a_channel = 3;
int right_b_channel = 4;
int lift_a_channel = 5;
int lift_b_channel = 6;

WiFiServer server(port);

Adafruit_APDS9960 apds;

void setup() {
  Serial.begin(115200);
  Serial.println("Setting up...");
  
  pinMode(led, OUTPUT);

  pinMode(line_center_pin, INPUT);
  pinMode(line_offset_pin, INPUT);
  
  pinMode(us_trigger_pin, OUTPUT);
  pinMode(us_echo_pin, INPUT);

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

  Serial.println("Setting up motors");
  ledcSetup(left_a_channel,50,8);
  ledcSetup(left_b_channel,50,8);
  ledcSetup(right_a_channel,50,8);
  ledcSetup(right_b_channel,50,8);
  ledcSetup(lift_a_channel,50,8);
  ledcSetup(lift_b_channel,50,8);
  ledcAttachPin(left_a_pin, left_a_channel);
  ledcAttachPin(left_b_pin, left_b_channel);
  ledcAttachPin(right_a_pin, right_a_channel);
  ledcAttachPin(right_b_pin, right_b_channel);
  ledcAttachPin(lift_a_pin, lift_a_channel);
  ledcAttachPin(lift_b_pin, lift_b_channel);
  stopMotors();

  Serial.println("Enabling AP.");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("Wait 100 ms for AP_START...");
  delay(100);
  Serial.println("Configurnig AP.");
  WiFi.softAPConfig(huzzah_ip, huzzah_ip, network_mask);

  Serial.println("WiFi ready.");

  server.begin();
  Serial.println("Server ready.");
  
  Serial.print("Waiting for connections at ");
  Serial.print(WiFi.softAPIP());
  Serial.print(":");
  Serial.println(port);

  digitalWrite(led, HIGH);
  led_time = millis();
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
  digitalWrite(us_trigger_pin, LOW);
  delayMicroseconds(2);
  // Send the ping
  digitalWrite(us_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(us_trigger_pin, LOW);
  // Measure how long the echo pin was held high (pulse width)
  return pulseIn(us_echo_pin, HIGH) / 58.0;
}

void stopMotors() {
  ledcWrite(left_a_channel, 0);
  ledcWrite(left_b_channel, 0);
  ledcWrite(right_a_channel, 0);
  ledcWrite(right_b_channel, 0);
  ledcWrite(lift_a_channel, 0);
  ledcWrite(lift_b_channel, 0);
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
        writeString(client, String(analogRead(line_center_pin)) + "\n");
      }
      else if(command == "GetLineOffset") {
        writeString(client, String(analogRead(line_offset_pin)) + "\n");
      } else if(command == "GetUltrasonic") {
        writeString(client, String(getUltrasonicDistance()) + "\n");
      } else if(command == "StopMotors") {
        stopMotors();
      } else if(command.substring(0,8) == "SetMotor") {
        String motor = command.substring(8,9);
        int channel = 0;
        int speed = command.substring(9).toInt();
        if(speed > 0) {
          if(motor == "L") channel = left_a_channel;
          else if(motor == "R") channel = right_a_channel;
          else if(motor == "I") channel = lift_a_channel;
        } else {
          speed *= -1;
          if(motor == "L") channel = left_b_channel;
          else if(motor == "R") channel = right_b_channel;
          else if(motor == "I") channel = lift_b_channel;
        }
        ledcWrite(channel, speed);
      }
    }
  } else {
    unsigned long now = millis();
    if((now - led_time) > 500) {
      led_state = !led_state;
      digitalWrite(led, led_state);
      led_time = now;
    }
  }
}

