#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Servo.h>  // Required for RC PWM
#include <Arduino.h>

const char *ssid = "HimalPixel";
const char *password = "abcdef11";

// Firebase Realtime Database Info
const char *firebaseHost = "stroke-rehab-arm-default-rtdb.asia-southeast1.firebasedatabase.app";
const char *firebaseAuth = "AIzaSyBfr5aBDUGnIKaOS62m9za4smyFWQae6tk";

// Device Configuration
const String rehabManus = "rehab123";

Servo esc;

#define Relay_1 D1
#define Relay_2 D2
#define Motor_PWM D4  // ESC signal pin

bool switch1 = false;
bool switch2 = true;
int PWMSignal = 0;

void connectToWiFi();
void readDataFromFirebase();

void setup() {
  Serial.begin(115200);
  connectToWiFi();

  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_2, OUTPUT);

  esc.attach(Motor_PWM);  // Attach ESC signal pin
}

void loop() {
  readDataFromFirebase();

  digitalWrite(Relay_1, switch1 ? HIGH : LOW);
  digitalWrite(Relay_2, switch2 ? HIGH : LOW);

  // Map 0-100 to 1000-2000us pulse width
  int escPulse = map(PWMSignal, 0, 100, 1000, 2000);
  esc.writeMicroseconds(escPulse);  // Send proper RC PWM signal

  Serial.print("PWMSignal (0-100): ");
  Serial.print(PWMSignal);
  Serial.print(" | ESC Pulse: ");
  Serial.println(escPulse);

  delay(100);
}

void connectToWiFi()
{
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }

  Serial.println(" Connected to WiFi!");
}

void readDataFromFirebase()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure(); // Skip SSL verification (for development only)

    // Construct Firebase URL
    String url = String("https://") + firebaseHost + "/rehabManus/" + rehabManus + ".json?auth=" + firebaseAuth;

    http.begin(client, url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
      String payload = http.getString();
      Serial.println("Data received successfully!");
      Serial.println("Response: " + payload);

      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);

      if (!error)
      {
        switch1 = doc["switch1"] | false;
        switch2 = doc["switch2"] | false;
        PWMSignal = doc["PWMSignal"] | 0;

        // Clamp value to 0-100 to avoid unexpected behavior
        PWMSignal = constrain(PWMSignal, 0, 100);
      }
      else
      {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
      }
    }
    else
    {
      Serial.print("Error reading data: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
  else
  {
    Serial.println("WiFi not connected!");
  }
}