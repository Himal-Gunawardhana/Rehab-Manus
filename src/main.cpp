#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <ESP8266Servo.h>


// WiFi Credentials
const char *ssid = "HimalPixel";
const char *password = "abcdef11";

// Firebase Realtime Database Info
const char *firebaseHost = "stroke-rehab-arm-default-rtdb.asia-southeast1.firebasedatabase.app";
const char *firebaseAuth = "AIzaSyBfr5aBDUGnIKaOS62m9za4smyFWQae6tk";

// Device Configuration
const String rehabManus = "rehab123";

// Pin Definitions
#define Relay_1 D1
#define Relay_2 D2
#define Motor_PWM D5

// Control Variables
bool switch1 = false;
bool switch2 = true;
int PWMSignal = 0; // Expecting 0–100 from Firebase

// ESC Control
ESP8266Servo esc;

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
    client.setInsecure(); // ⚠️ For development only

    // Firebase endpoint
    String url = String("https://") + firebaseHost + "/rehabManus/" + rehabManus + ".json?auth=" + firebaseAuth;

    http.begin(client, url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
      String payload = http.getString();
      Serial.println("Firebase response: " + payload);

      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);

      if (!error)
      {
        switch1 = doc["switch1"] | false;
        switch2 = doc["switch2"] | false;
        PWMSignal = doc["PWMSignal"] | 0;
        PWMSignal = constrain(PWMSignal, 0, 100); // Clamp to 0–100
      }
      else
      {
        Serial.print("JSON error: ");
        Serial.println(error.f_str());
      }
    }
    else
    {
      Serial.print("HTTP Error: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
  else
  {
    Serial.println("WiFi not connected.");
  }
}

void setup()
{
  Serial.begin(115200);
  connectToWiFi();

  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_2, OUTPUT);

  esc.attach(Motor_PWM); // Attach ESC control pin (D4)
  delay(2000); // Wait before sending PWM
  Serial.println("ESC Ready.");
}

unsigned long lastPwmTime = 0;

void loop()
{
  static unsigned long lastFirebaseRead = 0;
  unsigned long now = millis();

  // Read from Firebase every 500ms (not every loop)
  if (now - lastFirebaseRead >= 500) {
    readDataFromFirebase();
    lastFirebaseRead = now;
  }

  // Send PWM every 20ms
  if (now - lastPwmTime >= 20) {
    lastPwmTime = now;

    digitalWrite(Relay_1, switch1 ? HIGH : LOW);
    digitalWrite(Relay_2, switch2 ? HIGH : LOW);

    // Map 0–100 to 1000–2000 µs
    int escPulse = map(PWMSignal, 0, 100, 1000, 2000);
    esc.writeMicroseconds(escPulse);
    esc.update();

    Serial.print("PWMSignal: ");
    Serial.print(PWMSignal);
    Serial.print(" | ESC Pulse (µs): ");
    Serial.println(escPulse);
  }
}