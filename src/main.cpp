/*#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

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
#define Motor_PWM D4

// Control Variables
bool switch1 = false;
bool switch2 = true;
int PWMSignal = 0; // 0–100 from Firebase
int pulseWidth = 1500; // in microseconds

unsigned long lastPwmTime = 0;
const int pwmInterval = 20; // 20ms = 50Hz

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
    client.setInsecure(); // ⚠️ Skip SSL check (dev use)

    String url = String("https://") + firebaseHost + "/rehabManus/" + rehabManus + ".json?auth=" + firebaseAuth;

    http.begin(client, url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
      String payload = http.getString();
      Serial.println("Firebase: " + payload);

      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);

      if (!error)
      {
        switch1 = doc["switch1"] | false;
        switch2 = doc["switch2"] | false;
        PWMSignal = doc["PWMSignal"] | 0;
        PWMSignal = constrain(PWMSignal, 0, 100);

        // Convert 0–100 to 1000–2000 us
        pulseWidth = map(PWMSignal, 0, 100, 1000, 2000);
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
  pinMode(Motor_PWM, OUTPUT);

  Serial.println("ESC Ready.");
}

void sendPwmPulse()
{
  digitalWrite(Motor_PWM, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(Motor_PWM, LOW);
  // Remaining time of the 20ms cycle will be handled in loop delay
}

void loop()
{
  readDataFromFirebase();

  digitalWrite(Relay_1, switch1 ? HIGH : LOW);
  digitalWrite(Relay_2, switch2 ? HIGH : LOW);

  unsigned long currentMillis = millis();

  if (currentMillis - lastPwmTime >= pwmInterval)
  {
    lastPwmTime = currentMillis;
    sendPwmPulse();

    Serial.print("PWMSignal: ");
    Serial.print(PWMSignal);
    Serial.print(" | Pulse Width (µs): ");
    Serial.println(pulseWidth);
  }

  
}
*/

//Is this code works for this ESC's input D4

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Arduino.h>

// WiFi Credentials
const char *ssid = "HimalPixel";
const char *password = "abcdef11";

// Firebase Realtime Database Info
const char *firebaseHost = "stroke-rehab-arm-default-rtdb.asia-southeast1.firebasedatabase.app";
const char *firebaseAuth = "AIzaSyBfr5aBDUGnIKaOS62m9za4smyFWQae6tk";

// Device Configuration
const String rehabManus = "rehab123";

// Sensor Pins
#define Relay_1 D1
#define Relay_2 D2
#define Motor_PWM D4

// Variables
bool switch1 = false;
bool switch2 = true;
int PWMSignal = 0;

void connectToWiFi();
void readDataFromFirebase();

void setup()
{
  Serial.begin(115200);

  // Initialize WiFi
  connectToWiFi();

  // Initialize sensor pins
  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_2, OUTPUT);
  pinMode(Motor_PWM, OUTPUT);
}

void loop()
{
  // Read data from Firebase
  readDataFromFirebase();  

  // Control relays
  digitalWrite(Relay_1, switch1 ? HIGH : LOW);
  digitalWrite(Relay_2, switch2 ? HIGH : LOW);

  // Map 0-100 range to 0-1023 for ESP8266 PWM
  int pwmOutput = map(PWMSignal, 0, 100, 0, 1023);
  analogWrite(Motor_PWM, pwmOutput);

  // Debug output
  Serial.print("PWMSignal (0-100): ");
  Serial.print(PWMSignal);
  Serial.print(" | PWM Output (0-1023): ");
  Serial.println(pwmOutput);

  delay(100); // Adjust delay as needed
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

