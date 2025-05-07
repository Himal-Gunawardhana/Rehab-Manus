#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <Arduino.h>

// WiFi Credentials
const char *ssid = "HimalPixel";
const char *password = "abcdef11";

// Firebase Realtime Database Info
const char *firebaseHost = "iotbin-1f0c5-default-rtdb.asia-southeast1.firebasedatabase.app";
const char *firebaseAuth = "AIzaSyBSL2Kb6B8mO72SRatUrSoSfVyastAZuwo";

// Device Configuration
const String rehabManus = "rehab123";

// Sensor Pins
#define Relay_1 D1
#define Relay_2 D2
#define Motor_PWM D4

// Variables
bool switch1 = false;
bool switch2 = true;
int PWMSignal = 128;

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
  readDataFromFirebase();  // ✅ FIXED: Removed parameters

  // Control relays and motor based on Firebase data
  digitalWrite(Relay_1, switch1 ? HIGH : LOW);
  digitalWrite(Relay_2, switch2 ? HIGH : LOW);
  analogWrite(Motor_PWM, PWMSignal);

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

  Serial.println("Connected to WiFi!");
}

void readDataFromFirebase()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure(); // Use this to skip SSL certificate verification for testing

    // Create Firebase URL
    String url = String("https://") + firebaseHost + "/rehabManus/" + rehabManus + ".json?auth=" + firebaseAuth;

    // Send HTTP GET request
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