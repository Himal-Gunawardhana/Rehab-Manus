#ifndef ESP8266Servo_h
#define ESP8266Servo_h

#include <Arduino.h>

class ESP8266Servo {
public:
  ESP8266Servo();
  void attach(uint8_t pin);
  void writeMicroseconds(int value); // 1000–2000us
  void update(); // Call in loop() or timer every 20ms
private:
  uint8_t _pin;
  int _pulseWidth;
};

#endif