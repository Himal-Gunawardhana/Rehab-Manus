#include "ESP8266Servo.h"

ESP8266Servo::ESP8266Servo() {
  _pin = 255;
  _pulseWidth = 1500; // default neutral
}

void ESP8266Servo::attach(uint8_t pin) {
  _pin = pin;
  pinMode(_pin, OUTPUT);
}

void ESP8266Servo::writeMicroseconds(int value) {
  _pulseWidth = constrain(value, 1000, 2000);
}

void ESP8266Servo::update() {
    if (_pin == 255) return;
    digitalWrite(_pin, HIGH);
    delayMicroseconds(_pulseWidth);
    digitalWrite(_pin, LOW);
}