#pragma once
#include <Arduino.h>

class UltrasonicArray {
public:
  UltrasonicArray(
    int trigLeft,
    int echoLeft,
    int trigCenter,
    int echoCenter,
    int trigRight,
    int echoRight);

  void begin();
  void update(unsigned long currentTimeMs);

  float getLeftDistanceCm() const;
  float getCenterDistanceCm() const;
  float getRightDistanceCm() const;

private:
  int _trigLeft;
  int _echoLeft;
  int _trigCenter;
  int _echoCenter;
  int _trigRight;
  int _echoRight;

  float _leftDistanceCm;
  float _centerDistanceCm;
  float _rightDistanceCm;

  unsigned long _lastMeasurementMs;
  int _nextSensorIndex;

  float measureDistanceCm(int trigPin, int echoPin) const;
};