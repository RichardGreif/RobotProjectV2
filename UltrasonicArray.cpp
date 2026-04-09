#include "UltrasonicArray.h"
#include <Arduino.h>

UltrasonicArray::UltrasonicArray(
  int trigLeft,
  int echoLeft,
  int trigCenter,
  int echoCenter,
  int trigRight,
  int echoRight)
  : _trigLeft(trigLeft),
    _echoLeft(echoLeft),
    _trigCenter(trigCenter),
    _echoCenter(echoCenter),
    _trigRight(trigRight),
    _echoRight(echoRight),
    _leftDistanceCm(-1.0f),
    _centerDistanceCm(-1.0f),
    _rightDistanceCm(-1.0f),
    _lastMeasurementMs(0),
    _nextSensorIndex(0) {
}

void UltrasonicArray::begin() {
  pinMode(_trigLeft, OUTPUT);
  pinMode(_echoLeft, INPUT);

  pinMode(_trigCenter, OUTPUT);
  pinMode(_echoCenter, INPUT);

  pinMode(_trigRight, OUTPUT);
  pinMode(_echoRight, INPUT);

  digitalWrite(_trigLeft, LOW);
  digitalWrite(_trigCenter, LOW);
  digitalWrite(_trigRight, LOW);
}

void UltrasonicArray::update(unsigned long currentTimeMs) {
  const unsigned long measurementIntervalMs = 60;

  if (currentTimeMs - _lastMeasurementMs < measurementIntervalMs) {
    return;
  }

  if (_nextSensorIndex == 0) {
    _leftDistanceCm = measureDistanceCm(_trigLeft, _echoLeft);
  } else if (_nextSensorIndex == 1) {
    _centerDistanceCm = measureDistanceCm(_trigCenter, _echoCenter);
  } else {
    _rightDistanceCm = measureDistanceCm(_trigRight, _echoRight);
  }

  _nextSensorIndex++;
  if (_nextSensorIndex > 2) {
    _nextSensorIndex = 0;
  }

  _lastMeasurementMs = currentTimeMs;
}

float UltrasonicArray::getLeftDistanceCm() const {
  return _leftDistanceCm;
}

float UltrasonicArray::getCenterDistanceCm() const {
  return _centerDistanceCm;
}

float UltrasonicArray::getRightDistanceCm() const {
  return _rightDistanceCm;
}

float UltrasonicArray::measureDistanceCm(int trigPin, int echoPin) const {
  const unsigned long echoTimeoutUs = 30000;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, echoTimeoutUs); // TODO: Später umbauen, damit der main loop nicht blockiert wird

  if (duration == 0) {
    return -1.0f;
  }

  return duration * 0.0343f / 2.0f;
}