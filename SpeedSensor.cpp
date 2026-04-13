#include "SpeedSensor.h"
#include <Arduino.h>

SpeedSensor* SpeedSensor::_instance = nullptr;

SpeedSensor::SpeedSensor(int pinLeft, int pinRight)
  : _pinLeft(pinLeft),
    _pinRight(pinRight),
    _leftPhaseCount(0),
    _rightPhaseCount(0),
    _leftTotalPhaseCount(0),
    _rightTotalPhaseCount(0),
    _leftPhaseTimeUs(0),
    _rightPhaseTimeUs(0),
    _lastStateLeft(HIGH),
    _lastStateRight(HIGH),
    _lastChangeLeftUs(0),
    _lastChangeRightUs(0),
    _leftSpeedHz(0.0f),
    _rightSpeedHz(0.0f),
    _lastUpdateMs(0) {
}

void SpeedSensor::begin() {
  _instance = this;

  pinMode(_pinLeft, INPUT);
  pinMode(_pinRight, INPUT);

  _lastStateLeft = digitalRead(_pinLeft);
  _lastStateRight = digitalRead(_pinRight);

  _lastChangeLeftUs = micros();
  _lastChangeRightUs = _lastChangeLeftUs;

  attachInterrupt(digitalPinToInterrupt(_pinLeft), handleLeftInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinRight), handleRightInterrupt, CHANGE);
}

void SpeedSensor::reset() {
  noInterrupts();
  _leftPhaseCount = 0;
  _rightPhaseCount = 0;
  _leftTotalPhaseCount = 0;
  _rightTotalPhaseCount = 0;
  _leftPhaseTimeUs = 0;
  _rightPhaseTimeUs = 0;

  _lastStateLeft = digitalRead(_pinLeft);
  _lastStateRight = digitalRead(_pinRight);

  _lastChangeLeftUs = micros();
  _lastChangeRightUs = _lastChangeLeftUs;
  interrupts();
}

void SpeedSensor::update(unsigned long currentTimeMs) {
  const unsigned long sampleTimeMs = 100;

  if (currentTimeMs - _lastUpdateMs < sampleTimeMs) {
    return;
  }

  noInterrupts();
  unsigned long countL = _leftPhaseCount;
  unsigned long countR = _rightPhaseCount;
  unsigned long timeL = _leftPhaseTimeUs;
  unsigned long timeR = _rightPhaseTimeUs;

  _leftPhaseCount = 0;
  _rightPhaseCount = 0;
  _leftPhaseTimeUs = 0;
  _rightPhaseTimeUs = 0;

  _lastChangeLeftUs = micros();
  _lastChangeRightUs = _lastChangeLeftUs;
  interrupts();

  if (countL >= 2 && timeL > 0) {
    _leftSpeedHz = (countL * 1000000.0f) / (2.0f * timeL);
  } else {
    _leftSpeedHz = 0.0f;
  }

  if (countR >= 2 && timeR > 0) {
    _rightSpeedHz = (countR * 1000000.0f) / (2.0f * timeR);
  } else {
    _rightSpeedHz = 0.0f;
  }

  _lastUpdateMs = currentTimeMs;
}

float SpeedSensor::getLeftSpeedHz() const {
  return _leftSpeedHz;
}

float SpeedSensor::getRightSpeedHz() const {
  return _rightSpeedHz;
}

unsigned long SpeedSensor::getLeftTotalPhaseCount() const {
  noInterrupts();
  const unsigned long value = _leftTotalPhaseCount;
  interrupts();
  return value;
}

unsigned long SpeedSensor::getRightTotalPhaseCount() const {
  noInterrupts();
  const unsigned long value = _rightTotalPhaseCount;
  interrupts();
  return value;
}

void IRAM_ATTR SpeedSensor::handleLeftInterrupt() {
  if (_instance) {
    _instance->onLeftChange();
  }
}

void IRAM_ATTR SpeedSensor::handleRightInterrupt() {
  if (_instance) {
    _instance->onRightChange();
  }
}

void SpeedSensor::onLeftChange() {
  unsigned long now = micros();
  int newState = digitalRead(_pinLeft);

  if (newState != _lastStateLeft) {
    _leftPhaseTimeUs += now - _lastChangeLeftUs;
    _leftPhaseCount++;
    _leftTotalPhaseCount++;

    _lastStateLeft = newState;
    _lastChangeLeftUs = now;
  }
}

void SpeedSensor::onRightChange() {
  unsigned long now = micros();
  int newState = digitalRead(_pinRight);

  if (newState != _lastStateRight) {
    _rightPhaseTimeUs += now - _lastChangeRightUs;
    _rightPhaseCount++;
    _rightTotalPhaseCount++;

    _lastStateRight = newState;
    _lastChangeRightUs = now;
  }
}
