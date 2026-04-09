#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(
  int pwmA,
  int in1,
  int in2,
  int pwmB,
  int bin1,
  int bin2,
  bool leftForwardHigh,
  bool rightForwardHigh,
  int pwmFrequency,
  int pwmResolution)
  : _pwmA(pwmA),
    _in1(in1),
    _in2(in2),
    _pwmB(pwmB),
    _bin1(bin1),
    _bin2(bin2),
    _leftForwardHigh(leftForwardHigh),
    _rightForwardHigh(rightForwardHigh),
    _pwmFrequency(pwmFrequency),
    _pwmResolution(pwmResolution) {
}

void MotorDriver::begin() {
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(_bin1, OUTPUT);
  pinMode(_bin2, OUTPUT);

  ledcAttach(_pwmA, _pwmFrequency, _pwmResolution);
  ledcAttach(_pwmB, _pwmFrequency, _pwmResolution);

  stop();
}

void MotorDriver::stop() {
  ledcWrite(_pwmA, 0);
  ledcWrite(_pwmB, 0);

  digitalWrite(_in1, LOW);
  digitalWrite(_in2, LOW);
  digitalWrite(_bin1, LOW);
  digitalWrite(_bin2, LOW);
}

void MotorDriver::setLeft(bool forward, int speedPercent) {
  setDirection(_in1, _in2, _leftForwardHigh, forward);
  ledcWrite(_pwmA, percentToDuty(speedPercent));
}

void MotorDriver::setRight(bool forward, int speedPercent) {
  setDirection(_bin1, _bin2, _rightForwardHigh, forward);
  ledcWrite(_pwmB, percentToDuty(speedPercent));
}

void MotorDriver::setDirection(int in1, int in2, bool forwardHigh, bool forward) {
  if (forward) {
    digitalWrite(in1, forwardHigh ? HIGH : LOW);
    digitalWrite(in2, forwardHigh ? LOW : HIGH);
    return;
  }

  digitalWrite(in1, forwardHigh ? LOW : HIGH);
  digitalWrite(in2, forwardHigh ? HIGH : LOW);
}

int MotorDriver::percentToDuty(int speedPercent) const {
  return speedPercent * 255 / 100;
}