#include "DriveController.h"
#include "MotorDriver.h"
#include "SpeedSensor.h"

DriveController::DriveController(MotorDriver& motorDriver, SpeedSensor& speedSensor)
  : _motorDriver(motorDriver),
    _speedSensor(speedSensor),
    _targetLeftSpeedHz(0.0f),
    _targetRightSpeedHz(0.0f),
    _measuredLeftSpeedHz(0.0f),
    _measuredRightSpeedHz(0.0f),
    _leftPwmPercent(0),
    _rightPwmPercent(0),
    _lastControlUpdateMs(0) {
}

void DriveController::begin() {
  stop();
}

void DriveController::update(unsigned long currentTimeMs) {
  if (currentTimeMs - _lastControlUpdateMs < ControlIntervalMs) {
    return;
  }

  _measuredLeftSpeedHz = _speedSensor.getLeftSpeedHz();
  _measuredRightSpeedHz = _speedSensor.getRightSpeedHz();

  updateMotorControl(_targetLeftSpeedHz, _measuredLeftSpeedHz, _leftPwmPercent, true);
  updateMotorControl(_targetRightSpeedHz, _measuredRightSpeedHz, _rightPwmPercent, false);

  _lastControlUpdateMs = currentTimeMs;
}

void DriveController::setTargetSpeeds(float leftSpeedHz, float rightSpeedHz) {
  _targetLeftSpeedHz = leftSpeedHz;
  _targetRightSpeedHz = rightSpeedHz;
}

void DriveController::stop() {
  _targetLeftSpeedHz = 0.0f;
  _targetRightSpeedHz = 0.0f;
  _measuredLeftSpeedHz = 0.0f;
  _measuredRightSpeedHz = 0.0f;
  _leftPwmPercent = 0;
  _rightPwmPercent = 0;
  _motorDriver.stop();
}

float DriveController::getTargetLeftSpeedHz() const {
  return _targetLeftSpeedHz;
}

float DriveController::getTargetRightSpeedHz() const {
  return _targetRightSpeedHz;
}

float DriveController::getMeasuredLeftSpeedHz() const {
  return _measuredLeftSpeedHz;
}

float DriveController::getMeasuredRightSpeedHz() const {
  return _measuredRightSpeedHz;
}

int DriveController::getLeftPwmPercent() const {
  return _leftPwmPercent;
}

int DriveController::getRightPwmPercent() const {
  return _rightPwmPercent;
}

void DriveController::updateMotorControl(float targetSpeedHz, float measuredSpeedHz, int& pwmPercent, bool isLeftMotor) {
  if (targetSpeedHz == 0.0f) {
    pwmPercent = 0;

    if (isLeftMotor) {
      _motorDriver.setLeft(true, 0);
    } else {
      _motorDriver.setRight(true, 0);
    }

    return;
  }

  bool forward = targetSpeedHz > 0.0f;
  float targetSpeedAbs = forward ? targetSpeedHz : -targetSpeedHz;
  float measuredSpeedAbs = measuredSpeedHz >= 0.0f ? measuredSpeedHz : -measuredSpeedHz;

  float error = targetSpeedAbs - measuredSpeedAbs;
  pwmPercent += static_cast<int>(SpeedControllerKp * error); // TODO: PID Regler nicht nur Proportionalanteil
  pwmPercent = normalizePwmPercent(pwmPercent);

  if (isLeftMotor) {
    _motorDriver.setLeft(forward, pwmToMotor(pwmPercent));
  } else {
    _motorDriver.setRight(forward, pwmToMotor(pwmPercent));
  }
}

int DriveController::normalizePwmPercent(int pwmPercent) const {
  if (pwmPercent <= 0) {
    return 0;
  }

  if (pwmPercent >= MaxPwmPercent) {
    return MaxPwmPercent;
  }

  if (pwmPercent < MinPwmPercent) {
    return MinPwmPercent;
  }

  return pwmPercent;
}

int DriveController::pwmToMotor(int pwmPercent) const {
  if (pwmPercent <= 0) {
    return 0;
  }

  if (pwmPercent >= MaxPwmPercent) {
    return MaxPwmPercent;
  }

  if (pwmPercent < MinEffectivePwmPercent) {
    int distanceToZero = pwmPercent;
    int distanceToMin = MinEffectivePwmPercent - pwmPercent;

    if (distanceToZero <= distanceToMin) {
      return 0;
    }

    return MinEffectivePwmPercent;
  }

  return pwmPercent;
}