#pragma once
#include <Arduino.h>

class MotorDriver;
class SpeedSensor;

static constexpr int MinEffectivePwmPercent = 0; // Check real value
static constexpr int MaxPwmPercent = 100;

class DriveController {
public:
  DriveController(MotorDriver& motorDriver, SpeedSensor& speedSensor);

  void begin();
  void update(unsigned long currentTimeMs);

  void setTargetSpeeds(float leftSpeedHz, float rightSpeedHz);
  void stop();

  float getTargetLeftSpeedHz() const;
  float getTargetRightSpeedHz() const;

  float getMeasuredLeftSpeedHz() const;
  float getMeasuredRightSpeedHz() const;

  int getLeftPwmPercent() const;
  int getRightPwmPercent() const;

private:
  MotorDriver& _motorDriver;
  SpeedSensor& _speedSensor;

  float _targetLeftSpeedHz;
  float _targetRightSpeedHz;

  float _measuredLeftSpeedHz;
  float _measuredRightSpeedHz;

  int _leftPwmPercent;
  int _rightPwmPercent;

  unsigned long _lastControlUpdateMs;

  static constexpr unsigned long ControlIntervalMs = 10;
  static constexpr float SpeedControllerKp = 0.1f;
  static constexpr int MinPwmPercent = 0;
  static constexpr int MaxPwmPercent = 100;

  void updateMotorControl(float targetSpeedHz, float measuredSpeedHz, int& pwmPercent, bool isLeftMotor);
  int normalizePwmPercent(int pwmPercent) const;
  int pwmToMotor(int pwmPercent) const;
};