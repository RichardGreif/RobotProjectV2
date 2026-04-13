#pragma once
#include <Arduino.h>

class SpeedSensor {
public:
  SpeedSensor(int pinLeft, int pinRight);

  void begin();
  void reset();

  void update(unsigned long currentTimeMs);

  float getLeftSpeedHz() const;
  float getRightSpeedHz() const;
  unsigned long getLeftTotalPhaseCount() const;
  unsigned long getRightTotalPhaseCount() const;

private:
  int _pinLeft;
  int _pinRight;

  volatile unsigned long _leftPhaseCount;
  volatile unsigned long _rightPhaseCount;
  volatile unsigned long _leftTotalPhaseCount;
  volatile unsigned long _rightTotalPhaseCount;

  volatile unsigned long _leftPhaseTimeUs;
  volatile unsigned long _rightPhaseTimeUs;

  volatile int _lastStateLeft;
  volatile int _lastStateRight;

  volatile unsigned long _lastChangeLeftUs;
  volatile unsigned long _lastChangeRightUs;

  float _leftSpeedHz;
  float _rightSpeedHz;

  unsigned long _lastUpdateMs;

  static void handleLeftInterrupt();
  static void handleRightInterrupt();

  void onLeftChange();
  void onRightChange();

  static SpeedSensor* _instance;
};
