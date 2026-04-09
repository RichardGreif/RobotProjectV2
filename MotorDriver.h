#pragma once
#include <Arduino.h>

class MotorDriver {
public:
  MotorDriver(
    int pwmA,
    int in1,
    int in2,
    int pwmB,
    int bin1,
    int bin2,
    bool leftForwardHigh,
    bool rightForwardHigh,
    int pwmFrequency,
    int pwmResolution);

  void begin();
  void stop();
  void setLeft(bool forward, int speedPercent);
  void setRight(bool forward, int speedPercent);

private:
  int _pwmA;
  int _in1;
  int _in2;
  int _pwmB;
  int _bin1;
  int _bin2;
  bool _leftForwardHigh;
  bool _rightForwardHigh;
  int _pwmFrequency;
  int _pwmResolution;

  void setDirection(int in1, int in2, bool forwardHigh, bool forward);
  int percentToDuty(int speedPercent) const;
};