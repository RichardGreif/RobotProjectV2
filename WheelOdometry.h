#pragma once

#include "Common/Pose2D.h"

class WheelOdometry
{
public:
  WheelOdometry(
    float wheelBaseCm,
    float leftWheelSpeedCmPerSecPerHz,
    float rightWheelSpeedCmPerSecPerHz,
    float leftDirectionSign,
    float rightDirectionSign);

  void Reset(const Pose2D& pose, unsigned long nowMs);
  void Update(unsigned long nowMs, float leftSpeedHz, float rightSpeedHz);

  const Pose2D& GetPose() const;
  float GetLeftLinearSpeedCmPerSec() const;
  float GetRightLinearSpeedCmPerSec() const;

private:
  float wheelBaseCm_ = 0.0f;
  float leftWheelSpeedCmPerSecPerHz_ = 0.0f;
  float rightWheelSpeedCmPerSecPerHz_ = 0.0f;
  float leftDirectionSign_ = 1.0f;
  float rightDirectionSign_ = 1.0f;

  Pose2D pose_{};
  unsigned long lastUpdateMs_ = 0;
  bool initialized_ = false;

  float leftLinearSpeedCmPerSec_ = 0.0f;
  float rightLinearSpeedCmPerSec_ = 0.0f;
};
