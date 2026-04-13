#pragma once

#include "Common/Pose2D.h"

class SpeedSensor;

namespace RobotPoseTracking
{
  void Setup();
  void Update(unsigned long nowMs);

  const Pose2D& GetPose();
  float GetLeftWheelLinearSpeedCmPerSec();
  float GetRightWheelLinearSpeedCmPerSec();
  unsigned long GetLeftTotalPhaseCount();
  unsigned long GetRightTotalPhaseCount();
  SpeedSensor& GetSpeedSensor();
}
