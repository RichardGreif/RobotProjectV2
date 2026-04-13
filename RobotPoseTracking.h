#pragma once

#include "Common/Pose2D.h"

namespace RobotPoseTracking
{
  void Setup();
  void Update(unsigned long nowMs);

  const Pose2D& GetPose();
  float GetLeftWheelLinearSpeedCmPerSec();
  float GetRightWheelLinearSpeedCmPerSec();
}
