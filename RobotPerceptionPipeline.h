#pragma once

#include "Common/Pose2D.h"

namespace RobotPerceptionPipeline
{
  void Setup();
  void Update(
    unsigned long nowMs,
    const Pose2D& robotPose,
    float leftWheelLinearSpeedCmPerSec,
    float rightWheelLinearSpeedCmPerSec);
}
