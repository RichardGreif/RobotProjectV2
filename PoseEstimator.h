#pragma once

#include "Common/Pose2D.h"

class WheelOdometry;
class ImuTracker;

class PoseEstimator
{
public:
  void Reset(const Pose2D& initialPose);
  void UpdateFromSensors(const WheelOdometry& wheelOdometry, const ImuTracker& imuTracker);

  const Pose2D& GetPose() const;

private:
  Pose2D pose_{};
};
