#include "PoseEstimator.h"

#include "ImuTracker.h"
#include "WheelOdometry.h"

void PoseEstimator::Reset(const Pose2D& initialPose) {
  pose_ = initialPose;
}

void PoseEstimator::UpdateFromSensors(const WheelOdometry& wheelOdometry, const ImuTracker&) {
  pose_ = wheelOdometry.GetPose();
}

const Pose2D& PoseEstimator::GetPose() const {
  return pose_;
}
