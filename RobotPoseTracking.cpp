#include "RobotPoseTracking.h"

#include "Config.h"
#include "ImuTracker.h"
#include "PoseEstimator.h"
#include "SpeedSensor.h"
#include "WheelOdometry.h"

namespace
{
  SpeedSensor speedSensors(Pins::LeftSPEED, Pins::RightSPEED);

  Pose2D initialRobotPose{
    Vec2(SnapshotStreamConfig::InitialRobotXcm, SnapshotStreamConfig::InitialRobotYcm),
    SnapshotStreamConfig::InitialRobotYawRad
  };

  WheelOdometry wheelOdometry(
    OdometryConfig::WheelDistanceCm,
    OdometryConfig::LeftWheelSpeedCmPerSecPerHz,
    OdometryConfig::RightWheelSpeedCmPerSecPerHz,
    OdometryConfig::LeftDirectionSign,
    OdometryConfig::RightDirectionSign);

  ImuTracker imuTracker;
  PoseEstimator poseEstimator;
}

namespace RobotPoseTracking
{
  void Setup()
  {
    speedSensors.begin();
    imuTracker.Begin();

    const unsigned long now = millis();
    wheelOdometry.Reset(initialRobotPose, now);
    poseEstimator.Reset(initialRobotPose);
  }

  void Update(unsigned long nowMs)
  {
    speedSensors.update(nowMs);
    imuTracker.Update(nowMs);
    wheelOdometry.Update(nowMs, speedSensors.getLeftSpeedHz(), speedSensors.getRightSpeedHz());
    poseEstimator.UpdateFromSensors(wheelOdometry, imuTracker);
  }

  const Pose2D& GetPose()
  {
    return poseEstimator.GetPose();
  }

  float GetLeftWheelLinearSpeedCmPerSec()
  {
    return wheelOdometry.GetLeftLinearSpeedCmPerSec();
  }

  float GetRightWheelLinearSpeedCmPerSec()
  {
    return wheelOdometry.GetRightLinearSpeedCmPerSec();
  }
}
