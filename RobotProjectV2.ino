#include "RobotPerceptionPipeline.h"
#include "RobotPoseTracking.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("RobotProjectV2 snapshot sender starting");

  RobotPoseTracking::Setup();
  RobotPerceptionPipeline::Setup();
}

void loop()
{
  const unsigned long now = millis();

  RobotPoseTracking::Update(now);
  RobotPerceptionPipeline::Update(
    now,
    RobotPoseTracking::GetPose(),
    RobotPoseTracking::GetLeftWheelLinearSpeedCmPerSec(),
    RobotPoseTracking::GetRightWheelLinearSpeedCmPerSec());

  delay(10);
}
