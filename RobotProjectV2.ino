#include "RobotDriveControl.h"
#include "RobotPerceptionPipeline.h"
#include "RobotPoseTracking.h"
#include "Config.h"
#include "RemotePause.h"

namespace
{
  RemotePause remotePause(WifiConfig::Ssid, WifiConfig::Password);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("RobotProjectV2 snapshot sender starting");

  RobotPoseTracking::Setup();
  remotePause.begin();
  RobotDriveControl::Setup();
  RobotPerceptionPipeline::Setup();
}

void loop()
{
  const unsigned long now = millis();

  remotePause.update();
  RobotPoseTracking::Update(now);
  RobotDriveControl::Update(now, remotePause.isConnected() && !remotePause.isPaused());
  RobotPerceptionPipeline::Update(
    now,
    RobotPoseTracking::GetPose(),
    RobotPoseTracking::GetLeftWheelLinearSpeedCmPerSec(),
    RobotPoseTracking::GetRightWheelLinearSpeedCmPerSec());

  delay(10);
}
