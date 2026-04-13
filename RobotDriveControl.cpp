#include "RobotDriveControl.h"

#include "Config.h"
#include "DriveController.h"
#include "MotorDriver.h"
#include "RobotPoseTracking.h"
#include "Strategies/CalibrationLoopStrategy.h"

namespace
{
  MotorDriver motors(
    Pins::PWMA,
    Pins::AIN1,
    Pins::AIN2,
    Pins::PWMB,
    Pins::BIN1,
    Pins::BIN2,
    MotorConfig::LeftForwardHigh,
    MotorConfig::RightForwardHigh,
    MotorConfig::PwmFrequency,
    MotorConfig::PwmResolution);

  DriveController* driveController = nullptr;
  Strategies::CalibrationLoopStrategy strategy;
}

namespace RobotDriveControl
{
  void Setup()
  {
    motors.begin();
    static DriveController driveControllerInstance(motors, RobotPoseTracking::GetSpeedSensor());
    driveController = &driveControllerInstance;
    driveController->begin();

    const Strategies::DriveStrategyInput input{
      millis(),
      RobotPoseTracking::GetPose(),
      RobotPoseTracking::GetLeftTotalPhaseCount(),
      RobotPoseTracking::GetRightTotalPhaseCount()
    };
    strategy.Reset(input);
  }

  void Update(unsigned long nowMs)
  {
    if (driveController == nullptr)
    {
      return;
    }

    const Strategies::DriveStrategyInput input{
      nowMs,
      RobotPoseTracking::GetPose(),
      RobotPoseTracking::GetLeftTotalPhaseCount(),
      RobotPoseTracking::GetRightTotalPhaseCount()
    };

    const Strategies::DriveStrategyOutput output = strategy.Update(input);
    driveController->setTargetSpeeds(output.leftTargetSpeedHz, output.rightTargetSpeedHz);
    driveController->update(nowMs);
  }
}
