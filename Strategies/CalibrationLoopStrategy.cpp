#include "CalibrationLoopStrategy.h"

#include "../Config.h"

#include <Arduino.h>
#include <cmath>

namespace Strategies
{
  namespace
  {
    constexpr float kPi = 3.14159265f;
  }

  void CalibrationLoopStrategy::Reset(const DriveStrategyInput& input)
  {
    initialized_ = true;
    EnterDriveStraight(input);
  }

  DriveStrategyOutput CalibrationLoopStrategy::Update(const DriveStrategyInput& input)
  {
    if (!initialized_)
    {
      Reset(input);
    }

    switch (state_)
    {
      case State::DriveStraight:
      {
        const unsigned long leftDelta = input.leftTotalPhaseCount - leftStartCount_;
        const unsigned long rightDelta = input.rightTotalPhaseCount - rightStartCount_;
        const unsigned long averageDelta = (leftDelta + rightDelta) / 2;

        if (averageDelta >= StrategyConfig::StraightStepCount)
        {
          EnterWaitAfterStraight(input);
          return {};
        }

        return { StrategyConfig::StraightTargetSpeedHz, StrategyConfig::StraightTargetSpeedHz };
      }

      case State::WaitAfterStraight:
      {
        if (input.nowMs - waitStartMs_ >= StrategyConfig::WaitAfterStraightMs)
        {
          EnterTurnInPlace(input);
          return { StrategyConfig::TurnTargetSpeedHz, -StrategyConfig::TurnTargetSpeedHz };
        }

        return {};
      }

      case State::TurnInPlace:
      {
        const float deltaYaw = NormalizeAngle(input.pose.yaw - yawStartRad_);
        if (std::abs(deltaYaw) >= StrategyConfig::TurnAngleRad)
        {
          EnterDriveStraight(input);
          return { StrategyConfig::StraightTargetSpeedHz, StrategyConfig::StraightTargetSpeedHz };
        }

        return { StrategyConfig::TurnTargetSpeedHz, -StrategyConfig::TurnTargetSpeedHz };
      }
    }

    return {};
  }

  void CalibrationLoopStrategy::EnterDriveStraight(const DriveStrategyInput& input)
  {
    state_ = State::DriveStraight;
    leftStartCount_ = input.leftTotalPhaseCount;
    rightStartCount_ = input.rightTotalPhaseCount;
    Serial.println("[Strategy] Enter DriveStraight");
  }

  void CalibrationLoopStrategy::EnterWaitAfterStraight(const DriveStrategyInput& input)
  {
    state_ = State::WaitAfterStraight;
    waitStartMs_ = input.nowMs;
    Serial.println("[Strategy] Enter WaitAfterStraight");
  }

  void CalibrationLoopStrategy::EnterTurnInPlace(const DriveStrategyInput& input)
  {
    state_ = State::TurnInPlace;
    yawStartRad_ = input.pose.yaw;
    Serial.println("[Strategy] Enter TurnInPlace");
  }

  float CalibrationLoopStrategy::NormalizeAngle(float angleRad)
  {
    while (angleRad > kPi)
    {
      angleRad -= 2.0f * kPi;
    }

    while (angleRad < -kPi)
    {
      angleRad += 2.0f * kPi;
    }

    return angleRad;
  }
}
