#pragma once

#include "DriveStrategy.h"

namespace Strategies
{
  class CalibrationLoopStrategy : public DriveStrategy
  {
  public:
    void Reset(const DriveStrategyInput& input) override;
    DriveStrategyOutput Update(const DriveStrategyInput& input) override;

  private:
    enum class State
    {
      DriveStraight,
      WaitAfterStraight,
      TurnInPlace
    };

    void EnterDriveStraight(const DriveStrategyInput& input);
    void EnterWaitAfterStraight(const DriveStrategyInput& input);
    void EnterTurnInPlace(const DriveStrategyInput& input);
    static float NormalizeAngle(float angleRad);

    State state_ = State::DriveStraight;
    unsigned long leftStartCount_ = 0;
    unsigned long rightStartCount_ = 0;
    unsigned long waitStartMs_ = 0;
    float yawStartRad_ = 0.0f;
    bool initialized_ = false;
  };
}
