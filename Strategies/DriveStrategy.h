#pragma once

#include "../Common/Pose2D.h"

namespace Strategies
{
  struct DriveStrategyInput
  {
    unsigned long nowMs = 0;
    Pose2D pose{};
    unsigned long leftTotalPhaseCount = 0;
    unsigned long rightTotalPhaseCount = 0;
  };

  struct DriveStrategyOutput
  {
    float leftTargetSpeedHz = 0.0f;
    float rightTargetSpeedHz = 0.0f;
  };

  class DriveStrategy
  {
  public:
    virtual ~DriveStrategy() = default;

    virtual void Reset(const DriveStrategyInput& input) = 0;
    virtual DriveStrategyOutput Update(const DriveStrategyInput& input) = 0;
  };
}
