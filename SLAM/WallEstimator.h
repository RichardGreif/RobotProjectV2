#pragma once

#include <optional>
#include "Measurement.h"
#include "Vec2.h"

namespace SLAM
{
    class WallEstimator
    {
    public:
        WallEstimator() = default;

        WallPoint Update(const MeasurementPoint& measurement);
        bool HasState() const;

        const WallPoint& GetState() const;

        void Reset();

    private:
        std::optional<WallPoint> currentState_;
    };
}
