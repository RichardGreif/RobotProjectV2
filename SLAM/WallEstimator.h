#pragma once

#include <optional>
#include "Measurement.h"

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
