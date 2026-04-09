#pragma once

#include <optional>
#include "Measurement.h"
#include "Vec2.h"

namespace SLAM
{
    struct WallPoint
    {
        Vec2 point;
        Vec2 normal;
    };

    WallPoint CalculateWallPoint(const MeasurementPoint& measurement, const WallPoint& previousWallPoint);
    WallPoint CalculateNewEstimation(const WallPoint& currentWallPoint, const WallPoint& previousWallPoint);
    WallPoint CreateInitialWallPoint(const MeasurementPoint& measurement);

    class WallEstimator
    {
    public:
        WallEstimator() = default;

        WallPoint Update(const MeasurementPoint& measurement);

        const WallPoint& GetState() const;

        void Reset();

    private:
        std::optional<WallPoint> currentState_;
    };
}