#pragma once

#include "Vec2.h"

namespace SLAM
{

    struct MeasurementPoint
    {
        Vec2 position;
        Vec2 direction;
    };

    struct WallPoint
    {
        Vec2 point;
        Vec2 normal; // Length encodes confidence
    };

    WallPoint CreateInitialWallPoint(const MeasurementPoint& measurement);
    WallPoint CalculateWallPoint(const MeasurementPoint& measurement, const WallPoint* previousWallPoint = nullptr);
    WallPoint CalculateNewEstimation(const WallPoint& currentWallPoint, const WallPoint& previousWallPoint);
}
