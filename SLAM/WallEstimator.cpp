#include "WallEstimator.h"

namespace SLAM
{
    WallPoint WallEstimator::Update(const MeasurementPoint& measurement)
    {
        if (!currentState_.has_value())
        {
            currentState_ = CreateInitialWallPoint(measurement);
            return currentState_.value();
        }

        const WallPoint& previous = currentState_.value();

        WallPoint currentWallPoint = CalculateWallPoint(measurement, previous);
        currentState_ = CalculateNewEstimation(currentWallPoint, previous);

        return currentState_.value();
    }

    const WallPoint& WallEstimator::GetState() const
    {
        return currentState_.value(); // bewusst value() statt * → klarer Fehler bei falscher Nutzung
    }

    void WallEstimator::Reset()
    {
        currentState_.reset();
    }
}