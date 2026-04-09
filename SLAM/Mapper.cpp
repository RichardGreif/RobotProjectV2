#include "Mapper.h"

namespace SLAM
{
    Mapper::Mapper(float minObservationConfidence)
        : minObservationConfidence_(minObservationConfidence)
    {
    }

    void Mapper::ProcessMeasurement(int sensorIndex, const MeasurementPoint& measurement)
    {
        if (!IsValidSensorIndex(sensorIndex))
        {
            return;
        }

        WallEstimator& estimator = estimators_[sensorIndex];
        WallPoint wallPoint = estimator.Update(measurement);

        if (!ShouldAddToMap(wallPoint))
        {
            return;
        }

        wallMap_.AddObservation(wallPoint);
    }

    void Mapper::Reset()
    {
        for (WallEstimator& estimator : estimators_)
        {
            estimator.Reset();
        }
    }

    const WallMap& Mapper::GetWallMap() const // const to print the map without modifying it
    {
        return wallMap_;
    }

    WallMap& Mapper::GetWallMap()
    {
        return wallMap_;
    }

    float Mapper::GetMinObservationConfidence() const
    {
        return minObservationConfidence_;
    }

    void Mapper::SetMinObservationConfidence(float value)
    {
        minObservationConfidence_ = value;
    }

    bool Mapper::IsValidSensorIndex(int sensorIndex) const
    {
        return sensorIndex >= 0 && sensorIndex < SensorCount;
    }

    bool Mapper::ShouldAddToMap(const WallPoint& wallPoint) const
    {
        return wallPoint.normal.length() >= minObservationConfidence_;
    }
}