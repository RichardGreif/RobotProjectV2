#include "Mapper.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace SLAM
{
#ifdef ARDUINO
    constexpr bool mapperDebugEnabled = true;
#endif

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

#ifdef ARDUINO
        if (mapperDebugEnabled)
        {
            Serial.print("[Mapper] sensor=");
            Serial.print(sensorIndex);
            Serial.print(" measPos=(");
            Serial.print(measurement.position.x, 3);
            Serial.print(",");
            Serial.print(measurement.position.y, 3);
            Serial.print(") measDir=(");
            Serial.print(measurement.direction.x, 3);
            Serial.print(",");
            Serial.print(measurement.direction.y, 3);
            Serial.print(") wallPoint=(");
            Serial.print(wallPoint.point.x, 3);
            Serial.print(",");
            Serial.print(wallPoint.point.y, 3);
            Serial.print(") normal=(");
            Serial.print(wallPoint.normal.x, 5);
            Serial.print(",");
            Serial.print(wallPoint.normal.y, 5);
            Serial.print(") normalLen=");
            Serial.print(wallPoint.normal.length(), 5);
            Serial.print(" threshold=");
            Serial.println(minObservationConfidence_, 5);
        }
#endif

        if (!ShouldAddToMap(wallPoint))
        {
#ifdef ARDUINO
            if (mapperDebugEnabled)
            {
                Serial.println("[Mapper] reject wallPoint because normalLen is below threshold");
            }
#endif
            return;
        }

#ifdef ARDUINO
        if (mapperDebugEnabled)
        {
            Serial.println("[Mapper] accept wallPoint -> add to map");
        }
#endif

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

    bool Mapper::HasEstimatorState(int sensorIndex) const
    {
        return IsValidSensorIndex(sensorIndex) && estimators_[sensorIndex].HasState();
    }

    const WallPoint& Mapper::GetEstimatorState(int sensorIndex) const
    {
        return estimators_[sensorIndex].GetState();
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
