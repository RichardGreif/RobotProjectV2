#pragma once

#include <array>
#include <optional>

#include "Measurement.h"
#include "WallEstimator.h"
#include "WallMap.h"

namespace SLAM
{
    class Mapper
    {
    public:
        static constexpr int SensorCount = 3;

        explicit Mapper(float minObservationConfidence);

        void ProcessMeasurement(int sensorIndex, const MeasurementPoint& measurement);

        void Reset();

        const WallMap& GetWallMap() const;
        WallMap& GetWallMap();

        bool HasEstimatorState(int sensorIndex) const;
        const WallPoint& GetEstimatorState(int sensorIndex) const;

        float GetMinObservationConfidence() const;
        void SetMinObservationConfidence(float value);

    private:
        bool IsValidSensorIndex(int sensorIndex) const;
        bool ShouldAddToMap(const WallPoint& wallPoint) const;

        std::array<WallEstimator, SensorCount> estimators_{};
        WallMap wallMap_{};
        float minObservationConfidence_ = 0.0f;
    };
}