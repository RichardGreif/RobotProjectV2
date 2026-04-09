#pragma once

#include "Measurement.h"
#include <array>
#include <utility>


namespace SLAM
{
    struct WallSegment
    {
        Vec2 start;
        Vec2 end;
        float confidence;

        Vec2 GetNormal() const
        {
            Vec2 direction = end - start;
            return direction.cross(confidence).normalized();
        }

        Vec2 GetCenter() const
        {
            return (start + end) * 0.5f;
        }
    };

    class WallMap
    {
    public:
        void AddObservation(const WallPoint& observation);
        static constexpr int MaxSegments = 32; // TODO: move to some config

        int GetCount() const { return count_; }
        const WallSegment& GetSegment(int index) const { return segments_[index]; }

    private:
        std::array<WallSegment, MaxSegments> segments_;
        int count_ = 0;

        void MergeObservationIntoSegment(const WallPoint& observation, WallSegment& segment);
        void CreateNewSegmentFromObservation(const WallPoint& observation);

        void CombineSegments(int firstIndex, int secondIndex);

        std::pair<int, int> FindTwoMatchingSegments(const WallPoint& observation) const;

        void RemoveIndex(int index);

    };

}
