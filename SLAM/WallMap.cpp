#include "WallMap.h"


namespace SLAM
{
    constexpr float angleThreshold_ = 0.261799f; // 15 degrees in radians
    constexpr float normalDistanceThreshold_ = 1.0f; // in cm, adjust based on expected noise
    constexpr float lateralOverflowThreshold_ = 2.0f; // distance in cm beyond the segment endpoints to still consider a match
    constexpr float threshold = 0.01f; // minimal confidence squared to consider

    void WallMap::AddObservation(const WallPoint& observation)
    {
        if (observation.normal.squared() < threshold)
            return;

        auto [firstIndex, secondIndex] = FindTwoMatchingSegments(observation);

        if (firstIndex <0)
        {
            CreateNewSegmentFromObservation(observation);
            return;
        }

        // Merge in first segment
        MergeObservationIntoSegment(observation, segments_[firstIndex]);

        if (secondIndex >= 0)
        {
            // Combine first and second segment into one segment
            CombineSegments(firstIndex, secondIndex);
        }
    }

    std::pair<int, int> WallMap::FindTwoMatchingSegments(const WallPoint& observation) const
    {
        int firstIndex = -1;
        int secondIndex = -1;

        Vec2 observationNormal = observation.normal.normalized();

        for (int i = 0; i < count_; ++i)
        {
            const WallSegment& segment = segments_[i];

            Vec2 segmentNormal = segment.GetNormal();

            // 1. Winkel prüfen
            float angleError = std::abs(signedAngle(observationNormal, segmentNormal));
            if (angleError > angleThreshold_)
                continue;

            // 2. Normalabstand prüfen
            float normalDistance = std::abs(segment.GetCenter().dot(segmentNormal));
            if (normalDistance > normalDistanceThreshold_)
                continue;

            // 3. Seitliche Abweichung prüfen
            float toPointParallel = (observation.point - segment.start).dot(segment.end - segment.start);
            Vec2 wallVector = segment.end - segment.start;
            float segmentLengthSquared = (wallVector).squared();
            float lateralThreshold = lateralOverflowThreshold_ / wallVector.length();

            if (toPointParallel < -segmentLengthSquared * lateralThreshold || toPointParallel > segmentLengthSquared * ( 1 + lateralThreshold))
                continue;

            // ✔ Match gefunden
            if (firstIndex == -1)
            {
                firstIndex = i;
            }
            else
            {
                secondIndex = i;
                break; // 🔥 wir brauchen nur zwei → sofort abbrechen
            }

        }

        return { firstIndex, secondIndex };
    }

    void WallMap::RemoveIndex(int index)
    {
        if (index < 0 || index >= count_)
        {
            return;
        }

        const int lastIndex = count_ - 1;

        if (index != lastIndex)
        {
            segments_[index] = segments_[lastIndex];
        }

        --count_;
    }

    void WallMap::CombineSegments(int firstIndex, int secondIndex)
    {
        if (firstIndex < 0 || firstIndex >= count_ || secondIndex < 0 || secondIndex >= count_ || firstIndex == secondIndex)
        {
            return;
        }
        

        WallSegment& first = segments_[firstIndex];
        WallSegment& second = segments_[secondIndex];

        // Keep it simple at first and just take the further start and end point.
        Vec2 start = (first.end - first.start).dot(second.start - first.start) > 0 ? first.start : second.start;
        Vec2 end = (first.start - first.end).dot(second.end - first.end) > 0 ? second.end : first.end; 
        
        WallSegment newSegment{ start, end, first.confidence + second.confidence };

        segments_[firstIndex] = newSegment;
        RemoveIndex(secondIndex);
    }
}