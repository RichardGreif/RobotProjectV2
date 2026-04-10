#include "WallMap.h"

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace SLAM
{
    constexpr float angleThreshold_ = 0.261799f; // 15 degrees in radians
    constexpr float normalDistanceThreshold_ = 1.0f; // in cm
    constexpr float lateralOverflowThreshold_ = 2.0f; // in cm beyond segment endpoints
    constexpr float threshold = 0.01f; // minimal confidence squared to consider
    constexpr float initialSegmentHalfLength = 5.0f; // in cm

#ifdef ARDUINO
    constexpr bool wallMapDebugEnabled = true;

    void PrintObservation(const WallPoint& observation)
    {
        if (!wallMapDebugEnabled)
        {
            return;
        }

        Serial.print("obs point=(");
        Serial.print(observation.point.x, 3);
        Serial.print(",");
        Serial.print(observation.point.y, 3);
        Serial.print(") normalLen=");
        Serial.print(observation.normal.length(), 5);
    }
#endif

    void WallMap::CreateNewSegmentFromObservation(const WallPoint& observation)
    {
        if (count_ >= MaxSegments)
        {
            return;
        }

        const Vec2 normal = observation.normal.normalized();
        const Vec2 tangent = normal.cross(1.0f).normalized();
        const Vec2 halfExtent = tangent * initialSegmentHalfLength;

        segments_[count_] = {
            observation.point - halfExtent,
            observation.point + halfExtent,
            observation.normal.length()
        };

#ifdef ARDUINO
        if (wallMapDebugEnabled)
        {
            Serial.print("[WallMap] create segment idx=");
            Serial.print(count_);
            Serial.print(" ");
            PrintObservation(observation);
            Serial.print(" start=(");
            Serial.print(segments_[count_].start.x, 3);
            Serial.print(",");
            Serial.print(segments_[count_].start.y, 3);
            Serial.print(") end=(");
            Serial.print(segments_[count_].end.x, 3);
            Serial.print(",");
            Serial.print(segments_[count_].end.y, 3);
            Serial.println(")");
        }
#endif

        ++count_;
    }

    void WallMap::MergeObservationIntoSegment(const WallPoint& observation, WallSegment& segment)
    {
        const Vec2 wallVector = segment.end - segment.start;
        const float wallLength = wallVector.length();
        if (wallLength <= 1e-6f)
        {
            return;
        }

        const Vec2 direction = wallVector / wallLength;

        float startProjection = segment.start.dot(direction);
        float endProjection = segment.end.dot(direction);
        float observationProjection = observation.point.dot(direction);

        if (observationProjection < startProjection)
        {
            segment.start = observation.point;
        }

        if (observationProjection > endProjection)
        {
            segment.end = observation.point;
        }

        segment.confidence += observation.normal.length();

#ifdef ARDUINO
        if (wallMapDebugEnabled)
        {
            Serial.print("[WallMap] extend segment ");
            PrintObservation(observation);
            Serial.print(" newStart=(");
            Serial.print(segment.start.x, 3);
            Serial.print(",");
            Serial.print(segment.start.y, 3);
            Serial.print(") newEnd=(");
            Serial.print(segment.end.x, 3);
            Serial.print(",");
            Serial.print(segment.end.y, 3);
            Serial.print(") conf=");
            Serial.println(segment.confidence, 5);
        }
#endif
    }

    void WallMap::AddObservation(const WallPoint& observation)
    {
        if (observation.normal.squared() < threshold)
        {
            return;
        }

        auto [firstIndex, secondIndex] = FindTwoMatchingSegments(observation);

        if (firstIndex < 0)
        {
#ifdef ARDUINO
            if (wallMapDebugEnabled)
            {
                float bestAngleError = 1000.0f;
                float bestNormalDistance = 1000.0f;
                float bestParallelDistance = 1000.0f;
                int bestIndex = -1;

                const Vec2 observationNormal = observation.normal.normalized();

                for (int i = 0; i < count_; ++i)
                {
                    const WallSegment& segment = segments_[i];
                    const Vec2 wallVector = segment.end - segment.start;
                    const float wallLength = wallVector.length();
                    if (wallLength <= 1e-6f)
                    {
                        continue;
                    }

                    const Vec2 segmentNormal = segment.GetNormal();
                    const float angleError = std::abs(signedAngle(observationNormal, segmentNormal));

                    const float segmentNormalOffset = segment.GetCenter().dot(segmentNormal);
                    const float observationNormalOffset = observation.point.dot(segmentNormal);
                    const float normalDistance = std::abs(observationNormalOffset - segmentNormalOffset);

                    const Vec2 direction = wallVector / wallLength;
                    float projection = (observation.point - segment.start).dot(direction);
                    if (projection < 0.0f)
                    {
                        projection = -projection;
                    }
                    else if (projection > wallLength)
                    {
                        projection -= wallLength;
                    }
                    else
                    {
                        projection = 0.0f;
                    }

                    if (bestIndex < 0 || normalDistance < bestNormalDistance)
                    {
                        bestIndex = i;
                        bestAngleError = angleError;
                        bestNormalDistance = normalDistance;
                        bestParallelDistance = projection;
                    }
                }

                Serial.print("[WallMap] no match -> new segment ");
                PrintObservation(observation);
                Serial.print(" bestIdx=");
                Serial.print(bestIndex);
                Serial.print(" bestAngle=");
                Serial.print(bestAngleError, 5);
                Serial.print(" bestNormalDist=");
                Serial.print(bestNormalDistance, 5);
                Serial.print(" bestParallelOverflow=");
                Serial.println(bestParallelDistance, 5);
            }
#endif
            CreateNewSegmentFromObservation(observation);
            return;
        }

#ifdef ARDUINO
        if (wallMapDebugEnabled)
        {
            Serial.print("[WallMap] matched segment first=");
            Serial.print(firstIndex);
            Serial.print(" second=");
            Serial.print(secondIndex);
            Serial.print(" ");
            PrintObservation(observation);
            Serial.println();
        }
#endif

        MergeObservationIntoSegment(observation, segments_[firstIndex]);

        if (secondIndex >= 0)
        {
            CombineSegments(firstIndex, secondIndex);
        }
    }

    std::pair<int, int> WallMap::FindTwoMatchingSegments(const WallPoint& observation) const
    {
        int firstIndex = -1;
        int secondIndex = -1;

        const Vec2 observationNormal = observation.normal.normalized();

        for (int i = 0; i < count_; ++i)
        {
            const WallSegment& segment = segments_[i];
            const Vec2 wallVector = segment.end - segment.start;
            const float wallLength = wallVector.length();

#ifdef ARDUINO
            if (wallMapDebugEnabled)
            {
                Serial.print("[WallMap] check seg=");
                Serial.print(i);
                Serial.print(" segStart=(");
                Serial.print(segment.start.x, 3);
                Serial.print(",");
                Serial.print(segment.start.y, 3);
                Serial.print(") segEnd=(");
                Serial.print(segment.end.x, 3);
                Serial.print(",");
                Serial.print(segment.end.y, 3);
                Serial.print(") segLen=");
                Serial.print(wallLength, 5);
                Serial.print(" ");
                PrintObservation(observation);
            }
#endif

            if (wallLength <= 1e-6f)
            {
#ifdef ARDUINO
                if (wallMapDebugEnabled)
                {
                    Serial.println(" -> reject degenerate segment");
                }
#endif
                continue;
            }

            const Vec2 segmentNormal = segment.GetNormal();

            const float angleError = std::abs(signedAngle(observationNormal, segmentNormal));
            if (angleError > angleThreshold_)
            {
#ifdef ARDUINO
                if (wallMapDebugEnabled)
                {
                    Serial.print(" -> reject angle angleError=");
                    Serial.print(angleError, 5);
                    Serial.print(" threshold=");
                    Serial.println(angleThreshold_, 5);
                }
#endif
                continue;
            }

            const float segmentNormalOffset = segment.GetCenter().dot(segmentNormal);
            const float observationNormalOffset = observation.point.dot(segmentNormal);
            const float normalDistance = std::abs(observationNormalOffset - segmentNormalOffset);
            if (normalDistance > normalDistanceThreshold_)
            {
#ifdef ARDUINO
                if (wallMapDebugEnabled)
                {
                    Serial.print(" -> reject normalDist normalDistance=");
                    Serial.print(normalDistance, 5);
                    Serial.print(" threshold=");
                    Serial.print(normalDistanceThreshold_, 5);
                    Serial.print(" segOffset=");
                    Serial.print(segmentNormalOffset, 5);
                    Serial.print(" obsOffset=");
                    Serial.println(observationNormalOffset, 5);
                }
#endif
                continue;
            }

            const float toPointParallel = (observation.point - segment.start).dot(segment.end - segment.start);
            const float segmentLengthSquared = wallVector.squared();
            const float lateralThreshold = lateralOverflowThreshold_ / wallLength;
            const float minParallel = -segmentLengthSquared * lateralThreshold;
            const float maxParallel = segmentLengthSquared * (1 + lateralThreshold);

            if (toPointParallel < minParallel || toPointParallel > maxParallel)
            {
#ifdef ARDUINO
                if (wallMapDebugEnabled)
                {
                    Serial.print(" -> reject parallel toPointParallel=");
                    Serial.print(toPointParallel, 5);
                    Serial.print(" min=");
                    Serial.print(minParallel, 5);
                    Serial.print(" max=");
                    Serial.println(maxParallel, 5);
                }
#endif
                continue;
            }

#ifdef ARDUINO
            if (wallMapDebugEnabled)
            {
                Serial.print(" -> match angle=");
                Serial.print(angleError, 5);
                Serial.print(" normalDistance=");
                Serial.print(normalDistance, 5);
                Serial.print(" toPointParallel=");
                Serial.println(toPointParallel, 5);
            }
#endif

            if (firstIndex == -1)
            {
                firstIndex = i;
            }
            else
            {
                secondIndex = i;
                break;
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

        Vec2 start = (first.end - first.start).dot(second.start - first.start) > 0 ? first.start : second.start;
        Vec2 end = (first.start - first.end).dot(second.end - first.end) > 0 ? second.end : first.end;

        WallSegment newSegment{ start, end, first.confidence + second.confidence };

        segments_[firstIndex] = newSegment;
        RemoveIndex(secondIndex);
    }
}
