#include "Measurement.h"
#include "Vec2.h"

namespace SLAM
{
    constexpr float ANGLE_SPREAD = 0.261799f; // 15 degrees in radians, widening of the US sensor

    // [cm ^-2] Determines the maximal wallPoint normal. 0.05 -> roughly 10 cm maximal wall normal at 2 cm variation in measurement
    // Limits the maximal  wallPoint normal to 1/ ( normalizationFactor * US_precision))   
    // US_precision is around 3 cm based on the documentation
    constexpr float normalizationFactor = 0.04f;
    constexpr float initialConfidence = 0.5f; // Initial confidence for new wall points in cm

    WallPoint CalculateWallPoint(const MeasurementPoint& measurement, const WallPoint* previousWallPoint)
    {
        if (previousWallPoint == nullptr)
        {
            // If no previous wall point is available, use the measurement directly
            return { measurement.position + measurement.direction, measurement.direction.normalized() * initialConfidence };
        }

        // calculate point
        // calculate angle between measured direction and previous wall normal
        float angle = signedAngle(measurement.direction, previousWallPoint->normal);

        // if angle is <15 degrees, use previous wall normal
        Vec2 correctedDirection;
        if (std::abs(angle) < ANGLE_SPREAD)
        {
            correctedDirection = previousWallPoint->normal * measurement.direction.length() / previousWallPoint->normal.length();
        }
        else
        {
            // else calculate new normal by rotating measured direction by 15 degrees towards previous wall normal.
            correctedDirection = measurement.direction.rotate(measurement.direction, (angle > 0 ? -ANGLE_SPREAD : ANGLE_SPREAD));
        }   
        
        // point of the wall based on the measurement
        Vec2 point = measurement.position + correctedDirection;

        // normal of the wall based on the measurement
        Vec2 shift = (point - previousWallPoint->point );
        Vec2 normal = shift.cross(previousWallPoint->normal.cross(shift)) / shift.length() / previousWallPoint->normal.length(); 
        

        return { point, normal };
    }

    WallPoint CalculateNewEstimation(const WallPoint& currentWallPoint, const WallPoint& previousWallPoint)
    {
        // calculate new point as average of current and previous point weighted by their confidence (length of normal)
        float currentWeight = currentWallPoint.normal.length();
        float previousWeight = previousWallPoint.normal.length();
        float totalWeight = currentWeight + previousWeight;
        Vec2 newPoint = (currentWallPoint.point * currentWeight + previousWallPoint.point * previousWeight) / totalWeight;

        // calculate new normal as average of current and previous normal 
        Vec2 newNormal = (currentWallPoint.normal + previousWallPoint.normal) / (1 + normalizationFactor * (newPoint - previousWallPoint.point).squared()); 
        
        return { newPoint, newNormal };
    }
}