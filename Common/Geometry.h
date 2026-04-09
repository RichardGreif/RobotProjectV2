#pragma once

#include <cmath>
#include "Vec2.h"
#include "Pose2D.h"

namespace Geometry
{
    inline float Length(const Vec2& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    inline Vec2 Normalize(const Vec2& v)
    {
        const float len = Length(v);
        if (len <= 1e-6f)
        {
            return Vec2(0.0f, 0.0f);
        }

        return v / len;
    }

    inline float Dot(const Vec2& a, const Vec2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    inline float Cross(const Vec2& a, const Vec2& b)
    {
        return a.x * b.y - a.y * b.x;
    }

    inline Vec2 Rotate(const Vec2& v, float angle)
    {
        const float c = std::cos(angle);
        const float s = std::sin(angle);
        return Vec2(c * v.x - s * v.y, s * v.x + c * v.y);
    }

    inline Vec2 TransformToWorld(const Pose2D& pose, const Vec2& localPoint)
    {
        return pose.position + Rotate(localPoint, pose.yaw);
    }
}