#pragma once
#include <cmath>

struct Vec2;

inline float signedAngle(const Vec2& a, const Vec2& b);
inline Vec2 operator*(float scalar, const Vec2& vec);

struct Vec2
{
    float x = 0.0f;
    float y = 0.0f;

    Vec2() = default;
    Vec2(float xValue, float yValue) : x(xValue), y(yValue) {}

    Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator*(float scalar) const { return Vec2(x * scalar, y * scalar); }
    Vec2 operator/(float scalar) const { return Vec2(x / scalar, y / scalar); }

    Vec2& operator+=(const Vec2& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2 rotate(const Vec2& v, float angle) const
    {
        float c = cos(angle);
        float s = sin(angle);
        return {v.x * c - v.y * s, v.x * s + v.y * c};
    }

    float dot(const Vec2& other) const
    {
        return x * other.x + y * other.y;
    }

    
    float cross(const Vec2& other) const
    {
        return x * other.y - y * other.x;
    }

    Vec2 cross(float scalar) const
    // Returns a vector perpendicular to this vector, with length scaled by the scalar
    {
        return Vec2(scalar * y, -scalar * x);
    }

    float length() const
    {
        return std::sqrt(x * x + y * y);
    }

    float squared() const
    {
        return x * x + y * y;
    }

    Vec2 getParallelComponent(const Vec2& other) const
    {
        return (*this) * (this->dot(other) / this->dot(*this));
    }

    Vec2 normalized() const
    {
        float len = length();
        if (len == 0) return Vec2(0, 0);
        return Vec2(x / len, y / len);
    }
};

inline float signedAngle(const Vec2& a, const Vec2& b)
{
    return std::atan2(a.cross(b), a.dot(b));
}

inline Vec2 operator*(float scalar, const Vec2& vec)
{
    return Vec2(vec.x * scalar, vec.y * scalar);
}

