#include <physics/quaternion.h>
#include <cmath>

using namespace physics;

void Quaternion::Normalize()
{
    float magnitude = d*d + a*a + b*b + c*c;
    /* 크기가 0 이면 회전각 0 으로 설정한다 */
    if (magnitude == 0.0f)
    {
        d = 1.0f;
        return;
    }
    magnitude = 1.0f / sqrtf(magnitude);

    d *= magnitude;
    a *= magnitude;
    b *= magnitude;
    c *= magnitude;
}

Quaternion Quaternion::rotateByScaledVector(const Vector& vec, const float scale) const
{
    return *this * Quaternion(0.0f, vec.x * scale, vec.y * scale, vec.z * scale);
}

Quaternion Quaternion::operator+(const Quaternion& other) const
{
    Quaternion result;

    result.d = d + other.d;
    result.a = a + other.a;
    result.b = b + other.b;
    result.c = c + other.c;

    return result;
}

void Quaternion::operator+=(const Quaternion& other)
{
    d += other.d;
    a += other.a;
    b += other.b;
    c += other.c;
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
    Quaternion result;
    
    result.d = d*other.d - a*other.a - b*other.b - c*other.c;
    result.a = d*other.a + a*other.d + b*other.c - c*other.b;
    result.b = d*other.b - a*other.c + b*other.d + c*other.a;
    result.c = d*other.c + a*other.b - b*other.a + c*other.d;

    return result;
}

void Quaternion::operator*=(const Quaternion& other)
{
    Quaternion result;
    
    result.d = d*other.d - a*other.a - b*other.b - c*other.c;
    result.a = d*other.a + a*other.d + b*other.c - c*other.b;
    result.b = d*other.b - a*other.c + b*other.d + c*other.a;
    result.c = d*other.c + a*other.b - b*other.a + c*other.d;

    *this = result;
}

Quaternion Quaternion::operator*(const float value) const
{
    Quaternion result;

    result.d = d * value;
    result.a = a * value;
    result.b = b * value;
    result.c = c * value;

    return result;
}

void Quaternion::operator*=(const float value)
{
    d *= value;
    a *= value;
    b *= value;
    c *= value;
}