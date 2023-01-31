#include <iostream>
#include <cmath>
#include <cfloat>
#include <cassert>
#include <xmmintrin.h>
#include "Vector.h"

void Vector::normalize()
{
    float magnitudeInverse = 1.0f / magnitude();
    x *= magnitudeInverse;
    y *= magnitudeInverse;
    z *= magnitudeInverse;
}

float Vector::magnitude() const
{
    float sum = x*x + y*y + z*z;
    return sqrtf(sum);
}

float Vector::MagnitudeSq() const
{
    return x*x + y*y + z*z;
}

float Vector::dot(const Vector& other) const
{
    return x*other.x + y*other.y + z*other.z;
}

Vector Vector::cross(const Vector& other) const
{
    return Vector(
        y*other.z - z*other.y,
        -x*other.z + z*other.x,
        x*other.y - y*other.x
    );
}

void Vector::clear(){
    x = 0.f;
    y = 0.f;
    z = 0.f;
    w = 1.f;
}

Vector Vector::operator+(const Vector& other) const
{
    return Vector(x + other.x, y + other.y, z + other.z);
}

void Vector::operator+=(const Vector& other){
    x += other.x;
    y += other.y;
    z += other.z;
}

Vector Vector::operator-(const Vector& other) const
{
    return Vector(x - other.x, y - other.y, z - other.z);
}

void Vector::operator-=(const Vector& other)
{
    x -= other.x;
    y -= other.y;
    z -= other.z;
}

Vector Vector::operator*(const float value) const
{
    return Vector(x * value, y * value, z * value);
}

void Vector::operator*=(const float value)
{
    x *= value;
    y *= value;
    z *= value;
}

const float Vector::operator[](unsigned int idx) const
{
    switch (idx)
    {
    case 0:
        return x;

    case 1:
        return y;

    case 2:
        return z;
    
    default:
        std::cout << "Vector::operator[]::Out of index" << std::endl;
        return FLT_MAX;
    }
}

bool Vector::operator==(const Vector& rhs) const
{
    for (int i = 0; i < 4; ++i)
    {
        if (a[i] != rhs.a[i])
        {
            return false;
        }
    }

    return true;
}
void* Vector::operator new(size_t sz) {
    return _aligned_malloc(sz, 16);
}

void Vector::operator delete(void* ptr) {
    _aligned_free(ptr);
}