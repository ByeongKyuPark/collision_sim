#pragma once

#include "Vector.h"
#include <opengl/glm/glm.hpp>

namespace physics
{
    struct Quaternion
    {
        float d;
        float a;
        float b;
        float c;

        Quaternion() : d(0.0f), a(0.0f), b(1.0f), c(0.0f) {}
        Quaternion(float D, float A, float B, float C)
            : d(D), a(A), b(B), c(C) {}
        Quaternion(float D, const Vector& vec)
            : d(D), a(vec.x), b(vec.y), c(vec.z) {}

        void Normalize();

        Quaternion rotateByScaledVector(const Vector& vec, const float scale) const;

        Quaternion operator+(const Quaternion& other) const;
        void operator+=(const Quaternion& other);
        Quaternion operator*(const Quaternion& other) const;
        void operator*=(const Quaternion& other);
        Quaternion operator*(const float value) const;
        void operator*=(const float value);

    };
	//https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
    inline Quaternion ConvertToQuaternion(const Vector& vec, float angle) noexcept
    {
        Quaternion quaternion;
        float s = glm::sin(angle * 0.5f);
        quaternion.d = glm::cos(angle * 0.5f);
        quaternion.a = vec.x * s;
        quaternion.b = vec.y * s;
        quaternion.c = vec.z * s;

        return quaternion;
    }
}
