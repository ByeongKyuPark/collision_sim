#pragma once

#include "Vector.h"

namespace physics
{
    struct Matrix3
    {
        float entries[9];

        Matrix3();
        Matrix3(float value);
        Matrix3(float v1, float v2, float v3);

        void SetDiagonal(float value);
        Matrix3 Transpose() const;
        Matrix3 Inverse() const;

        Matrix3 operator+(const Matrix3& other) const;
        Matrix3& operator+=(const Matrix3& other);
        Matrix3 operator-(const Matrix3& other) const;
        Matrix3& operator-=(const Matrix3& other);
        Matrix3 operator*(const Matrix3& other) const;
        Matrix3& operator*=(const Matrix3& other);
        Vector operator*(const Vector& vec) const;
        Matrix3 operator*(const float value) const;
        Matrix3& operator*=(const float value);
        Matrix3& operator=(const Matrix3& other);
    };    
}