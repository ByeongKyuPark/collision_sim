#pragma once
#include "Vector.h"

namespace physics
{
    struct Matrix4
    {
        __declspec(align(16)) union
        {
            __declspec(align(16)) float entries[16] = { 0 };
            __declspec(align(16)) Vector v[4];
            __declspec(align(16)) float m[4][4];
        };

        Matrix4();
        Matrix4(float value);
        Matrix4(float v1, float v2, float v3, float v4);
        Matrix4(float i0, float i1, float i2, float i3, float i4, float i5, float i6, float i7, float i8, float i9, float i10, float i11, float i12, float i13, float i14, float i15);
        void setDiagonal(float value);
        Matrix4 transpose() const noexcept;
        Matrix4 inverse() const;
        void* operator new(size_t i);
        void operator delete(void* p);
        void* operator new[](size_t size);
        void operator delete[](void* p);

        Matrix4 operator+(const Matrix4& other) const;
        Matrix4& operator+=(const Matrix4& other);
        Matrix4 operator-(const Matrix4& other) const;
        Matrix4& operator-=(const Matrix4& other);
        Matrix4 operator*(const Matrix4& other) const;
        Vector operator*(const Vector& vec) const;
        Matrix4 operator*(const float value) const;
        Matrix4& operator*=(const float value);
        Matrix4& operator=(const Matrix4& other);
    };    
}