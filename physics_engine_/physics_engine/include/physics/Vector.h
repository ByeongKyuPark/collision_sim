#pragma once
#include <new>
namespace physics
{
    struct __declspec(align((16))) Vector
    {
        __declspec(align(16)) union
        {
            __declspec(align(16)) float a[4] = { 0 };
            __declspec(align(16)) struct
            {
                float x;
                float y;
                float z;
                float w;
            };
        };

        Vector(float X=0.f) :Vector(X,X,X) {}
        Vector(float X, float Y, float Z, float W=1.f) : x(X), y(Y), z(Z), w{W} {}

        Vector operator-()const { return Vector{ -x,-y,-z }; }

        void normalize();
        float magnitude() const;
        float MagnitudeSq() const;

        float dot(const Vector& other) const;
        Vector cross(const Vector& other) const;
        void clear();

        void* operator new(size_t sz);
        void operator delete(void* ptr);

        bool operator==(const Vector& rhs) const;

        Vector operator+(const Vector& other) const;
        void operator+=(const Vector& other);

        Vector operator-(const Vector& other) const;
        void operator-=(const Vector& other);

        Vector operator*(const float value) const;
        void operator*=(const float value);
        const float operator[](unsigned int idx) const;
    };
}