#pragma once
#include <new>
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

	/* 생성자 */
	Vector(float X = 0.f) :Vector(X, X, X) {}
	Vector(float X, float Y, float Z, float W = 1.f) : x(X), y(Y), z(Z), w{ W } {}

	Vector operator-()const { return Vector{ -x,-y,-z }; }

	/* 벡터를 정규화한다 */
	void normalize();

	/* 벡터의 크기를 반환한다 */
	float magnitude() const;
	/* 벡터 크기의 제곱을 반환한다 */
	float MagnitudeSq() const;

	/* 주어진 벡터와의 내적을 반환한다 */
	float dot(const Vector& other) const;

	/* 주어진 벡터와의 외적을 반환한다 */
	Vector cross(const Vector& other) const;

	/* 벡터를 0 으로 초기화한다 */
	void clear();

	/*****************
	 * 연산자 오버로딩 *
	 *****************/

	 //temporary thing <- should override all new(s) & delete(s)
	void* operator new(size_t sz);
	void operator delete(void* ptr);

	bool operator==(const Vector& rhs) const;

	/* 벡터끼리 더하기 */
	Vector operator+(const Vector& other) const;
	void operator+=(const Vector& other);

	/* 벡터끼리 빼기 */
	Vector operator-(const Vector& other) const;
	void operator-=(const Vector& other);

	/* 벡터와 스칼라끼리 곱하기 */
	Vector operator*(const float value) const;
	void operator*=(const float value);

	/* 인덱스로 접근하기 */
	const float operator[](unsigned int idx) const;
};
