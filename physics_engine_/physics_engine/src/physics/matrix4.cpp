#include <physics/matrix4.h>
#include <iostream>

#include <xmmintrin.h>

using namespace physics;


/*  Calculate multiplication between two matrices
 *  @param mat Matrix4
 *  @return A resultant matrix
 */
Matrix4 __stdcall Matrix4::operator*(const Matrix4& mat) const {
    Matrix4 temp{ this->transpose() };
    Matrix4 temp2{ mat.transpose() };
    Matrix4 result{ 0 };

    __m128 c1 = _mm_load_ps(&temp2.entries[0]);
    __m128 c2 = _mm_load_ps(&temp2.entries[4]);
    __m128 c3 = _mm_load_ps(&temp2.entries[8]);
    __m128 c4 = _mm_load_ps(&temp2.entries[12]);
    for (int i = 0; i < 4; i++) {
        __m128 v1 = _mm_set1_ps(mat.entries[4 * i + 0]);
        __m128 v2 = _mm_set1_ps(mat.entries[4 * i + 1]);
        __m128 v3 = _mm_set1_ps(mat.entries[4 * i + 2]);
        __m128 v4 = _mm_set1_ps(mat.entries[4 * i + 3]);
        __m128 elem = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(v1, c1), _mm_mul_ps(v2, c2)),
            _mm_add_ps(_mm_mul_ps(v3, c3), _mm_mul_ps(v4, c4))
        );
        _mm_store_ps(&result.entries[4 * i], elem);
    }
    return result.transpose();
}


/*  Calculate multiplication between a matrix & vector
 *  @param vec Vector4
 *  @return A resultant Vector
 */
Vector __stdcall Matrix4::operator*(const Vector& vec)const {
    Vector result{ 0.f };
    Matrix4 mat{ this->transpose() };
    __m128 v = _mm_load_ps(vec.a);

    __m128 v1 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0));
    __m128 v2 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1));
    __m128 v3 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2));
    __m128 v4 = _mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 3, 3, 3));

    __m128 c1 = _mm_load_ps(&mat.entries[0]);
    __m128 c2 = _mm_load_ps(&mat.entries[4]);
    __m128 c3 = _mm_load_ps(&mat.entries[8]);
    __m128 c4 = _mm_load_ps(&mat.entries[12]);

    __m128 p1 = _mm_mul_ps(v1, c1);
    __m128 p2 = _mm_mul_ps(v2, c2);
    __m128 p3 = _mm_mul_ps(v3, c3);
    __m128 p4 = _mm_mul_ps(v4, c4);

    _mm_store_ps(result.a, _mm_add_ps(_mm_add_ps(p1, p2), _mm_add_ps(p3, p4)));
    return result;
}
Matrix4::Matrix4()
{
    setDiagonal(1.0f);
}

Matrix4::Matrix4(float value)
{
    setDiagonal(value);
}

physics::Matrix4::Matrix4(float i0, float i1, float i2, float i3, float i4, float i5, float i6, float i7, float i8, float i9, float i10, float i11, float i12, float i13, float i14, float i15)
    :entries{i0,i1,i2,i3 ,i4 ,i5 ,i6 ,i7 ,i8 ,i9 ,i10 ,i11 ,i12 ,i13 ,i14 ,i15 }
{
}

Matrix4::Matrix4(float v1, float v2, float v3, float v4)
{
    entries[0] = v1;
    entries[1] = 0.0f;
    entries[2] = 0.0f;
    entries[3] = 0.0f;

    entries[4] = 0.0f;
    entries[5] = v2;
    entries[6] = 0.0f;
    entries[7] = 0.0f;

    entries[8] = 0.0f;
    entries[9] = 0.0f;
    entries[10] = v3;
    entries[11] = 0.0f;

    entries[12] = 0.0f;
    entries[13] = 0.0f;
    entries[14] = 0.0f;
    entries[15] = v4;
}

void Matrix4::setDiagonal(float value)
{
    entries[0] = value;
    entries[1] = 0.0f;
    entries[2] = 0.0f;
    entries[3] = 0.0f;

    entries[4] = 0.0f;
    entries[5] = value;
    entries[6] = 0.0f;
    entries[7] = 0.0f;

    entries[8] = 0.0f;
    entries[9] = 0.0f;
    entries[10] = value;
    entries[11] = 0.0f;

    entries[12] = 0.0f;
    entries[13] = 0.0f;
    entries[14] = 0.0f;
    entries[15] = value;
}

Matrix4 Matrix4::transpose() const noexcept
{
    return{ 
           entries[0], entries[4], entries[8],entries[12],
           entries[1],entries[5],entries[9],entries[13],
           entries[2],entries[6],entries[10],entries[14],
           entries[3],entries[7],entries[11],entries[15]
    };
}

Matrix4 Matrix4::inverse() const
{
    Matrix4 temp{ this->transpose() };
    float inv[16]{};

    inv[0] = temp.entries[5]  * temp.entries[10] * temp.entries[15] - 
             temp.entries[5]  * temp.entries[11] * temp.entries[14] - 
             temp.entries[9]  * temp.entries[6]  * temp.entries[15] + 
             temp.entries[9]  * temp.entries[7]  * temp.entries[14] +
             temp.entries[13] * temp.entries[6]  * temp.entries[11] - 
             temp.entries[13] * temp.entries[7]  * temp.entries[10];

    inv[4] = -temp.entries[4]  * temp.entries[10] * temp.entries[15] + 
              temp.entries[4]  * temp.entries[11] * temp.entries[14] + 
              temp.entries[8]  * temp.entries[6]  * temp.entries[15] - 
              temp.entries[8]  * temp.entries[7]  * temp.entries[14] - 
              temp.entries[12] * temp.entries[6]  * temp.entries[11] + 
              temp.entries[12] * temp.entries[7]  * temp.entries[10];

    inv[8] = temp.entries[4]  * temp.entries[9] * temp.entries[15] - 
             temp.entries[4]  * temp.entries[11] * temp.entries[13] - 
             temp.entries[8]  * temp.entries[5] * temp.entries[15] + 
             temp.entries[8]  * temp.entries[7] * temp.entries[13] + 
             temp.entries[12] * temp.entries[5] * temp.entries[11] - 
             temp.entries[12] * temp.entries[7] * temp.entries[9];

    inv[12] = -temp.entries[4]  * temp.entries[9] * temp.entries[14] + 
               temp.entries[4]  * temp.entries[10] * temp.entries[13] +
               temp.entries[8]  * temp.entries[5] * temp.entries[14] - 
               temp.entries[8]  * temp.entries[6] * temp.entries[13] - 
               temp.entries[12] * temp.entries[5] * temp.entries[10] + 
               temp.entries[12] * temp.entries[6] * temp.entries[9];

    inv[1] = -temp.entries[1]  * temp.entries[10] * temp.entries[15] + 
              temp.entries[1]  * temp.entries[11] * temp.entries[14] + 
              temp.entries[9]  * temp.entries[2] * temp.entries[15] - 
              temp.entries[9]  * temp.entries[3] * temp.entries[14] - 
              temp.entries[13] * temp.entries[2] * temp.entries[11] + 
              temp.entries[13] * temp.entries[3] * temp.entries[10];

    inv[5] = temp.entries[0]  * temp.entries[10] * temp.entries[15] - 
             temp.entries[0]  * temp.entries[11] * temp.entries[14] - 
             temp.entries[8]  * temp.entries[2] * temp.entries[15] + 
             temp.entries[8]  * temp.entries[3] * temp.entries[14] + 
             temp.entries[12] * temp.entries[2] * temp.entries[11] - 
             temp.entries[12] * temp.entries[3] * temp.entries[10];

    inv[9] = -temp.entries[0]  * temp.entries[9] * temp.entries[15] + 
              temp.entries[0]  * temp.entries[11] * temp.entries[13] + 
              temp.entries[8]  * temp.entries[1] * temp.entries[15] - 
              temp.entries[8]  * temp.entries[3] * temp.entries[13] - 
              temp.entries[12] * temp.entries[1] * temp.entries[11] + 
              temp.entries[12] * temp.entries[3] * temp.entries[9];

    inv[13] = temp.entries[0]  * temp.entries[9] * temp.entries[14] - 
              temp.entries[0]  * temp.entries[10] * temp.entries[13] - 
              temp.entries[8]  * temp.entries[1] * temp.entries[14] + 
              temp.entries[8]  * temp.entries[2] * temp.entries[13] + 
              temp.entries[12] * temp.entries[1] * temp.entries[10] - 
              temp.entries[12] * temp.entries[2] * temp.entries[9];

    inv[2] = temp.entries[1]  * temp.entries[6] * temp.entries[15] - 
             temp.entries[1]  * temp.entries[7] * temp.entries[14] - 
             temp.entries[5]  * temp.entries[2] * temp.entries[15] + 
             temp.entries[5]  * temp.entries[3] * temp.entries[14] + 
             temp.entries[13] * temp.entries[2] * temp.entries[7] - 
             temp.entries[13] * temp.entries[3] * temp.entries[6];

    inv[6] = -temp.entries[0]  * temp.entries[6] * temp.entries[15] + 
              temp.entries[0]  * temp.entries[7] * temp.entries[14] + 
              temp.entries[4]  * temp.entries[2] * temp.entries[15] - 
              temp.entries[4]  * temp.entries[3] * temp.entries[14] - 
              temp.entries[12] * temp.entries[2] * temp.entries[7] + 
              temp.entries[12] * temp.entries[3] * temp.entries[6];

    inv[10] = temp.entries[0]  * temp.entries[5] * temp.entries[15] - 
              temp.entries[0]  * temp.entries[7] * temp.entries[13] - 
              temp.entries[4]  * temp.entries[1] * temp.entries[15] + 
              temp.entries[4]  * temp.entries[3] * temp.entries[13] + 
              temp.entries[12] * temp.entries[1] * temp.entries[7] - 
              temp.entries[12] * temp.entries[3] * temp.entries[5];

    inv[14] = -temp.entries[0]  * temp.entries[5] * temp.entries[14] + 
               temp.entries[0]  * temp.entries[6] * temp.entries[13] + 
               temp.entries[4]  * temp.entries[1] * temp.entries[14] - 
               temp.entries[4]  * temp.entries[2] * temp.entries[13] - 
               temp.entries[12] * temp.entries[1] * temp.entries[6] + 
               temp.entries[12] * temp.entries[2] * temp.entries[5];

    inv[3] = -temp.entries[1] * temp.entries[6] * temp.entries[11] + 
              temp.entries[1] * temp.entries[7] * temp.entries[10] + 
              temp.entries[5] * temp.entries[2] * temp.entries[11] - 
              temp.entries[5] * temp.entries[3] * temp.entries[10] - 
              temp.entries[9] * temp.entries[2] * temp.entries[7] + 
              temp.entries[9] * temp.entries[3] * temp.entries[6];

    inv[7] = temp.entries[0] * temp.entries[6] * temp.entries[11] - 
             temp.entries[0] * temp.entries[7] * temp.entries[10] - 
             temp.entries[4] * temp.entries[2] * temp.entries[11] + 
             temp.entries[4] * temp.entries[3] * temp.entries[10] + 
             temp.entries[8] * temp.entries[2] * temp.entries[7] - 
             temp.entries[8] * temp.entries[3] * temp.entries[6];

    inv[11] = -temp.entries[0] * temp.entries[5] * temp.entries[11] + 
               temp.entries[0] * temp.entries[7] * temp.entries[9] + 
               temp.entries[4] * temp.entries[1] * temp.entries[11] - 
               temp.entries[4] * temp.entries[3] * temp.entries[9] - 
               temp.entries[8] * temp.entries[1] * temp.entries[7] + 
               temp.entries[8] * temp.entries[3] * temp.entries[5];

    inv[15] = temp.entries[0] * temp.entries[5] * temp.entries[10] - 
              temp.entries[0] * temp.entries[6] * temp.entries[9] - 
              temp.entries[4] * temp.entries[1] * temp.entries[10] + 
              temp.entries[4] * temp.entries[2] * temp.entries[9] + 
              temp.entries[8] * temp.entries[1] * temp.entries[6] - 
              temp.entries[8] * temp.entries[2] * temp.entries[5];

    float determinant = temp.entries[0] * inv[0] + temp.entries[1] * inv[4] + temp.entries[2] * inv[8] + temp.entries[3] * inv[12];

    if (determinant == 0.0f)
    {
        std::cout << "MATRIX4::This matrix's inverse does not exist." << std::endl;
        return *this;
    }

    determinant = 1.0f / determinant;
    Matrix4 result;

    for (int i = 0; i < 16; i++) {
        result.entries[i] = inv[i] * determinant;
    }

    return result.transpose();
}

Matrix4 Matrix4::operator+(const Matrix4& other) const
{
    Matrix4 result;

    for (int i = 0; i < 16; ++i)
    {
        result.entries[i] = entries[i] + other.entries[i];
    }

    return result;
}

Matrix4& Matrix4::operator+=(const Matrix4& other)
{
    for (int i = 0; i < 16; ++i)
    {
        entries[i] += other.entries[i];
    }
    return *this;
}

Matrix4 Matrix4::operator-(const Matrix4& other) const
{
    Matrix4 result;

    for (int i = 0; i < 16; ++i)
    {
        result.entries[i] = entries[i] - other.entries[i];
    }

    return result;
}

Matrix4& Matrix4::operator-=(const Matrix4& other)
{
    for (int i = 0; i < 16; ++i)
    {
        entries[i] -= other.entries[i];
    }
    return *this;
}

Matrix4 Matrix4::operator*(const float value) const
{
    Matrix4 result;

    for (int i = 0; i < 16; ++i)
    {
        result.entries[i] *= value;
    }

    return result;
}

Matrix4& Matrix4::operator*=(const float value)
{
    for (int i = 0; i < 16; ++i)
    {
        entries[i] *= value;
    }
    return *this;
}

Matrix4& Matrix4::operator=(const Matrix4& other)
{
    for (int i = 0; i < 16; ++i)
        entries[i] = other.entries[i];
    return *this;
}

void* Matrix4::operator new(size_t sz) {
    return _aligned_malloc(sz, 16);
}

void Matrix4::operator delete(void* ptr) {
    _aligned_free(ptr);
}

void* Matrix4::operator new[](size_t sz) {
    return _aligned_malloc(sz, 16);
}
void Matrix4::operator delete[](void* p) {
    _aligned_free(p);
}