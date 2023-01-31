/*--------------------------------------------------------------
Primary Author: ByeongKyu Park
-----------------------------------------------------------------*/
#include "pch.h"
#include "matrix4.h"
#include "Vector.h"

#include "gtest/gtest.h"

TEST(Matrix_Mutliplication, IdentityMatrixTimesZeroMatrix_Expects_ZeroMatrix) {
	const Matrix4 m1 = Matrix4{1.f};
	const Matrix4 m2(0.f);

	Matrix4 r = m1* m2;
	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(m2[i], r[i]);
	}
}

TEST(Matrix_Mutliplication, ZeroMatrixTimesIdentityMatrix_Expects_ZeroMatrix) {
	const Matrix4 m1(0);
	const Matrix4 m2 = Matrix4{1.f};

	Matrix4 r = m1* m2;

	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(m1[i], r[i]);
	}
}

TEST(Matrix_Mutliplication, IdentityMatrixTimesIdentityMatrix_Expects_IdentityMatrix) {
	const Matrix4 m1 = Matrix4{1.f};
	const Matrix4 m2 = Matrix4{1.f};

	Matrix4 r = m1*m2;
	Matrix4 identity = Matrix4{1.f};
	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(identity[i], r[i]);
	}
}

TEST(Matrix_Mutliplication, IdentityMatrixTimesMatrix_Expects_Matrix) {
	const Matrix4 m1 = Matrix4{1.f};
	const Matrix4 m2(1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4);

	Matrix4 r = m1*m2;
	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(m2[i], r[i]);
	}

}

TEST(Matrix_Mutliplication, MatrixTimesIdentityMatrix_Expects_Matrix) {
	const Matrix4 m1(1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4);
	const Matrix4 m2{ 1.f };

	Matrix4 r = m1*m2;
	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(m1[i], r[i]);
	}
}

TEST(Matrix_Mutliplication, MatrixSquared_Expects_MatrixSquared) {
	const Matrix4 m1(1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4);

	Matrix4 r = m1 *m1;

	const Matrix4 expected(10, 20, 30, 40, 10, 20, 30, 40, 10, 20, 30, 40, 10, 20, 30, 40);

	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(expected[i], r[i]);
	}
}

TEST(Matrix_Mutliplication, MatrixTimesMatrix_Expects_CorrectValues) {
	const Matrix4 m1(1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4);
	const Matrix4 m2(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);

	Matrix4 r = m1* m2;

	const Matrix4 expected(90, 100, 110, 120, 90, 100, 110, 120, 90, 100, 110, 120, 90, 100, 110, 120);

	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(expected[i], r[i]);
	}
}
TEST(Matrix_Mutliplication, MatrixTimesMatrix_Expects_CorrectValues2) {
	const Matrix4 m1(71, 29, 44, 81, 8, 57, 26, 18, 29, 59, 4, 61, 73, 83, 84, 91);
	const Matrix4 m2(17, 41, 78, 53, 23, 54, 33, 2, 11, 44, 1, 51, 47, 5, 77, 25);

	Matrix4 r = m1* m2;

	const Matrix4 expected(6165, 6818, 12776, 8090, 2579, 4640, 3917, 2314, 4761, 4856, 8910, 3384, 8351, 11626, 15524, 10594);

	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(expected[i], r[i]);
	}
}
TEST(Matrix_Mutliplication, MatrixTimesMatrix_Expects_CorrectValues3) {
	const Matrix4 m1(6, 62, 8, 42, 52, 25, 82, 97, 96, 96, 14, 19, 64, 93, 34, 64);
	const Matrix4 m2(37, 74, 2, 35, 6, 53, 28, 69, 57, 68, 25, 15, 68, 86, 4, 33);

	Matrix4 r = m1*m2;

	const Matrix4 expected(3906, 7886, 2116, 5994, 13344, 19091, 3242, 7976, 6218, 14778, 3306, 10821, 9216, 17481, 3838, 11279);

	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(expected[i], r[i]);
	}
}
TEST(Matrix_Mutliplication, Heap_MatrixTimesMatrix_Expects_CorrectValues) {
	const Matrix4* m1 = new Matrix4(6, 62, 8, 42, 52, 25, 82, 97, 96, 96, 14, 19, 64, 93, 34, 64);
	const Matrix4* m2 = new Matrix4(37, 74, 2, 35, 6, 53, 28, 69, 57, 68, 25, 15, 68, 86, 4, 33);

	Matrix4 r = (*m1)* (*m2);

	const Matrix4 expected(3906, 7886, 2116, 5994, 13344, 19091, 3242, 7976, 6218, 14778, 3306, 10821, 9216, 17481, 3838, 11279);

	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(expected[i], r[i]);
	}
	delete m1;
	delete m2;
	m1 = m2 = nullptr;
}
/*
* (M1*M2)*M3 = M1*(M2*M3)
*/
TEST(Matrix_Mutliplication, TestAssociativity_Expects_IsAssociative) {
	const Matrix4 m1(-3, 6, -9, 12, -15, 18, -21, 24, -2, 4, -6, 8, -10, 12, -14, 16);
	const Matrix4 m2(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
	const Matrix4 m3(17, 19, -23, 29, 31, -37, 39, 41, -2, 3, 5, -7, 11, 13, -14, 15);

	Matrix4 r1 = m1* m2;
	r1 = r1* m3;
	Matrix4 r2 = m2* m3;
	r2 = m1* r2;

	for (int i = 0; i < 16; ++i) {
		EXPECT_FLOAT_EQ(r1[i], r2[i]);
	}
}

TEST(Matrix_Vector_Multiplication, IdentityMatrixTimesVector0_Expects_Vector0) {
	const Matrix4 m = Matrix4{1.f};
	const Vector v(0);

	auto r = m*v;

	EXPECT_EQ(r.x, 0);
	EXPECT_EQ(r.y, 0);
	EXPECT_EQ(r.z, 0);
	EXPECT_EQ(r.w, 1);
}

TEST(Matrix_Vector_Multiplication, IdentityMatrixTimesVector1_Expects_Vector1) {
	const Matrix4 m = Matrix4{1.f};
	const Vector v(1);

	auto r = m* v;

	EXPECT_EQ(r.x, 1);
	EXPECT_EQ(r.y, 1);
	EXPECT_EQ(r.z, 1);
	EXPECT_EQ(r.w, 1);
}

TEST(Matrix_Vector_Multiplication, IdentityMatrixTimesVector_Expects_Vector) {
	const Matrix4 m = Matrix4{1.f};
	const Vector v(1, 2, 3, 4);

	auto r = m*v;

	EXPECT_EQ(r.x, v.x);
	EXPECT_EQ(r.y, v.y);
	EXPECT_EQ(r.z, v.z);
	EXPECT_EQ(r.w, v.w);
}

TEST(Matrix_Vector_Multiplication, Matrix0TimesV0_Expects_Vector0) {
	const Matrix4 m(0);
	const Vector v(0);

	auto r = m*v;

	EXPECT_EQ(r.x, 0);
	EXPECT_EQ(r.y, 0);
	EXPECT_EQ(r.z, 0);
	EXPECT_EQ(r.w, 0);
}

TEST(Matrix_Vector_Multiplication, Matrix0TimesVector1_Expects_Vector0) {
	const Matrix4 m(0);
	const Vector v(1);

	auto r = m*v;

	EXPECT_EQ(r.x, 0);
	EXPECT_EQ(r.y, 0);
	EXPECT_EQ(r.z, 0);
	EXPECT_EQ(r.w, 0);
}

TEST(Matrix_Vector_Multiplication, Matrix1TimesVector0_Expects_Vector0) {
	const Matrix4 m(1);
	const Vector v(0);

	auto r = m*v;

	EXPECT_EQ(r.x, 0);
	EXPECT_EQ(r.y, 0);
	EXPECT_EQ(r.z, 0);
	EXPECT_EQ(r.w, 1);
}

TEST(Matrix_Vector_Multiplication, Matrix1TimesVector1_Expects_Vector) {
	const Matrix4 m(1);
	const Vector v(1);

	auto r = m*v;

	EXPECT_EQ(r.x, 1);
	EXPECT_EQ(r.y, 1);
	EXPECT_EQ(r.z, 1);
	EXPECT_EQ(r.w, 1);
}

TEST(Matrix_Vector_Multiplication, Matrix1TimesVector_Expects_Vector10) {
	const Matrix4 m(1);
	const Vector v(1, 2, 3, 4);

	auto r = m*v;

	EXPECT_EQ(r.x, 1);
	EXPECT_EQ(r.y, 2);
	EXPECT_EQ(r.z, 3);
	EXPECT_EQ(r.w, 4);
}

TEST(Matrix_Vector_Multiplication, MatrixTimesVector_Expects_CorrectValue1) {
	const Matrix4 m(1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4);
	const Vector v(1, 2, 3, 4);

	auto r = m*v;

	EXPECT_EQ(r.x, 30);
	EXPECT_EQ(r.y, 30);
	EXPECT_EQ(r.z, 30);
	EXPECT_EQ(r.w, 30);
}

TEST(Matrix_Vector_Multiplication, MatrixTimesVector_Expects_CorrectValue2) {
	const Matrix4 m(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16);
	const Vector v(1, 2, 3, 4);

	auto r = m*v;

	EXPECT_EQ(r.x, 30);
	EXPECT_EQ(r.y, 70);
	EXPECT_EQ(r.z, 110);
	EXPECT_EQ(r.w, 150);
}

TEST(Matrix_Vector_Multiplication, MatrixTimesVector_Expects_CorrectValue3) {
	const Matrix4 m(43, 22, 63, 46, 75, 86, 88, 23, 8, 7, 13, 79, 62, 99, 71, 1);
	const Vector v(18, 96, 51, 64);

	auto r = m*v;

	EXPECT_EQ(r.x, 9043);
	EXPECT_EQ(r.y, 15566);
	EXPECT_EQ(r.z, 6535);
	EXPECT_EQ(r.w, 14305);
}
TEST(Matrix_Vector_Multiplication, Heap_MatrixTimesVector_Expects_CorrectValue) {
	const Matrix4* m = new Matrix4(43, 22, 63, 46, 75, 86, 88, 23, 8, 7, 13, 79, 62, 99, 71, 1);
	const Vector* v = new Vector(18, 96, 51, 64);

	auto r = (*m) * (*v);

	EXPECT_EQ(r.x, 9043);
	EXPECT_EQ(r.y, 15566);
	EXPECT_EQ(r.z, 6535);
	EXPECT_EQ(r.w, 14305);

	delete m;
	delete v;
	m = nullptr;
	v = nullptr;
}
/*
* M*(v1+v2)==M*v1+M*v2
*/
TEST(Matrix_Vector_Multiplication, MatrixTimesVector_Expects_IsDistributive) {
	const Matrix4 m(16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1);
	const Vector v1(-1, -2, 3, 4);
	const Vector v2(5, -6, -7, 8);
	const Vector v1PlusV2{ v1.x + v2.x,v1.y + v2.y,v1.z + v2.z, v1.w + v2.w };

	auto r1 = m* v1;
	auto r2 = m* v2;

	Vector r1PlusR2{ r1.x + r2.x,r1.y + r2.y,r1.z + r2.z,r1.w + r2.w };
	auto r3 = m* v1PlusV2;

	EXPECT_EQ(r1PlusR2.x, r3.x);
	EXPECT_EQ(r1PlusR2.y, r3.y);
	EXPECT_EQ(r1PlusR2.z, r3.z);
	EXPECT_EQ(r1PlusR2.w, r3.w);
}

TEST(Matrix_Vector_Multiplication, MatrixTimesVector_Expects_Vector0) {
	const Matrix4 m(16, -15, -14, 13, -12, 11, 10, -9, 8, -7, -6, 5, 4, -3, -2, 1);
	const Vector v(1);

	auto r = m*v;

	EXPECT_EQ(r.x, 0);
	EXPECT_EQ(r.y, 0);
	EXPECT_EQ(r.z, 0);
	EXPECT_EQ(r.w, 0);
}