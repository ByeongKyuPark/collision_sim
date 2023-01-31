#pragma once
#include <cmath>
#include <physics/Vector.h>
namespace physics {
	const float  PI = 3.14159f;
	const float  TWO_PI = 2.0f * PI;
	const float  HALF_PI = 0.5f * PI;
	const float  QUARTER_PI = 0.25f * PI;
	const float  EIGHTH_PI = 0.125f * PI;
	const float  SIXTEENTH_PI = 0.0625f * PI;

	const float  DEG_TO_RAD = PI / 180.0f;
	const float  RAD_TO_DEG = 180.0f / PI;

	const float  EPSILON = 0.00001f;

	const Vector XAXIS(1.0f, 0.0f, 0.0f);
	const Vector YAXIS(0.0f, 1.0f, 0.0f);
	const Vector ZAXIS(0.0f, 0.0f, 1.0f);
}
