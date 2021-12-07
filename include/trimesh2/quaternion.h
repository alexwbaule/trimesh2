#ifndef TRIMESH_QUATERNION_1604124753258_H
#define TRIMESH_QUATERNION_1604124753258_H
#include "trimesh2/Vec.h"

namespace trimesh
{
	class quaternion
	{
	public:
		quaternion();
		~quaternion();
		quaternion(float w, float x, float y, float z);
		quaternion(float w, const vec3& src);
		static quaternion rotationTo(const vec3& from, const vec3& to);
		quaternion QuaterNormalized() const;
		static quaternion fromAxes(const vec3& xAxis, const vec3& yAxis, const vec3& zAxis);
		static quaternion fromRotationMatrix(float rot3x3[3][3]);

		inline quaternion conjugated() const;

		inline vec3 vector() const;
		vec3 rotatedVector(const vec3& vector3) const;
		inline float dotProduct(const quaternion& q1, const quaternion& q2);
		void normalize();
		quaternion normalized() const;

		static quaternion fromDirection(vec3 dir, const vec3& fixedValue);
		static quaternion fromAxisAndAngle(const vec3& axis, float angle);
		static quaternion fromEular(vec3 angles);
		static bool qFuzzyIsNull(float f);
	
	public:
		float wp, xp, yp, zp;
	};

	const quaternion operator/(const quaternion& quaternion, float divisor);
	vec3 operator*(const quaternion& quaternion, const vec3& from);
	const quaternion operator*(const quaternion& q1, const quaternion& q2);
}

#endif // TRIMESH_QUATERNION_1604124753258_H