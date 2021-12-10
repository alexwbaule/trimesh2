#include "trimesh2/quaternion.h"

namespace trimesh
{
	quaternion::quaternion()
		:wp(1.0f), xp(0.0f), yp(0.0f), zp(0.0f)
	{

	}

	quaternion::~quaternion()
	{

	}

	quaternion::quaternion(float w, float x, float y, float z)
	{
		wp = w;
		xp = x;
		yp = y;
		zp = z;
	}

	quaternion::quaternion(float w, const vec3& src)
	{
		wp = w;
		xp = src.at(0);
		yp = src.at(1);
		zp = src.at(2);
	}

	quaternion quaternion::fromRotationMatrix(float rot3x3[3][3])
	{
		// Algorithm from:
	// http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q55

		float scalar;
		float axis[3];

		const float trace = rot3x3[0][0] + rot3x3[1][1] + rot3x3[2][2];
		if (trace > 0.00000001f) {
			const float s = 2.0f * std::sqrt(trace + 1.0f);
			scalar = 0.25f * s;
			axis[0] = (rot3x3[2][1] - rot3x3[1][2]) / s;
			axis[1] = (rot3x3[0][2] - rot3x3[2][0]) / s;
			axis[2] = (rot3x3[1][0] - rot3x3[0][1]) / s;
		}
		else {
			static int s_next[3] = { 1, 2, 0 };
			int i = 0;
			if (rot3x3[1][1] > rot3x3[0][0])
				i = 1;
			if (rot3x3[2][2] > rot3x3[i][i])
				i = 2;
			int j = s_next[i];
			int k = s_next[j];

			const float s = 2.0f * std::sqrt(rot3x3[i][i] - rot3x3[j][j] - rot3x3[k][k] + 1.0f);
			axis[i] = 0.25f * s;
			scalar = (rot3x3[k][j] - rot3x3[j][k]) / s;
			axis[j] = (rot3x3[j][i] + rot3x3[i][j]) / s;
			axis[k] = (rot3x3[k][i] + rot3x3[i][k]) / s;
		}
		return quaternion(scalar, axis[0], axis[1], axis[2]);
	}

	inline quaternion quaternion::conjugated() const
	{
		return quaternion(wp, -xp, -yp, -zp);
	}

	inline vec3 quaternion::vector() const
	{
		return vec3(xp, yp, zp);
	}

	vec3 quaternion::rotatedVector(const vec3& vector3) const
	{
		return (*this * quaternion(0, vector3) * conjugated()).vector();
	}

	inline float quaternion::dotProduct(const quaternion& q1, const quaternion& q2)
	{
		return q1.wp * q2.wp + q1.xp * q2.xp + q1.yp * q2.yp + q1.zp * q2.zp;
	}

	quaternion quaternion::fromAxes(const vec3& xAxis, const vec3& yAxis, const vec3& zAxis)
	{
		//QMatrix3x3 rot3x3(Qt::Uninitialized);
		float rot3x3[3][3] = { 0 };
		rot3x3[0][0] = xAxis.at(0);
		rot3x3[1][0] = xAxis.at(1);
		rot3x3[2][0] = xAxis.at(2);
		rot3x3[0][1] = yAxis.at(0);
		rot3x3[1][1] = yAxis.at(1);
		rot3x3[2][1] = yAxis.at(2);
		rot3x3[0][2] = zAxis.at(0);
		rot3x3[1][2] = zAxis.at(1);
		rot3x3[2][2] = zAxis.at(2);

		return fromRotationMatrix(rot3x3);
	}

	quaternion quaternion::fromDirection(vec3 dir, const vec3& fixedValue)
	{
		if (qFuzzyIsNull(dir.at(0)) && qFuzzyIsNull(dir.at(1)) && qFuzzyIsNull(dir.at(2)))
			return quaternion();

		const vec3 zAxis(trimesh::normalized(dir));
		vec3 xAxis(fixedValue TRICROSS zAxis);
		if (qFuzzyIsNull(len2(xAxis)))
		{
			// collinear or invalid up vector; derive shortest arc to new direction
			return quaternion::rotationTo(vec3(0.0f, 0.0f, 1.0f), zAxis);
		}

		trimesh::normalize(xAxis);
		const vec3 yAxis((zAxis TRICROSS xAxis));

		return quaternion::fromAxes(xAxis, yAxis, zAxis);
	}

	quaternion quaternion::fromAxisAndAngle(const vec3& axis, float angle)
	{
		// Algorithm from:
		// http://www.j3d.org/matrix_faq/matrfaq_latest.html#Q56
		// We normalize the result just in case the values are close
		// to zero, as suggested in the above FAQ.
		float a = angle * M_PI_2f / 180.0f;
		float s = std::sin(a);
		float c = std::cos(a);
		trimesh::vec3 ax = trimesh::normalized(axis);
		return quaternion(c, ax.x * s, ax.y * s, ax.z * s).normalized();
	}

	quaternion quaternion::fromEular(vec3 angles)
	{
		double yaw = angles.z * M_PI / 180.0;
		double pitch = angles.y * M_PI / 180;
		double roll = angles.x * M_PI / 180;
		double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);

		quaternion q;
		q.wp = cy * cp * cr + sy * sp * sr;
		q.xp = cy * cp * sr - sy * sp * cr;
		q.yp = sy * cp * sr + cy * sp * cr;
		q.zp = sy * cp * cr - cy * sp * sr;
		return q.normalized();
	}

	void quaternion::normalize()
	{
		// Need some extra precision if the length is very small.
		double len = double(xp) * double(xp) +
			double(yp) * double(yp) +
			double(zp) * double(zp) +
			double(wp) * double(wp);
		if (qFuzzyIsNull(len - 1.0f) || qFuzzyIsNull(len))
			return;

		len = std::sqrt(len);

		xp /= len;
		yp /= len;
		zp /= len;
		wp /= len;
	}

	quaternion quaternion::normalized() const
	{
		// Need some extra precision if the length is very small.
		double len = double(xp) * double(xp) +
			double(yp) * double(yp) +
			double(zp) * double(zp) +
			double(wp) * double(wp);
		if (qFuzzyIsNull(len - 1.0f))
			return *this;
		else if (!qFuzzyIsNull(len))
			return *this / std::sqrt(len);
		else
			return quaternion(0.0f, 0.0f, 0.0f, 0.0f);
	}

	quaternion quaternion::rotationTo(const vec3& from, const vec3& to)
	{
		// Based on Stan Melax's article in Game Programming Gems

		const vec3 v0(trimesh::normalized(from));
		const vec3 v1(trimesh::normalized(to));

		//float d = dotProduct(v0, v1) + 1.0f;
		float d = (v0 DOT v1) + 1.0f;


		// if dest vector is close to the inverse of source vector, ANY axis of rotation is valid
		if (qFuzzyIsNull(d))
		{
			vec3 axis = (vec3(1.0f, 0.0f, 0.0f) TRICROSS v0);
			if (qFuzzyIsNull(len2(axis)))
				axis = (vec3(0.0f, 1.0f, 0.0f) TRICROSS v0);
			trimesh::normalize(axis);

			// same as QQuaternion::fromAxisAndAngle(axis, 180.0f)
			return quaternion(0.0f, axis.at(0), axis.at(1), axis.at(2));
		}

		d = std::sqrt(2.0f * d);
		const vec3 axis((v0 TRICROSS v1) / d);

		return quaternion(d * 0.5f, axis).QuaterNormalized();
	}

	quaternion quaternion::QuaterNormalized() const
	{
		// Need some extra precision if the length is very small.
		double len = double(xp) * double(xp) +
			double(yp) * double(yp) +
			double(zp) * double(zp) +
			double(wp) * double(wp);
		if (qFuzzyIsNull(len - 1.0f))
			return *this;
		else if (!qFuzzyIsNull(len))
			return *this / std::sqrt(len);
		else
			return quaternion(0.0f, 0.0f, 0.0f, 0.0f);
	}


	const quaternion operator/(const quaternion& q, float divisor)
	{
		return quaternion(q.wp / divisor, q.xp / divisor, q.yp / divisor, q.zp / divisor);
	}

	vec3 operator*(const quaternion& q, const vec3& from)
	{
		return q.rotatedVector(from);
	}

	const quaternion operator*(const quaternion& q1, const quaternion& q2)
	{
		float yy = (q1.wp - q1.yp) * (q2.wp + q2.zp);
		float zz = (q1.wp + q1.yp) * (q2.wp - q2.zp);
		float ww = (q1.zp + q1.xp) * (q2.xp + q2.yp);
		float xx = ww + yy + zz;
		float qq = 0.5f * (xx + (q1.zp - q1.xp) * (q2.xp - q2.yp));

		float w = qq - ww + (q1.zp - q1.yp) * (q2.yp - q2.zp);
		float x = qq - xx + (q1.xp + q1.wp) * (q2.xp + q2.wp);
		float y = qq - yy + (q1.wp - q1.xp) * (q2.yp + q2.zp);
		float z = qq - zz + (q1.zp + q1.yp) * (q2.wp - q2.xp);

		return quaternion(w, x, y, z);
	}

	bool quaternion::qFuzzyIsNull(float f)
	{
		return fabsf(f) <= 0.00001f;
	}
	
	trimesh::fxform fromQuaterian(const trimesh::quaternion& q)
	{
		float x2 = q.xp * q.xp;
		float y2 = q.yp * q.yp;
		float z2 = q.zp * q.zp;
		float xy = q.xp * q.yp;
		float xz = q.xp * q.zp;
		float yz = q.yp * q.zp;
		float wx = q.wp * q.xp;
		float wy = q.wp * q.yp;
		float wz = q.wp * q.zp;


		// This calculation would be a lot more complicated for non-unit length quaternions
		// Note: The constructor of Matrix4 expects the Matrix in column-major format like expected by
		//   OpenGL
		return trimesh::fxform(1.0f - 2.0f * (y2 + z2), 2.0f * (xy - wz), 2.0f * (xz + wy), 0.0f,
			2.0f * (xy + wz), 1.0f - 2.0f * (x2 + z2), 2.0f * (yz - wx), 0.0f,
			2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (x2 + y2), 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f);
	}
}