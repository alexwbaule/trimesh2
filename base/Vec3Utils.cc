#include "trimesh2/Vec3Utils.h"

namespace trimesh
{
	float vv_angle(const trimesh::vec3& v1, const trimesh::vec3& v2)
	{
		double denominator = std::sqrt((double)trimesh::len2(v1) * (double)trimesh::len2(v2));
		double cosinus = 0.0;
#ifdef WIN32
		if (denominator < DBL_MIN)
			cosinus = 1.0;
#else
        if (denominator < INT8_MIN)
            cosinus = 1.0;
#endif

		cosinus = (double)(trimesh::dot(v1, v2) / denominator);
		cosinus = cosinus > 1.0 ? 1.0 : (cosinus < -1.0 ? -1.0 : cosinus);
		double angle = std::acos(cosinus);
#ifdef WIN32
		if (!_finite(angle) || angle > M_PI)
			angle = 0.0;
#elif __APPLE__
	    if (!isfinite(angle) || angle > M_PI)
            angle = 0.0;
#elif defined(__ANDROID__)
		if (!isfinite(angle) || angle > M_PI)
			angle = 0.0;
#elif __WEB__
		if (!isfinite(angle) || angle > M_PI)
			angle = 0.0;
#else
        if (!__finite(angle) || angle > M_PI)
            angle = 0.0;
#endif
		return (float)angle;
	}

	trimesh::vec3 plane_point_project_shpere(const trimesh::vec2& p, const trimesh::vec2& c, const trimesh::vec2& size, bool skipz)
	{
		trimesh::vec3 pt3d;

		float dx = p.x - c.x;
		float dy = p.y - c.y;
#ifdef WIN32
		float w2 = __max(fabs(dx), __min(fabs(size.x - c.x), fabs(c.x)));
		float h2 = __max(fabs(dy), __min(fabs(size.y - c.y), fabs(c.y)));

		float maxdim = __max(w2, h2);
#else
        float w2 = fmax(fabs(dx), fmin(fabs(size.x - c.x), fabs(c.x)));
        float h2 = fmax(fabs(dy), fmin(fabs(size.y - c.y), fabs(c.y)));

        float maxdim = fmax(w2, h2);
#endif
		pt3d.x = dx / maxdim;
		pt3d.y = dy / maxdim;
		if (skipz)
		{
			pt3d.z = 0.0f;
			trimesh::normalize(pt3d);
		}
		else
		{
			float x2_plus_y2 = pt3d.x * pt3d.x + pt3d.y * pt3d.y;
			if (x2_plus_y2 > 1.0f)
			{
				pt3d.z = 0.0f;
				float length = std::sqrt(x2_plus_y2);
				pt3d.x /= length;
				pt3d.y /= length;
			}
			else
			{
				pt3d.z = std::sqrt(1.0f - x2_plus_y2);
			}
		}

		return pt3d;
	}

	trimesh::box3 transform_box(const trimesh::box3& box, const trimesh::fxform& xf)
	{
		trimesh::box3 tbox;
		if (box.valid)
		{
			trimesh::vec3 bmin = box.min;
			trimesh::vec3 bmax = box.max;

			tbox += xf * trimesh::vec3(bmin.x, bmin.y, bmin.z);
			tbox += xf * trimesh::vec3(bmin.x, bmin.y, bmax.z);
			tbox += xf * trimesh::vec3(bmin.x, bmax.y, bmax.z);
			tbox += xf * trimesh::vec3(bmin.x, bmax.y, bmin.z);
			tbox += xf * trimesh::vec3(bmax.x, bmin.y, bmin.z);
			tbox += xf * trimesh::vec3(bmax.x, bmin.y, bmax.z);
			tbox += xf * trimesh::vec3(bmax.x, bmax.y, bmax.z);
			tbox += xf * trimesh::vec3(bmax.x, bmax.y, bmin.z);
		}

		return tbox;
	}

}
