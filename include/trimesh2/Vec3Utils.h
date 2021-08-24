#ifndef VEC3_UTILS_H
#define VEC3_UTILS_H
#include "Vec.h"
#include "XForm.h"
#include "Box.h"

namespace trimesh
{
	float vv_angle(const trimesh::vec3& v1, const trimesh::vec3& v2);
	trimesh::vec3 plane_point_project_shpere(const trimesh::vec2& p, const trimesh::vec2& center, const trimesh::vec2& size, bool skip_z = false);

	trimesh::box3 transform_box(const trimesh::box3& box, const trimesh::fxform& xf);
}


#endif // VEC3_UTILS_H