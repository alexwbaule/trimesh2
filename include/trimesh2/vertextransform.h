#ifndef TRIMESH_VERTEXTRANSFORM_1597373253543_H
#define TRIMESH_VERTEXTRANSFORM_1597373253543_H
#include "Vec.h"
#include <vector>
#include "XForm.h"

namespace trimesh
{
	void transformVertex(std::vector<vec3>& vertexes, fxform& xf);
	void ompTransformVertex(std::vector<vec3>& vertexes, fxform& xf);

	void vecAdd(std::vector<float>& inA, std::vector<float>& inB, std::vector<float>& outC);
}

#endif // TRIMESH_VERTEXTRANSFORM_1597373253543_H