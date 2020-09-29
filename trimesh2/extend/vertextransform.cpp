#include "trimesh2/vertextransform.h"

namespace trimesh
{
	void transformVertex(std::vector<vec3>& vertexes, fxform& xf)
	{
		int nv = (int)vertexes.size();

		for (int i = 0; i < nv; i++)
			vertexes[i] = xf * vertexes[i];
	}

	void ompTransformVertex(std::vector<vec3>& vertexes, fxform& xf)
	{
		int nv = (int)vertexes.size();

#pragma omp parallel for
		for (int i = 0; i < nv; i++)
			vertexes[i] = xf * vertexes[i];
	}

	void vecAdd(std::vector<float>& inA, std::vector<float>& inB, std::vector<float>& outC)
	{
		if (inA.size() == outC.size() && inB.size() == outC.size())
		{
			int size = (int)inA.size();
			for (int i = 0; i < size; ++i)
				outC.at(i) = inA.at(i) + inB.at(i);
		}
	}
}