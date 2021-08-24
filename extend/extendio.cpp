#include "trimesh2/extendio.h"

namespace trimesh
{
	void writeTriangleSoup(trimesh::TriMesh* mesh, const char* name)
	{
		if (mesh)
		{
			int vertexSize = (int)mesh->vertices.size();
			int faceSize = vertexSize / 3;
			mesh->faces.resize(faceSize);
			for (int i = 0; i < faceSize; ++i)
			{
				trimesh::TriMesh::Face& face = mesh->faces.at(i);
				face[0] = 3 * i;
				face[1] = 3 * i + 1;
				face[2] = 3 * i + 2;
			}
			int errorCode = 0;
			mesh->write(name, errorCode);
		}
	}
}