set(INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/")
set(TRIMESH_INCLUDE ${INCLUDE_DIR}trimesh2/Vec.h ${INCLUDE_DIR}trimesh2/XForm.h ${INCLUDE_DIR}trimesh2/Box.h ${INCLUDE_DIR}trimesh2/bsphere.h
					${INCLUDE_DIR}trimesh2/mathcompat.h ${INCLUDE_DIR}trimesh2/mathutil.h ${INCLUDE_DIR}trimesh2/strutil.h 
					${INCLUDE_DIR}trimesh2/TriMesh.h ${INCLUDE_DIR}trimesh2/Vec3Utils.h)
					
set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libsrc/")
set(TRIMESH_SRC ${SRC_DIR}TriMesh_bounding.cc ${SRC_DIR}TriMesh_connectivity.cc ${SRC_DIR}TriMesh_curvature.cc
				${SRC_DIR}TriMesh_grid.cc ${SRC_DIR}TriMesh_io.cc ${SRC_DIR}TriMesh_normals.cc 
				${SRC_DIR}TriMesh_pointareas.cc ${SRC_DIR}TriMesh_stats.cc ${SRC_DIR}TriMesh_tstrips.cc
				${SRC_DIR}KDtree.cc ${SRC_DIR}remove.cc ${SRC_DIR}reorder_verts.cc ${SRC_DIR}Vec3Utils.cpp)

set(SRCS ${TRIMESH_INCLUDE} ${TRIMESH_SRC})
set(INCS ${INCLUDE_DIR})
__add_real_target(trimesh2 lib SOURCE ${SRCS} INC ${INCS})

INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/trimesh2/ DESTINATION include/trimesh2/ FILES_MATCHING PATTERN "*.h")