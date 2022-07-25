file(GLOB BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/base/*.cc)
file(GLOB EXTEND_SRC ${CMAKE_CURRENT_SOURCE_DIR}/extend/*.cpp)

set(SRCS ${BASE_SRC} ${EXTEND_SRC})
set(INCS ${CMAKE_CURRENT_SOURCE_DIR}/include/)

__add_real_target(trimesh2 lib 
							SOURCE ${SRCS}
							INC ${INCS}
							INTERFACE ${INCS}
							)

if(CC_INSTALL_TRIMESH2)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/include/ trimesh2)
endif()
