cmake_minimum_required(VERSION 3.1)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
include(ConfigureAllModules) 

project(trimesh2)
add_subdirectory(trimesh2)

__configure_all()
