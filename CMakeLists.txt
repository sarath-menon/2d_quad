cmake_minimum_required(VERSION 3.20)
project(2d_quad VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)

include_directories(controller)


add_subdirectory(model)
add_subdirectory(project)
add_subdirectory(external)
