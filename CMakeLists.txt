cmake_minimum_required(VERSION 3.20)
project(2d_quad VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)

###############################################################################
# Load installed packages
###############################################################################

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

include_directories(controller)
include_directories(math_helper)
include_directories(external/PX4-Matrix)

add_subdirectory(model)
add_subdirectory(project)
add_subdirectory(external)
