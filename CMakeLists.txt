cmake_minimum_required(VERSION 3.20)
project(2d_quad VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 17)


add_executable(2d_quad 
main.cpp
)

target_include_directories(2d_quad PRIVATE
controller
model
)