

add_executable(quad_2d
main.cpp
)

target_link_libraries(quad_2d PRIVATE
model_lib
mahi::gui
yaml-cpp
)



