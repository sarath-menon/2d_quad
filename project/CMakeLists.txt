

add_executable(quad_2d
main.cpp
)

target_include_directories(quad_2d PRIVATE include)

target_link_libraries(quad_2d PRIVATE
model_lib
controller_lib
mahi::gui
yaml-cpp
fastcdr 
fastrtps
msg_lib
mocap_fastdds_lib
)


# add_executable(quad_2d_old
# main.cpp
# )

# target_include_directories(quad_2d_old PRIVATE include)

# target_link_libraries(quad_2d_old PRIVATE
# model_lib
# mahi::gui
# yaml-cpp
# fastcdr 
# fastrtps
# msg_lib
# mocap_fastdds_lib
# )


