set(TARGET_NAME arm_controller)

add_library(${TARGET_NAME} STATIC)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${TARGET_NAME} PUBLIC Eigen3::Eigen yaml-cpp)

# 添加源文件
target_sources(${TARGET_NAME} PRIVATE
    src/kinematic.cc
    src/jacobian.cc
    src/cubic_time_scaler.cc
    src/lspb_time_scaler.cc
    src/quintic_time_scaler.cc
    src/cartesian_linear_path.cc
    src/joint_path.cc
    src/cartesian_trajectory.cc
    src/joint_trajectory.cc
    src/joint_multi_point_path.cc
    src/cartesian_multi_point_path.cc
    src/arm_controller.cc
    src/gripper_controller.cc
    src/arm_manager.cc
)

# 安装静态库目标
install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include/${TARGET_NAME}
)
