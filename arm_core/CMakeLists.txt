set(TARGET_NAME robotic_arm_sdk)

add_library(${TARGET_NAME} STATIC)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 仿真支持（可选，由顶层 BUILD_SIM_PROXY 控制）
if(BUILD_SIM_PROXY)
    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/simulation/sim_adapter/include
        ${CMAKE_SOURCE_DIR}/simulation/common_header/include
    )
    target_compile_definitions(${TARGET_NAME} PUBLIC BUILD_SIM_SUPPORT)
    target_link_libraries(${TARGET_NAME} PUBLIC sim_adapter)
endif()

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
    src/s6h4d_ctrl.cc
    src/arm_instance.cc
    src/joint_multi_point_path.cc
    src/cartesian_multi_point_path.cc
    src/gripper_ctrl_base.cc
    src/geniarm_gripper.cc
    src/geniarm_ctrl.cc
    src/robo_manager.cc
)
set(LINK_LIBS  dm_driver)
if(BUILD_DEFAULT_SERIAL)
    list(APPEND LINK_LIBS serial)
endif()
target_link_libraries(${TARGET_NAME} PRIVATE ${LINK_LIBS})

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

install(DIRECTORY cfg/
    DESTINATION ${CMAKE_BINARY_DIR}/cfg
)