set(SIM_ADAPTER_TARGET_NAME sim_adapter)

add_library(${SIM_ADAPTER_TARGET_NAME} STATIC)

target_sources(${SIM_ADAPTER_TARGET_NAME} PRIVATE
  src/mujoco_sim_ctrl.cc
)

target_include_directories(${SIM_ADAPTER_TARGET_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${SIM_ADAPTER_TARGET_NAME} PRIVATE
  s6h4d_sdk
  sim_common_header
)

# 安装静态库目标
install(TARGETS ${SIM_ADAPTER_TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include/${SIM_ADAPTER_TARGET_NAME}
)