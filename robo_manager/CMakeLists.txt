set(CUR_TARGET_NAME robotic_arm_sdk)

add_library(${CUR_TARGET_NAME} STATIC)

target_include_directories(${CUR_TARGET_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/${CUR_TARGET_NAME}>
)

target_sources(${CUR_TARGET_NAME} PRIVATE
    robo_manager.cc
)

target_link_libraries(${CUR_TARGET_NAME} PUBLIC
    actuator_controller
    arm_controller
)

install(TARGETS ${CUR_TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(FILES robo_manager.h
    DESTINATION include/${CUR_TARGET_NAME}
)
