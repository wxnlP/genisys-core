set(TARGET_NAME robo_manager)

add_library(${TARGET_NAME} STATIC)

target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/actuator/include
    ${CMAKE_SOURCE_DIR}/arm_control/include
)

target_sources(${TARGET_NAME} PRIVATE
    robo_manager.cc
)

target_link_libraries(${TARGET_NAME} PUBLIC
    actuator_controller
    arm_controller
)

install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
