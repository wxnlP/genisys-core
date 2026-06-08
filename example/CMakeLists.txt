set(KINEMATIC_TARGET_NAME kinematic_example)
set(JACOBIAN_TARGET_NAME jacobian_example)
set(TRAJECTORY_TARGET_NAME trajectory_example)
set(SIM_SEND_TARGET_NAME sim_send_example)
set(SIM_READ_TARGET_NAME sim_read_example)
set(SIM_MOVEL_MOVEJ_TARGET_NAME sim_movel_movej_example)
set(SIM_MULTI_POINT_TARGET_NAME sim_multi_point_example)
set(SIM_GRIPPER_TARGET_NAME sim_gripper_example)
set(REAL_GRIPPER_TARGET_NAME real_gripper_example)

add_executable(kinematic_demo demo.cc)
add_executable(${KINEMATIC_TARGET_NAME} kinematic_sample.cc)
add_executable(${JACOBIAN_TARGET_NAME} jacobian_sample.cc)
add_executable(${TRAJECTORY_TARGET_NAME} trajectory_sample.cc)
add_executable(${REAL_GRIPPER_TARGET_NAME} real_gripper_sample.cc)


target_link_libraries(${KINEMATIC_TARGET_NAME} PRIVATE
    robotic_arm_sdk
)
target_link_libraries(${JACOBIAN_TARGET_NAME} PRIVATE
    robotic_arm_sdk
)
target_link_libraries(${TRAJECTORY_TARGET_NAME} PRIVATE
    robotic_arm_sdk
)

target_link_libraries(kinematic_demo PRIVATE
    robotic_arm_sdk
)

target_link_libraries(${REAL_GRIPPER_TARGET_NAME} PRIVATE
    robotic_arm_sdk    
    dm_driver    
)

install(TARGETS ${KINEMATIC_TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${TRAJECTORY_TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if (BUILD_SIM_PROXY)

    add_executable(${SIM_SEND_TARGET_NAME} sim_ctrl_sample.cc)
    add_executable(${SIM_READ_TARGET_NAME} sim_read_sample.cc)
    add_executable(${SIM_MOVEL_MOVEJ_TARGET_NAME} movel_movej_sim_sample.cc)
    add_executable(${SIM_MULTI_POINT_TARGET_NAME} multi_point_movel_movej_sim_sample.cc)
    add_executable(${SIM_GRIPPER_TARGET_NAME} sim_gripper_sample.cc)

    target_link_libraries(${SIM_SEND_TARGET_NAME} PRIVATE
        sim_adapter
        robotic_arm_sdk
    )
    target_link_libraries(${SIM_READ_TARGET_NAME} PRIVATE
        sim_adapter
        robotic_arm_sdk
    )
    target_link_libraries(${SIM_MOVEL_MOVEJ_TARGET_NAME} PRIVATE
        sim_adapter
        robotic_arm_sdk
    )
    target_link_libraries(${SIM_MULTI_POINT_TARGET_NAME} PRIVATE
        sim_adapter
        robotic_arm_sdk
    )
    target_link_libraries(${SIM_GRIPPER_TARGET_NAME} PRIVATE
        sim_adapter
        robotic_arm_sdk
    )


    install(TARGETS ${SIM_MOVEL_MOVEJ_TARGET_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    install(TARGETS ${SIM_MULTI_POINT_TARGET_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    install(TARGETS ${SIM_GRIPPER_TARGET_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif()