set(KINEMATIC_TARGET_NAME kinematic_example)
set(JACOBIAN_TARGET_NAME jacobian_example)
set(TRAJECTORY_TARGET_NAME trajectory_example)
set(REAL_GRIPPER_TARGET_NAME real_gripper_example)
set(REAL_ARM_TARGET_NAME real_arm_example)


add_executable(${KINEMATIC_TARGET_NAME} kinematic_sample.cc)
add_executable(${JACOBIAN_TARGET_NAME} jacobian_sample.cc)
add_executable(${TRAJECTORY_TARGET_NAME} trajectory_sample.cc)
add_executable(${REAL_GRIPPER_TARGET_NAME} real_gripper_sample.cc)
add_executable(${REAL_ARM_TARGET_NAME} real_arm_sample.cc)


target_link_libraries(${KINEMATIC_TARGET_NAME} PRIVATE
    robotic_arm_sdk
)
target_link_libraries(${JACOBIAN_TARGET_NAME} PRIVATE
    robotic_arm_sdk
)
target_link_libraries(${TRAJECTORY_TARGET_NAME} PRIVATE
    robotic_arm_sdk
)

target_link_libraries(${REAL_GRIPPER_TARGET_NAME} PRIVATE
    robotic_arm_sdk    
    dm_driver
)

target_link_libraries(${REAL_ARM_TARGET_NAME} PRIVATE
    robotic_arm_sdk    
    dm_driver   
    serial 
)
