set(KINEMATIC_TARGET_NAME kinematic_example)
set(JACOBIAN_TARGET_NAME jacobian_example)
set(TRAJECTORY_TARGET_NAME trajectory_example)
set(MUJOCO_SIM_SEND_TARGET_NAME mujoco_sim_send_example)
set(MUJOCO_SIM_READ_TARGET_NAME mujoco_sim_read_example)
set(HTTP_SERVER_TARGET_NAME http_server_example)
set(HTTP_CLIENT_TARGET_NAME http_client_example)


add_executable(${KINEMATIC_TARGET_NAME} kinematic_sample.cc)
add_executable(${JACOBIAN_TARGET_NAME} jacobian_sample.cc)
add_executable(${TRAJECTORY_TARGET_NAME} trajectory_sample.cc)
add_executable(${MUJOCO_SIM_SEND_TARGET_NAME} mujoco_sim_ctrl_sample.cc)
add_executable(${MUJOCO_SIM_READ_TARGET_NAME} mujoco_sim_read_sample.cc)
add_executable(${HTTP_SERVER_TARGET_NAME} http_server_sample.cc)
add_executable(${HTTP_CLIENT_TARGET_NAME} http_client_sample.cc)


target_link_libraries(${KINEMATIC_TARGET_NAME} PRIVATE
    s6h4d_sdk
)
target_link_libraries(${JACOBIAN_TARGET_NAME} PRIVATE
    s6h4d_sdk
)
target_link_libraries(${TRAJECTORY_TARGET_NAME} PRIVATE
    s6h4d_sdk
)
target_link_libraries(${MUJOCO_SIM_SEND_TARGET_NAME} PRIVATE
    sim_adapter
    s6h4d_sdk
)
target_link_libraries(${MUJOCO_SIM_READ_TARGET_NAME} PRIVATE
    sim_adapter
    s6h4d_sdk
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