set(KINEMATIC_TARGET_NAME kinematic_example)
set(JACOBIAN_TARGET_NAME jacobian_example)
set(TRAJECTORY_TARGET_NAME trajectory_example)


add_executable(${KINEMATIC_TARGET_NAME} kinematic_sample.cc)
add_executable(${JACOBIAN_TARGET_NAME} jacobian_sample.cc)
add_executable(${TRAJECTORY_TARGET_NAME} trajectory_sample.cc)


target_link_libraries(${KINEMATIC_TARGET_NAME} PRIVATE
    s6h4d_sdk
)
target_link_libraries(${JACOBIAN_TARGET_NAME} PRIVATE
    s6h4d_sdk
)
target_link_libraries(${TRAJECTORY_TARGET_NAME} PRIVATE
    s6h4d_sdk
)

install(
    DIRECTORY interface/
    DESTINATION include/s6h4d_sdk/example
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