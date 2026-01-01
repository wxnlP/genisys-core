set(KINEMATIC_TARGET_NAME kinematic_example)
set(TRAJECTORY_PLANNER_TARGET_NAME trajectory_planner_example)


add_executable(${KINEMATIC_TARGET_NAME} kinematic_sample.cc)
add_executable(${TRAJECTORY_PLANNER_TARGET_NAME} trajectory_planner_sample.cc)


target_link_libraries(${KINEMATIC_TARGET_NAME} PRIVATE
    s6h4d_sdk
)
target_link_libraries(${TRAJECTORY_PLANNER_TARGET_NAME} PRIVATE
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

install(TARGETS ${TRAJECTORY_PLANNER_TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)