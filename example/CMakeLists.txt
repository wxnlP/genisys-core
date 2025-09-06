set(TARGET_NAME s6h4d_sdk_example)

add_executable(${TARGET_NAME})


target_sources(${TARGET_NAME} PRIVATE
    main.cc
)

target_link_libraries(${TARGET_NAME} PRIVATE
    s6h4d_sdk
)

install(
    DIRECTORY interface/
    DESTINATION include/s6h4d_sdk/example
)

install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)