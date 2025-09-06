set(TARGET_NAME s6h4d_sdk_example)

add_executable(${TARGET_NAME})

target_include_directories(${TARGET_NAME} PUBLIC
    interface
)

target_sources(${TARGET_NAME} PRIVATE
    main.cc
)

target_link_libraries(${TARGET_NAME} PUBLIC
    s6h4d_sdk
    serial
)

install(
    DIRECTORY interface/
    DESTINATION include/s6h4d_sdk
)

install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)