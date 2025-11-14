set(TARGET_NAME s6h4d_sdk)

add_library(${TARGET_NAME} STATIC)

# 头文件搜索路径
target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 添加源文件
target_sources(${TARGET_NAME} PRIVATE
    src/interface_base.cc
    src/kinematic.cc
)

if(BUILD_EXAMPLES)
    target_include_directories(${TARGET_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../example/interface/
    )
endif()

if(BUILD_DEFAULT_SERIAL)
    target_link_libraries(${TARGET_NAME} PUBLIC serial)
endif()


# 安装静态库目标
install(TARGETS ${TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include/${TARGET_NAME}
)