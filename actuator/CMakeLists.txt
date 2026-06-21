set(CUR_TARGET_NAME "actuator_controller")

add_subdirectory(third_party/dm_driver)
add_subdirectory(third_party/serial)

add_library(${CUR_TARGET_NAME} STATIC)

file(GLOB_RECURSE src src/*.cc)

target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

target_include_directories(${CUR_TARGET_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${CUR_TARGET_NAME}>
)

target_link_libraries(${CUR_TARGET_NAME} PUBLIC 
  serial
  dm_driver
)

# 安装静态库目标
install(TARGETS ${CUR_TARGET_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
    DESTINATION include/${CUR_TARGET_NAME}
)