set(CUR_TARGET_NAME "actuator_controller")

add_subdirectory(third_party/dm_driver)
option(BUILD_DEFAULT_SERIAL "Build the default serial library" ON)
if(BUILD_DEFAULT_SERIAL)
    add_subdirectory(third_party/serial)
endif()

add_library(${CUR_TARGET_NAME} STATIC)

file(GLOB_RECURSE src src/*.cc)

target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

target_include_directories(${CUR_TARGET_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/arm_core/include>
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