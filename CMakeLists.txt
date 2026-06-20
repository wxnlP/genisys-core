cmake_minimum_required(VERSION 3.24)

project(robotic_arm_sdk LANGUAGES C CXX)

# 修改默认的库文件、存档文件的存放位置
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})


# 指定项目的C++版本
set(CMAKE_CXX_STANDARD 20)
# 强制编译器必须支持
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 禁用编译器扩展，确保代码兼容性
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 是否构建默认 serial 库
option(BUILD_DEFAULT_SERIAL "Build the default serial library" ON)

# 是否构建示例
option(BUILD_EXAMPLES "Build examples" ON)


add_subdirectory(actuator)
add_subdirectory(arm_control)
add_subdirectory(robo_manager)

if (BUILD_EXAMPLES)
    add_subdirectory(example)
endif()

install(DIRECTORY cfg/
    DESTINATION ${CMAKE_BINARY_DIR}/cfg
)