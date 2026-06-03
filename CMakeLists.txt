cmake_minimum_required(VERSION 3.24)

project(s6h4d_sdk LANGUAGES C CXX)

# 修改默认的库文件、存档文件的存放位置
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
# openEuler 补丁
set(CMAKE_PREFIX_PATH "$ENV{HOME}/local" ${CMAKE_PREFIX_PATH})

# 指定项目的C++版本
set(CMAKE_CXX_STANDARD 20)
# 强制编译器必须支持
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 禁用编译器扩展，确保代码兼容性
set(CMAKE_CXX_EXTENSIONS OFF)

# 是否构建默认 serial 库
option(BUILD_DEFAULT_SERIAL "Build the default serial library" ON)

# 是否构建示例
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_SIM_PROXY "Build sim proxy" ON)


add_subdirectory(s6h4d_core)
add_subdirectory(third_party/dm_driver)

if(BUILD_DEFAULT_SERIAL) 
    add_subdirectory(third_party/serial)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()

if(BUILD_SIM_PROXY)
    add_subdirectory(simulation/common_header)
    add_subdirectory(simulation/sim_proxy)
    add_subdirectory(simulation/sim_adapter)
endif()
