cmake_minimum_required(VERSION 3.8)

project(s6h4d_sdk LANGUAGES C CXX)

# 指定项目的C++版本
set(CMAKE_CXX_STANDARD 20)
# 强制编译器必须支持
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 禁用编译器扩展，确保代码兼容性
set(CMAKE_CXX_EXTENSIONS OFF)

add_subdirectory(s6h4d_core)

if(BUILD_DEFAULT_SERIAL) 
    add_subdirectory(third_party/serial)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()
