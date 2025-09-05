cmake_minimum_required(VERSION 3.8)

set(PROJECT_NAME s6h4d_sdk)

project(${PROJECT_NAME} LANGUAGES CXX)

# 指定项目的C++版本
set(CMAKE_CXX_STANDARD 20)
# 强制编译器必须支持
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 禁用编译器扩展，确保代码兼容性
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(${PROJECT_NAME} STATIC)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_sources(${PROJECT_NAME} PRIVATE
    interface/interface_base.cc
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin 
)

install(DIRECTORY interface/ common/
    DESTINATION include/${PROJECT_NAME}
)