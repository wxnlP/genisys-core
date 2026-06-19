cmake_minimum_required(VERSION 3.24)
project(dm_driver LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_TARGET_NAME dm_driver)
add_library(${CMAKE_TARGET_NAME} STATIC)

target_include_directories(${CMAKE_TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/port/linux
)

target_sources(${CMAKE_TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/core/dm_motor_driver.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/port/linux/socket_can.c
    ${CMAKE_CURRENT_SOURCE_DIR}/port/linux/canfd_bus.cc
)

option(DMDRIVER_BUILD_WITH_EXAMPLE "Build dm_driver examples" ON)

if (DMDRIVER_BUILD_WITH_EXAMPLE)
    add_executable(socketcan_demo example/socketcan_demo.cc)
    target_link_libraries(socketcan_demo PRIVATE ${CMAKE_TARGET_NAME})
    add_executable(motor_demo example/motor_demo.cc)
    target_link_libraries(motor_demo PRIVATE ${CMAKE_TARGET_NAME})
endif()