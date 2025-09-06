cmake_minimum_required(VERSION 3.24)
project(serial LANGUAGES CXX)

# source files
set(SOURCES src/serial.cc)
# header files
set(INCLUDE_DIRS include)

if(APPLE)
    # If MacOS
    list(APPEND SOURCES src/impl/unix.cc)
    list(APPEND SOURCES src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If Unix
    list(APPEND SOURCES src/impl/unix.cc)
    list(APPEND SOURCES src/impl/list_ports/list_ports_linux.cc)
else()
    # If Windows
    list(APPEND SOURCES src/impl/win.cc)
    list(APPEND SOURCES src/impl/list_ports/list_ports_win.cc)
endif()

# Create a static library
add_library(${PROJECT_NAME} STATIC)

# system libraries
if(APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE  ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE rt pthread)
else() 
    target_link_libraries(${PROJECT_NAME} PRIVATE setupapi)
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
    ${INCLUDE_DIRS}
)

target_sources(${PROJECT_NAME} PRIVATE
    ${SOURCES}
)

# add example executable
add_executable(serial_example examples/serial_example.cc)
target_link_libraries(serial_example PRIVATE ${PROJECT_NAME})


install(DIRECTORY include/
    DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin 
)

install(TARGETS serial_example
    RUNTIME DESTINATION bin 
)
