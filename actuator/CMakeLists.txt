set(CUR_TARGET_NAME "actuator_controller")

if(GENISYS_CORE_BUILD_ARM STREQUAL "geniarm")
  if(DEFINED GENISYS_CORE_BUILD_EXAMPLES)
    set(DMDRIVER_BUILD_WITH_EXAMPLE ${GENISYS_CORE_BUILD_EXAMPLES})
  endif()
  add_subdirectory(third_party/dm_driver)
  set(ARM_SOURCES
    src/actuator_manager.cc
    src/geniarm_ctrl.cc
    src/geniarm_gripper.cc
  )
  set(ARM_LIBS dm_driver)
else()
  message(FATAL_ERROR "Unsupported GENISYS_CORE_BUILD_ARM: ${GENISYS_CORE_BUILD_ARM} (supported: geniarm)")
endif()

add_library(${CUR_TARGET_NAME} STATIC)

target_sources(${CUR_TARGET_NAME} PRIVATE ${ARM_SOURCES})

target_include_directories(${CUR_TARGET_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${CUR_TARGET_NAME}>
)

target_link_libraries(${CUR_TARGET_NAME} PUBLIC 
  ${ARM_LIBS}
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