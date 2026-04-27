include(FetchContent)

FetchContent_Declare(
    aimrt
    GIT_REPOSITORY https://gitee.com/agiros/AimRT.git
    GIT_TAG v1.4.0
)

FetchContent_GetProperties(aimrt)

function(get_aimrt)
  FetchContent_GetProperties(aimrt)
  if(NOT aimrt_POPULATED)
    set(AIMRT_BUILD_RUNTIME ON)

    set(AIMRT_BUILD_WITH_PROTOBUF ON)

    if(AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2)
      set(AIMRT_BUILD_WITH_ROS2 ON)
    endif()

    set(AIMRT_BUILD_NET_PLUGIN ON)
    set(AIMRT_BUILD_TIME_MANIPULATOR_PLUGIN ON)
    set(AIMRT_BUILD_ECHO_PLUGIN ON)

    FetchContent_MakeAvailable(aimrt)
  endif()
endfunction()

get_aimrt()
