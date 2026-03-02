# rtfw-connect/CMakeLists.txt

add_library(rtfw-connect STATIC
    src/shm_querier.cpp
    src/shm_connector.cpp
    src/shm_controller.cpp
)

# 모던 CMake 스타일의 별칭(alias) 생성
add_library(rtfw::connect ALIAS rtfw-connect)

# 이 라이브러리 사용자에게 공개할 헤더 경로를 지정합니다.
target_include_directories(rtfw-connect PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(rtfw-connect
    PUBLIC
        rtfw::common  # 공용 데이터 구조체 사용
    PRIVATE
        pthread       # POSIX 스레드 (필요 시)
        rt            # clock_nanosleep, shm_open 등을 위해 필요
)

option(BUILD_ROS2 "Build ROS2 bridge" OFF)
if(BUILD_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rosidl_generator_cpp REQUIRED)

  add_library(rtfw-connect-ros2 STATIC
      src/ros2/bridge.cpp
      src/ros2/remote_client.cpp
  )
    target_include_directories(rtfw-connect-ros2 PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include_ros2
    )
  target_link_libraries(rtfw-connect-ros2 PUBLIC rtfw::connect)
  add_library(rtfw::connect::ros2 ALIAS rtfw-connect-ros2)
    
    # target_link_libraries(rtfw-connect-ros2
    #     PUBLIC
    #         ${rclcpp_LIBRARIES}
    #         ${std_msgs_LIBRARIES}
    # )

    # target_include_directories(rtfw-connect-ros2
    #     PUBLIC
    #         ${rclcpp_INCLUDE_DIRS}
    #         ${std_msgs_INCLUDE_DIRS}
    # )
    ament_target_dependencies(rtfw-connect-ros2 PUBLIC rclcpp std_msgs rosidl_generator_cpp)
endif()
