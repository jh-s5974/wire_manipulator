find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ros_test
    main.cpp
)

target_link_libraries(ros_test rtfw::rtfw)
ament_target_dependencies(ros_test rclcpp std_msgs)
