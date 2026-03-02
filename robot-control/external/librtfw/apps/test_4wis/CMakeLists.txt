cmake_minimum_required(VERSION 3.8)
project(test_4wis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
# find_package(Qt5Core REQUIRED)
# find_package(Qt5Widgets REQUIRED)
# find_package(osqp REQUIRED)
# find_package(OsqpEigen REQUIRED)

# add_library(ipopt SHARED IMPORTED)
# set_target_properties(ipopt PROPERTIES IMPORTED_LOCATION /lib/libipopt.so)


# add_library(modbus SHARED IMPORTED)
# set_target_properties(modbus PROPERTIES IMPORTED_LOCATION "/usr/lib/x86_64-linux-gnu/libmodbus.so")

find_path(Spinnaker_INCLUDE_DIR Spinnaker.h 
    PATHS /opt/spinnaker /usr /usr/local
    PATH_SUFFIXES include)
find_library(Spinnaker_LIBS NAMES Spinnaker 
  PATHS /opt/spinnaker /usr /usr/local
  PATH_SUFFIXES lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# add_executable(vision src/vision.cpp)
# target_link_libraries(vision ${Spinnaker_LIBS})
# target_include_directories(vision PUBLIC ${Spinnaker_INCLUDE_DIR})
# ament_target_dependencies(vision rclcpp geometry_msgs OpenCV Eigen3 tf2 tf2_ros)


# add_executable(vision_calib src/vision_calib.cpp)
# target_link_libraries(vision_calib ${Spinnaker_LIBS})
# target_include_directories(vision_calib PUBLIC ${Spinnaker_INCLUDE_DIR})
# ament_target_dependencies(vision_calib OpenCV)

# add_executable(gui src/gui.cpp)
# ament_target_dependencies(gui rclcpp std_msgs geometry_msgs Eigen3 Qt5Core Qt5Widgets)

# add_executable(imu src/imu.cpp)
# ament_target_dependencies(imu rclcpp std_msgs geometry_msgs Eigen3)

# add_executable(joystick src/joystick.cpp)
# ament_target_dependencies(joystick rclcpp std_msgs geometry_msgs)

# add_executable(mecanum src/mecanum.cpp)
# ament_target_dependencies(mecanum rclcpp std_msgs geometry_msgs Eigen3)

# add_executable(wheel_if src/wheel_if.cpp)
# ament_target_dependencies(wheel_if rclcpp std_msgs std_srvs)

# add_executable(crane_if src/crane_if.cpp)
# target_include_directories(crane_if PRIVATE "/usr/include/modbus")
# target_link_libraries(crane_if modbus)
# ament_target_dependencies(crane_if rclcpp std_msgs geometry_msgs Eigen3 std_srvs)

# add_executable(daq src/daq.cpp)
# ament_target_dependencies(daq rclcpp std_msgs geometry_msgs Eigen3 std_srvs)

# add_executable(crane_pos_control src/crane_pos_control.cpp)
# ament_target_dependencies(crane_pos_control rclcpp std_msgs std_srvs)

# add_executable(tracking_control src/tracking_control.cpp)
# target_link_libraries(tracking_control OsqpEigen::OsqpEigen)
# ament_target_dependencies(tracking_control rclcpp std_msgs geometry_msgs std_srvs Eigen3)

# add_executable(crane src/crane.cpp)
# ament_target_dependencies(crane rclcpp std_msgs geometry_msgs std_srvs Eigen3)

# add_executable(abnormal_detector src/abnormal_detector.cpp)
# ament_target_dependencies(abnormal_detector rclcpp std_msgs geometry_msgs std_srvs Eigen3)

add_executable(standalone src/standalone.cpp)
target_link_libraries(standalone ${Spinnaker_LIBS} ${OpenCV_LIBS} ${Eigen3_LIBS})
target_include_directories(standalone PRIVATE ${Spinnaker_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR} ${Eigen3_INCLUDE_DIR})
ament_target_dependencies(standalone rclcpp std_msgs geometry_msgs dynamixel_sdk)


install(TARGETS
  # vision
  # vision_calib
  # gui
  # imu
  # joystick
  # mecanum
  # wheel_if
  # crane_if
  # daq
  # crane_pos_control
  # tracking_control
  # abnormal_detector
  # crane
  standalone
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
