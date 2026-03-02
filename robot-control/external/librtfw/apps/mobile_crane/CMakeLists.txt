

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)


find_package(OpenCV REQUIRED)
find_path(Spinnaker_INCLUDE_DIR Spinnaker.h 
    PATHS /opt/spinnaker /usr /usr/local
    PATH_SUFFIXES include)
find_library(Spinnaker_LIBS NAMES Spinnaker 
  PATHS /opt/spinnaker /usr /usr/local
  PATH_SUFFIXES lib)
find_package(Eigen3 REQUIRED)
find_package(manif REQUIRED)

add_executable(crane
    main.cpp
)

target_link_libraries(crane rtfw::rtfw ${OpenCV_LIBS} ${Spinnaker_LIBS})
target_include_directories(crane PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIR} ${Spinnaker_INCLUDE_DIR})
target_include_directories(crane SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${manif_INCLUDE_DIRS})

add_executable(json_exporter
    json_exporter.cpp
)
target_link_libraries(json_exporter rtfw::rtfw)
target_include_directories(json_exporter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(json_exporter SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${manif_INCLUDE_DIRS})



# ament_target_dependencies(crane rclcpp std_msgs geometry_msgs sensor_msgs)