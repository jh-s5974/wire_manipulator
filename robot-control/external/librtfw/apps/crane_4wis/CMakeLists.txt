

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)


set(CMAKE_PREFIX_PATH "/home/pms/onnxruntime-1.23.2" ${CMAKE_PREFIX_PATH})
find_package(onnxruntime REQUIRED)

find_package(OpenCV REQUIRED)
find_path(Spinnaker_INCLUDE_DIR Spinnaker.h 
    PATHS /opt/spinnaker /usr /usr/local
    PATH_SUFFIXES include)
find_library(Spinnaker_LIBS NAMES Spinnaker 
  PATHS /opt/spinnaker /usr /usr/local
  PATH_SUFFIXES lib)
find_package(Eigen3 REQUIRED)
find_package(manif REQUIRED)

add_executable(run_crane4wis
    main.cpp
)

target_link_libraries(run_crane4wis rtfw::rtfw ${OpenCV_LIBS} ${Spinnaker_LIBS} onnxruntime::onnxruntime)
target_include_directories(run_crane4wis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIR} ${Spinnaker_INCLUDE_DIR})
target_include_directories(run_crane4wis SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${manif_INCLUDE_DIRS})

add_executable(json_exporter_crane4wis
    json_exporter.cpp
)
target_link_libraries(json_exporter_crane4wis rtfw::rtfw)
target_include_directories(json_exporter_crane4wis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(json_exporter_crane4wis SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${manif_INCLUDE_DIRS})



# ament_target_dependencies(crane rclcpp std_msgs geometry_msgs sensor_msgs)