cmake_minimum_required(VERSION 3.8)
project(rlcode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================================
# 1. 프로젝트 전체 공통 의존성 설정 (한 번만 실행)
# ===================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(msg_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(Torch_DIR /opt/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

set(MUJOCO_DIR /home/khrlab/.mujoco/mujoco-3.2.0)
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS ${MUJOCO_DIR}/include)
find_library(MUJOCO_LIBRARY libmujoco.so.3.2.0 PATHS ${MUJOCO_DIR}/lib)


# imu 세팅
add_subdirectory(src/IMU/cpp/examples/ez_async_data)
add_subdirectory(src/IMU/cpp/examples/getting_started)

set(SOURCE
        src/IMU/cpp/src/attitude.cpp
        src/IMU/cpp/src/compositedata.cpp
        src/IMU/cpp/src/conversions.cpp
        src/IMU/cpp/src/criticalsection.cpp
        src/IMU/cpp/src/dllvalidator.cpp
        src/IMU/cpp/src/error_detection.cpp
        src/IMU/cpp/src/event.cpp
        src/IMU/cpp/src/ezasyncdata.cpp
        src/IMU/cpp/src/memoryport.cpp
        src/IMU/cpp/src/packet.cpp
        src/IMU/cpp/src/packetfinder.cpp
        src/IMU/cpp/src/port.cpp
        src/IMU/cpp/src/position.cpp
        src/IMU/cpp/src/rtcmlistener.cpp
        src/IMU/cpp/src/rtcmmessage.cpp
        src/IMU/cpp/src/searcher.cpp
        src/IMU/cpp/src/sensors.cpp
        src/IMU/cpp/src/serialport.cpp
        src/IMU/cpp/src/thread.cpp
        src/IMU/cpp/src/types.cpp
        src/IMU/cpp/src/util.cpp
        src/IMU/cpp/src/utilities.cpp
        src/IMU/cpp/src/vntime.cpp
        src/IMU/cpp/include/vn/exceptions.h
        src/IMU/cpp/include/vn/matrix.h
        src/IMU/cpp/include/vn/attitude.h
        src/IMU/cpp/include/vn/boostpython.h
        src/IMU/cpp/include/vn/dllvalidator.h
        src/IMU/cpp/include/vn/signal.h
        src/IMU/cpp/include/vn/error_detection.h
        src/IMU/cpp/include/vn/position.h
        src/IMU/cpp/include/vn/registers.h
        src/IMU/cpp/include/vn/rtcmlistener.h
        src/IMU/cpp/include/vn/rtcmmessage.h
        src/IMU/cpp/include/vn/utilities.h
        src/IMU/cpp/include/vn/memoryport.h
        src/IMU/cpp/include/vn/nocopy.h
        src/IMU/cpp/include/vn/compositedata.h
        src/IMU/cpp/include/vn/criticalsection.h
        src/IMU/cpp/include/vn/compiler.h
        src/IMU/cpp/include/vn/sensors.h
        src/IMU/cpp/include/vn/searcher.h
        src/IMU/cpp/include/vn/event.h
        src/IMU/cpp/include/vn/ezasyncdata.h
        src/IMU/cpp/include/vn/serialport.h
        src/IMU/cpp/include/vn/export.h
        src/IMU/cpp/include/vn/vector.h
        src/IMU/cpp/include/vn/vntime.h
        src/IMU/cpp/include/vn/packetfinder.h
        src/IMU/cpp/include/vn/conversions.h
        src/IMU/cpp/include/vn/types.h
        src/IMU/cpp/include/vn/int.h
        src/IMU/cpp/include/vn/thread.h
        src/IMU/cpp/include/vn/mock.h
        src/IMU/cpp/include/vn/port.h
        src/IMU/cpp/include/vn/util.h
        src/IMU/cpp/include/vn/consts.h
        src/IMU/cpp/include/vn/packet.h)
  
add_library(libvncxx ${SOURCE})  
target_include_directories(libvncxx PRIVATE src/IMU/cpp/include)

find_package(Eigen3 REQUIRED)


# ===================================================================
# 2. C++ 노드 추가를 위한 함수 정의
# ===================================================================
function(code_executable NODE_NAME SOURCE_FILE)
    add_executable(${NODE_NAME} ${SOURCE_FILE})
    ament_target_dependencies(${NODE_NAME}
        "rclcpp"
        "ament_index_cpp"
        "yaml-cpp"
        "msg_interfaces"
        Eigen3
    )
    target_include_directories(${NODE_NAME} PRIVATE 
        ${MUJOCO_INCLUDE_DIR} 
        ${EIGEN3_INCLUDE_DIR}
        )
    target_link_libraries(${NODE_NAME}
        ${MUJOCO_LIBRARY}    
        ${YAML_CPP_LIBRARIES}
        ${TORCH_LIBRARIES}
        glfw
        pthread
        rt
        ${EIGEN3_LIBRARIES}
    )
    install(TARGETS
        ${NODE_NAME}
        DESTINATION lib/${PROJECT_NAME}
    )
endfunction()

function(imucode_executable NODE_NAME SOURCE_FILE)
    add_executable(${NODE_NAME} ${SOURCE_FILE})
    ament_target_dependencies(${NODE_NAME}
        "rclcpp"
        "ament_index_cpp"
        "yaml-cpp"
        "msg_interfaces"
        Eigen3
    )
    target_include_directories(${NODE_NAME} PRIVATE 
        ${MUJOCO_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        src/IMU/cpp/include
    )
    target_link_libraries(${NODE_NAME}
        ${MUJOCO_LIBRARY}    
        ${YAML_CPP_LIBRARIES}
        ${TORCH_LIBRARIES}
        glfw
        pthread
        rt
        libvncxx
        ${EIGEN3_LIBRARIES}
    )
    install(TARGETS
        ${NODE_NAME}
        DESTINATION lib/${PROJECT_NAME}
    )
endfunction()
# ===================================================================
# 3. 함수를 사용하여 C++ 노드 추가
# ===================================================================
code_executable(policy_code src/policy_code.cpp)
code_executable(policy_code1 src/policy_code1.cpp)
code_executable(policy_code2 src/policy_code2.cpp)

code_executable(mujoco_sim src/mujoco_sim.cpp)

code_executable(motor_test1 src/motor_test1.cpp)
code_executable(motor_test2 src/motor_test2.cpp)
code_executable(motor_test3 src/motor_test3.cpp)
code_executable(motor_test4 src/motor_test4.cpp)
code_executable(motor_test5 src/motor_test5.cpp)

imucode_executable(imu_test1 src/imu_test1.cpp)
imucode_executable(imu_test2 src/imu_test2.cpp)
imucode_executable(imu_test3 src/imu_test3.cpp)
imucode_executable(imu_test4 src/imu_test4.cpp)
# ===================================================================
# 4. 기타 스크립트 및 리소스 설치
# ===================================================================
install(PROGRAMS
  src/controller.py
  src/obstorviz.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  urdf
  mjcf
  config
  launch
  policy
  DESTINATION share/${PROJECT_NAME}
)


ament_package()