cmake_minimum_required(VERSION 3.8)
project(mujoco_rl)

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
    )
    target_include_directories(${NODE_NAME} PRIVATE ${MUJOCO_INCLUDE_DIR} ${ONNXRUNTIME_INCLUDE_PATH})
    target_link_libraries(${NODE_NAME}
        ${MUJOCO_LIBRARY}    
        ${YAML_CPP_LIBRARIES}
        ${TORCH_LIBRARIES}
        glfw
        GL
    )
    install(TARGETS
        ${NODE_NAME}
        DESTINATION lib/${PROJECT_NAME}
    )
endfunction()


# ===================================================================
# 3. 함수를 사용하여 C++ 노드 추가
# ===================================================================
code_executable(mujoco_ros1 src/mujoco_ros1.cpp)
code_executable(mujoco_ros2 src/mujoco_ros2.cpp)
code_executable(mujoco_ros3 src/mujoco_ros3.cpp)
code_executable(mujoco_ros4 src/mujoco_ros4.cpp)
code_executable(mujoco_ros5 src/mujoco_ros5.cpp)
code_executable(mujoco_ros6 src/mujoco_ros6.cpp)
code_executable(pttest src/pttest.cpp)
code_executable(policy_runner1 src/policy_runner1.cpp)
code_executable(policy_runner2 src/policy_runner2.cpp)
code_executable(policy_runner3 src/policy_runner3.cpp)


# ===================================================================
# 4. 기타 스크립트 및 리소스 설치
# ===================================================================
install(PROGRAMS
  src/test_code.py
  src/mujoco_ros_code.py
  src/controller_test.py
  src/controller_ros_code.py
  src/joy_sub_test.py
  src/policy_ros_code.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  mjcf
  config
  launch
  policy
  DESTINATION share/${PROJECT_NAME}
)

ament_package()