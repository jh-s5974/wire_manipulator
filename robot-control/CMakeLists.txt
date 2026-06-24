cmake_minimum_required(VERSION 3.16)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif()

project(robot_control2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
include(FetchContent)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
  GIT_SHALLOW ON
)
FetchContent_MakeAvailable(nlohmann_json)

find_package(Eigen3 REQUIRED)
find_package(glfw3  REQUIRED)

set(MUJOCO_DIR $ENV{HOME}/.mujoco/mujoco-3.2.0)
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS ${MUJOCO_DIR}/include)
find_library(MUJOCO_LIBRARY libmujoco.so.3.2.0 PATHS ${MUJOCO_DIR}/lib)

if(NOT MUJOCO_INCLUDE_DIR)
  message(FATAL_ERROR "MuJoCo headers not found. Expected at ${MUJOCO_DIR}/include")
endif()
if(NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo library not found. Expected at ${MUJOCO_DIR}/lib")
endif()

add_subdirectory(external)

# ── ROS2 조인트 명령 브리지 (선택) ──
# `source /opt/ros/humble/setup.bash` 후 cmake 를 (재)실행하면 자동 활성화된다.
# 없으면 ROS2 없이 빌드되어 기존 동작(GUI 전용)이 그대로 유지된다.
# NOTE: 반드시 add_subdirectory(external) 뒤에 둔다 — librtfw 의 vendored spdlog 타깃
#       (spdlog::spdlog)이 먼저 생성되어야, rclcpp 가 끌어오는 시스템 spdlog config 의
#       if(NOT TARGET) 가드가 동작해 ALIAS 중복 정의 충돌을 피한다.
option(WITH_ROS2 "Build with ROS2 joint command bridge (/joint_command)" ON)
set(ROS2_ENABLED FALSE)
if(WITH_ROS2)
  find_package(ament_cmake QUIET)
  find_package(rclcpp QUIET)
  find_package(std_msgs QUIET)
  if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND)
    set(ROS2_ENABLED TRUE)
    message(STATUS "ROS2 found — joint command bridge ENABLED (topic /joint_command)")
  else()
    message(STATUS "ROS2 not found — joint command bridge DISABLED. "
                   "Run 'source /opt/ros/humble/setup.bash' then re-run cmake to enable.")
  endif()
endif()

# ── robot_simul: MuJoCo 시뮬레이션 (매니퓰레이터) ──
add_executable(robot_simul robot_simul.cpp)
target_include_directories(robot_simul SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(robot_simul PRIVATE
    ${MUJOCO_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks
)
target_link_libraries(robot_simul PRIVATE rtfw::rtfw ${MUJOCO_LIBRARY} glfw)
target_link_libraries(robot_simul PRIVATE uWebSockets)
target_link_libraries(robot_simul PRIVATE nlohmann_json::nlohmann_json)

# ── robot_control: 실제 하드웨어 제어 ──
add_executable(robot_control robot_control.cpp)
target_include_directories(robot_control SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories(robot_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks
)
target_link_libraries(robot_control PRIVATE rtfw::rtfw)
target_link_libraries(robot_control PRIVATE uWebSockets)
target_link_libraries(robot_control PRIVATE nlohmann_json::nlohmann_json)

# ── ROS2 브리지 연결 (활성화된 경우에만; WITH_ROS2 매크로로 main 에서 가드) ──
# ament_target_dependencies 는 plain 형식 target_link_libraries 를 쓰는데, 이 프로젝트는
# 위에서 keyword(PRIVATE) 형식을 쓰므로 혼용 에러가 난다. 대신 imported 타깃을 직접 링크
# (rclcpp::rclcpp / ${std_msgs_TARGETS})하면 include/lib 가 INTERFACE 속성으로 전파된다.
if(ROS2_ENABLED)
  foreach(tgt robot_simul robot_control)
    target_compile_definitions(${tgt} PRIVATE WITH_ROS2)
    target_link_libraries(${tgt} PRIVATE rclcpp::rclcpp ${std_msgs_TARGETS})
  endforeach()
endif()
