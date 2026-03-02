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
find_package(glfw3 REQUIRED)

set(Torch_DIR /opt/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)

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



add_executable(robot_simul
    robot_simul.cpp
    tasks/policy_runner.cpp
)
target_include_directories(robot_simul SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${manif_INCLUDE_DIRS})
target_include_directories(robot_simul PRIVATE 
    ${MUJOCO_INCLUDE_DIR}
    /opt/openrobots/include
  ${CMAKE_CURRENT_SOURCE_DIR}/tasks
)
target_link_libraries(robot_simul PRIVATE rtfw::rtfw ${TORCH_LIBRARIES} ${MUJOCO_LIBRARY} glfw)
target_link_libraries(robot_simul PRIVATE uWebSockets)
target_link_libraries(robot_simul PRIVATE nlohmann_json::nlohmann_json)




add_executable(robot_control
    robot_control.cpp
    tasks/policy_runner.cpp
)
target_include_directories(robot_control SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS} ${manif_INCLUDE_DIRS})
target_include_directories(robot_control PRIVATE 
    ${MUJOCO_INCLUDE_DIR}
    /opt/openrobots/include
  ${CMAKE_CURRENT_SOURCE_DIR}/tasks
)
target_link_libraries(robot_control PRIVATE rtfw::rtfw ${TORCH_LIBRARIES})
target_link_libraries(robot_control PRIVATE uWebSockets)
target_link_libraries(robot_control PRIVATE nlohmann_json::nlohmann_json)