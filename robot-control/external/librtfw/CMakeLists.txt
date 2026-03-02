cmake_minimum_required(VERSION 3.16)
project(RTFrameworkProject CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Boost 라이브러리 찾기
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
add_subdirectory(third_party/spdlog)

# 라이브러리와 애플리케이션 서브디렉토리 추가
add_subdirectory(rtfw-common)
add_subdirectory(rtfw-connect)
add_subdirectory(rtfw)
# add_subdirectory(apps)
add_subdirectory(tools)

message(STATUS "Project configured successfully.")
message(STATUS "To build, run: cmake --build .")