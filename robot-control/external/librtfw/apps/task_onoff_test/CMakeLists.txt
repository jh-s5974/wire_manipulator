cmake_minimum_required(VERSION 3.16)
project(task_onoff_test)

# C++17 표준 요구
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# rtfw 및 관련 라이브러리 링크
add_executable(task_onoff_test
    main.cpp
)

target_link_libraries(task_onoff_test
    PRIVATE
        rtfw::rtfw
        rtfw-common
        pthread
        rt
)

target_include_directories(task_onoff_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/../../rtfw/include
        ${PROJECT_SOURCE_DIR}/../../rtfw-common/include
)
