# apps/monitor_tool/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)
project(rtmonitor)


set(IMGUI_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/imgui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/imgui_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/imgui_tables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/imgui_widgets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/backends/imgui_impl_opengl3.cpp
)

# ImGui를 정적(STATIC) 라이브러리 타겟으로 생성
add_library(imgui_static STATIC ${IMGUI_SOURCES})

# ImGui 라이브러리에 필요한 헤더 경로 설정
target_include_directories(imgui_static PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/../external/imgui/backends
)


# 실행 파일 정의
add_executable(rtmonitor 
                main.cpp
                ui_state.cpp
                graph_renderer.cpp
                stats_renderer.cpp
)

# 필요한 라이브러리 찾기 (GLFW, OpenGL)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# 라이브러리 링크
target_link_libraries(rtmonitor PRIVATE
    rtfw::connect
    imgui_static
    glfw
    ${OPENGL_LIBRARIES}
)
    
set_target_properties(rtmonitor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools)
# add_executable(bridge_sample bridge_sample.cpp)
# target_link_libraries(bridge_sample rtfw::connect::ros2)


# find_package(ament_cmake REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# add_executable(connect_test connect_test.cpp)
# ament_target_dependencies(connect_test nav_msgs sensor_msgs)
# target_link_libraries(connect_test rtfw::connect::ros2)
