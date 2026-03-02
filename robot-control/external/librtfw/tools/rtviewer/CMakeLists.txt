# apps/monitor_tool/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)
project(rtviewer)



# 실행 파일 정의
add_executable(rtviewer 
                main.cpp
                editor_ui.cpp
)

# 필요한 라이브러리 찾기 (GLFW, OpenGL)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# 라이브러리 링크
target_link_libraries(rtviewer PRIVATE
    rtfw::connect
    imgui_static
    glfw
    ${OPENGL_LIBRARIES}
)