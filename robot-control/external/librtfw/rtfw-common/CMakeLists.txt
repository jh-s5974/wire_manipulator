# rtfw-common/CMakeLists.txt

# rtfw-common은 .cpp 파일이 없는 헤더 온리 라이브러리이므로
# 'INTERFACE' 타입으로 정의합니다.
# INTERFACE 라이브러리는 자신을 사용하는 타겟에게 속성(예: include 경로)만 전달합니다.

# find_package(yaml-cpp REQUIRED)

add_library(rtfw-common STATIC
    src/FileBlackbox.cpp
)
# rtfw::common 이라는 모던 CMake 스타일의 별칭(alias)을 생성합니다.
# 이를 통해 다른 타겟에서 'rtfw-common' 대신 'rtfw::common'으로 참조할 수 있습니다.
add_library(rtfw::common ALIAS rtfw-common)

target_link_libraries(rtfw-common PUBLIC
    yaml-cpp
)

# 이 라이브러리를 사용하는 타겟들이 헤더를 찾을 수 있도록
# 공개(INTERFACE) include 경로를 지정합니다.
# $<BUILD_INTERFACE:...>는 빌드 트리 내에서의 경로를 의미합니다.
# ${CMAKE_CURRENT_SOURCE_DIR}는 이 CMakeLists.txt 파일이 있는 디렉토리입니다.
target_include_directories(rtfw-common PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)