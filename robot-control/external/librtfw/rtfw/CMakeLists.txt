# rtfw/CMakeLists.txt

# Boost 라이브러리 찾기 (SPSC 큐를 위해 필요)
# 헤더 온리 컴포넌트만 사용하므로 컴포넌트 명시는 생략합니다.
# find_package(Boost REQUIRED)
# find_package(spdlog REQUIRED)


# rtfw 코어 라이브러리를 STATIC으로 정의합니다.
add_library(rtfw STATIC
    src/rt_framework.cpp
    src/task.cpp
    src/timeline.cpp
    src/shm_context.cpp
    src/shm_allocator.cpp
    src/parameter_manager.cpp
    # src/file_archive.cpp
)

# 모던 CMake 스타일의 별칭(alias) 생성
add_library(rtfw::rtfw ALIAS rtfw)

# 공개(PUBLIC) 헤더 경로를 지정합니다.
target_include_directories(rtfw PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# rtfw 라이브러리의 의존성을 설정합니다.
target_link_libraries(rtfw
    PUBLIC
        # rtfw의 공개 헤더(non_rt_api.h 등)가 client와 common의 헤더를
        # 참조하므로, rtfw를 사용하는 타겟도 이들을 알아야 합니다.
        rtfw::connect
        rtfw::common
        spdlog::spdlog_header_only
        # find_package가 설정한 Boost_INCLUDE_DIRS 변수를 통해 헤더 경로 추가
        # ${Boost_INCLUDE_DIRS}
    PRIVATE
        # 프레임워크 내부 구현에만 필요한 시스템 라이브러리
        pthread
        rt
        # spdlog::spdlog
)

# 개발 중 유용한 컴파일러 경고 옵션을 추가합니다.
if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(rtfw PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        -g # 디버깅 심볼 생성
    )
endif()