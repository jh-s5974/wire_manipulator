add_executable(poc_app
    main.cpp
    # poc_tasks.h는 헤더 온리이므로 소스 리스트에는 main.cpp만 필요
)

# poc_app은 rtfw 라이브러리에 의존
# rtfw::rtfw ALIAS를 사용하면 의존성(헤더 경로, 링크 라이브러리)이 자동으로 전파됨
target_link_libraries(poc_app PRIVATE rtfw::rtfw)