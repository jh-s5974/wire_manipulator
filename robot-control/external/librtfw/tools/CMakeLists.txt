cmake_minimum_required(VERSION 3.16)

# Build helper tools under tools/
add_executable(rtcli
    rtcli/src/main.cpp
    rtcli/src/command_handler.cpp
    rtcli/src/scheduler_commands.cpp
    rtcli/src/record_commands.cpp
    rtcli/src/task_commands.cpp
    rtcli/src/data_commands.cpp
)

target_include_directories(rtcli PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtfw-common/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtfw/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../rtfw-connect/include
    ${CMAKE_CURRENT_SOURCE_DIR}/rtcli/include
)

target_link_libraries(rtcli PRIVATE rt rtfw-connect pthread)

set_target_properties(rtcli PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools)

# Create a symbolic link for backward compatibility if desired
# add_custom_target(shm_control ALL
#     COMMAND ${CMAKE_COMMAND} -E create_symlink rtcli shm_control
#     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tools
# )

add_subdirectory(rtmonitor)
add_subdirectory(rtviewer)