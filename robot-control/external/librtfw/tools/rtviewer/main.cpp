// apps/monitor_tool/main.cpp

#include <iostream>      
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>


#include <chrono>
#include <string>
#include <vector>
#include <numeric>
#include <queue>
#include <set>
#include <cmath>
#include <algorithm>
#include <any>
#include <cstring>
#include <thread>
#include "editor_ui.h" // GUI 렌더링 함수가 포함된 헤더
// #include "rtfw_common/file_archive.h"
#include "rtfw_common/FileBlackbox.h"



using namespace std::chrono_literals;
using namespace rtfw::common;


static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

rtfw::blackbox::FileBlackbox g_blackbox;
uint64_t log_frequency;
std::map<uint64_t, std::vector<LogEntryView>> data_cache;
bool contents_init = false;
static std::string g_initial_filepath;


int main(int argc, char** argv) {

    if (argc != 2) {
        // 인자가 없거나 너무 많으면 사용법 출력 후 종료
        std::cerr << "Usage: " << argv[0] << " <path_to_log_file>" << std::endl;
        return EXIT_FAILURE;
    }

    // // 2. 파일 경로를 저장
    g_initial_filepath = argv[1];

    // 3. 애플리케이션 시작 전, 파일 자동 로드 시도
    bool is_load = false;
    try {
        g_blackbox.start(g_initial_filepath, rtfw::blackbox::Mode::REPLAY);
        is_load = g_blackbox.initialize_metadata(std::map<uint64_t, std::string>(), 0);
        data_cache.clear();
        auto keymap = g_blackbox.getKeymapCache();
        for (auto i=0; i <= g_blackbox.getLastTick(); i++) {
            g_blackbox.onTick(i);
            std::vector<LogEntryView> views;
            for (auto& it: *keymap) {
                if (!it.second.update)
                    continue;
                it.second.update = false;

                views.push_back({
                    it.second.start_tick,
                    it.second.end_tick,
                    it.first,
                    it.second.type_hash,
                    it.second.data
                });
            }
            data_cache.insert({i, views});                
        }
        contents_init = true;
    } catch (...) {
        g_blackbox.shutdown();
    }
    if (!is_load) {
        std::cerr << "Error: Failed to open log file: " << g_initial_filepath << std::endl;
        // 파일을 열 수 없더라도 뷰어는 빈 상태로 실행할 수 있도록 계속 진행
    }


    // GLFW 초기화
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    // 윈도우 생성
    GLFWwindow* window = glfwCreateWindow(500, 700, "RTFW Log Viewer", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // ImGui 컨텍스트 초기화
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

    // ImGui 스타일 설정
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight(); 

    // 렌더러 백엔드 초기화
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    auto tick = std::chrono::steady_clock::now();
    // 메인 루프
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // ImGui 새 프레임 시작
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ----------------- UI 코드 시작 -----------------

        render_log_file_editor_window();

        // ----------------- UI 코드 종료 -----------------

        // 렌더링
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        std::this_thread::sleep_until(tick + 50ms);
        tick += 50ms;
    }

    // 종료 처리
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}