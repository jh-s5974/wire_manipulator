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
#include <rtfw_connect/client.h> 
#include "graph_renderer.h"
#include "stats_renderer.h"
#include "ui_state.h"


using namespace std::chrono_literals;
using namespace rtfw::common;


static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
};


struct Signal{};
std::any interpret_raw_data(const void* data_ptr, size_t data_size, size_t type_hash) {
    if (!data_ptr || data_size == 0) {
        return {}; // 빈 std::any 반환
    }

    // --- 타입 해시를 이용한 분기 ---
    
    if (type_hash == typeid(bool).hash_code() && data_size == sizeof(bool)) {
        return *static_cast<const bool*>(data_ptr);
    }
    if (type_hash == typeid(int).hash_code() && data_size == sizeof(int)) {
        return *static_cast<const int*>(data_ptr);
    }
    if (type_hash == typeid(float).hash_code() && data_size == sizeof(float)) {
        return *static_cast<const float*>(data_ptr);
    }
    if (type_hash == typeid(double).hash_code() && data_size == sizeof(double)) {
        return *static_cast<const double*>(data_ptr);
    }
    
    if (type_hash == typeid(Signal).hash_code()) {
        // Signal은 내용이 없으므로, 타입 자체를 반환
        return Signal{};
    }

    // --- 아는 타입이 아닐 경우 ---
    // 원시 바이트를 std::vector<char>로 복사하여 반환
    std::vector<char> raw_bytes(data_size);
    memcpy(raw_bytes.data(), data_ptr, data_size);
    return raw_bytes;
}

void render_data_inspector_window(SharedMemoryQuerier& querier) {
    // descriptors 목록은 한 번만 가져와 캐싱
    static std::vector<DataBlockDescriptor> descriptors = querier.getAllDescriptors();
    static int selected_desc_idx = -1; // 선택된 descriptor의 인덱스
    static std::string display_data_string = "Select a data key to inspect."; // 표시할 최종 문자열

    // --- [1. 창의 위치와 크기 설정] ---
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    // 화면 좌상단에 위치, 너비는 화면의 30%, 높이는 50%
    ImGui::SetNextWindowPos(viewport->WorkPos + ImVec2(viewport->WorkSize.x * 0.8f, viewport->WorkSize.y * 0.1f));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.2f, viewport->WorkSize.y * 0.4f));

    // --- [2. 창 이동 및 크기 조절 방지 플래그] ---
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("Data Inspector", nullptr, window_flags)) {
        // --- 1. 데이터 키 선택 콤보박스 ---
        const char* preview_value = (selected_desc_idx >= 0) ? descriptors[selected_desc_idx].key : "Select Data Key...";
        static bool is_dragging_in_combo = false;
        if (ImGui::BeginCombo("##DataKeysCombo", preview_value)) {
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left, ImGui::GetIO().MouseDragThreshold))
                is_dragging_in_combo = true;
            
            for (int i = 0; i < descriptors.size(); ++i) {
                const bool is_selected = (selected_desc_idx == i);
                if (ImGui::Selectable(descriptors[i].key, is_selected, is_dragging_in_combo? ImGuiSelectableFlags_NoAutoClosePopups: 0) && !is_dragging_in_combo) {
                    selected_desc_idx = i;
                }
                if (is_selected) ImGui::SetItemDefaultFocus();
            }

            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem))
                ImGui::SetScrollY(ImGui::GetScrollY() - ImGui::GetIO().MouseDelta.y);
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
                is_dragging_in_combo = false;

            ImGui::EndCombo();
        }

        ImGui::Separator();

        // --- 2. 선택된 키의 데이터 조회 및 표시 ---
        if (selected_desc_idx >= 0) {
            const DataBlockDescriptor& selected_desc = descriptors[selected_desc_idx];
            
            ImGui::Text("Key: %s", selected_desc.key);
            // ImGui::Text("Type: %s", type_name.data()); // string_view의 data()로 char* 얻기
            ImGui::Text("Size: %zu bytes", selected_desc.data_size);
            ImGui::Separator();
        
            std::any interpreted_data;
            try {
                // Querier로 최신 데이터 포인터를 얻어옴
                const void* raw_ptr = querier.accessRawData(selected_desc.key);
                
                if (raw_ptr) {
                    // [핵심] 헬퍼 함수를 호출하여 데이터 해석
                    interpreted_data = interpret_raw_data(raw_ptr, selected_desc.data_size, selected_desc.type_hash);
                }
            } catch(const std::exception& e) {
                ImGui::TextColored(ImVec4(1,0,0,1), "Error reading data: %s", e.what());
            }

            // --- 해석된 데이터를 바탕으로 UI 렌더링 ---
            if (!interpreted_data.has_value()) {
                ImGui::Text("No data available.");
            } 
            else if (auto* data = std::any_cast<bool>(&interpreted_data)) {
                ImGui::Text("Type: bool");
                ImGui::Separator();
                ImGui::Text("Value: %s", *data? "true": "false");
            }
            else if (auto* data = std::any_cast<int>(&interpreted_data)) {
                ImGui::Text("Type: int");
                ImGui::Separator();
                ImGui::Text("Value: %d", *data);
            }
            else if (auto* data = std::any_cast<float>(&interpreted_data)) {
                ImGui::Text("Type: float");
                ImGui::Separator();
                ImGui::Text("Value: %f", *data);
            }
            else if (auto* data = std::any_cast<double>(&interpreted_data)) {
                ImGui::Text("Type: double");
                ImGui::Separator();
                ImGui::Text("Value: %lf", *data);
            }
            // 3. Signal 타입인지 확인
            else if (std::any_cast<Signal>(&interpreted_data)) {
                ImGui::Text("Type: Signal");
                ImGui::Separator();
                ImGui::Text("Signal");
            }
            // 4. 모르는 타입 (std::vector<char>로 반환된 경우)
            else if (auto* raw_bytes = std::any_cast<std::vector<char>>(&interpreted_data)) {
                ImGui::Text("Type: Unknown (raw bytes)");
                ImGui::Separator();
                // 16진수 뷰어 등 구현
                ImGui::Text("Size: %zu bytes", raw_bytes->size());
                // ...
            }

            ImGui::Text("Live Data Content:");
            // 고정된 높이(예: 8줄)를 가진 스크롤 가능한 자식 창 생성
            float child_height = ImGui::GetTextLineHeightWithSpacing() * 8;
            ImGui::BeginChild("DataContentView", ImVec2(-1.0f, child_height), true, ImGuiWindowFlags_HorizontalScrollbar);
            ImGui::TextWrapped("Task ID: %d", selected_desc.task_id);
            ImGui::EndChild();
        } else {
            ImGui::Text("Live Data Content:");
            // 고정된 높이(예: 8줄)를 가진 스크롤 가능한 자식 창 생성
            float child_height = ImGui::GetTextLineHeightWithSpacing() * 8;
            ImGui::BeginChild("DataContentView", ImVec2(-1.0f, child_height), true, ImGuiWindowFlags_HorizontalScrollbar);
            ImGui::TextWrapped("%s", display_data_string.c_str());
            ImGui::EndChild();
        }
    }
    ImGui::End();
}


void render_connection_window() {
    // 화면 중앙에 작은 창 표시
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Connecting...", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("Could not connect to RTFW. Is it running?");
    if (ImGui::Button("Retry Connection")) {
        rtfw::monitor::req_refresh = true;
    }
    ImGui::End();
}


int main(int, char**) {
    // GLFW 초기화
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);

    GLFWwindow* window;

    // --- << 모니터 정보 직접 조회 >> ---    
    // 1. 주 모니터에 대한 핸들을 가져옵니다.
    GLFWmonitor* primary_monitor = glfwGetPrimaryMonitor();
    if (primary_monitor == nullptr) {
        // 모니터를 찾을 수 없는 경우, 안전한 기본값으로 fallback
        std::cerr << "Warning: Could not get primary monitor. Using default size." << std::endl;
        window = glfwCreateWindow(1280, 720, "RTFW Monitor", nullptr, nullptr);
    } else {
        // 2. 모니터의 현재 비디오 모드 정보를 가져옵니다.
        const GLFWvidmode* mode = glfwGetVideoMode(primary_monitor);
        if (mode == nullptr) {
            // 비디오 모드를 얻을 수 없는 경우, fallback
            std::cerr << "Warning: Could not get video mode. Using default size." << std::endl;
            window = glfwCreateWindow(1280, 720, "RTFW Monitor", nullptr, nullptr);
        } else {
            // 윈도우 생성
            window = glfwCreateWindow(mode->width, mode->height, "RTFW Monitor", nullptr, nullptr);
        }
    }

    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // ImGui 컨텍스트 초기화
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

        // 기본 폰트를 비우고, 우리가 원하는 폰트를 로드합니다.
    io.Fonts->Clear(); 
    
    // [핵심] TTF 파일로부터 폰트를 로드합니다.
    // 16.0f는 픽셀 단위의 기본 크기입니다.
    ImFont* font = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 18.0f);
    if (font == nullptr) {
        std::cerr << "Error: Failed to load font 'Roboto-Regular.ttf'.\n"
                  << "Please ensure the font file is in the correct path." << std::endl;
        // 폰트 로드 실패 시, ImGui는 자동으로 ProggyClean을 다시 사용합니다.
    }
    
    // (선택적이지만 권장) Bold 폰트도 함께 로드
    // io.Fonts->AddFontFromFileTTF("assets/fonts/Roboto-Bold.ttf", 18.0f);

    // [선택적] 폰트 품질 향상을 위한 오버샘플링 (이전 답변 참조)
    // ImFontConfig font_config;
    // font_config.OversampleH = 2;
    // font_config.OversampleV = 2;
    // io.Fonts->AddFontFromFileTTF("...", 18.0f, &font_config);

    // 폰트 아틀라스를 다시 빌드하도록 합니다.
    // 대부분의 ImGui 백엔드는 첫 렌더링 시 자동으로 이 작업을 수행합니다.
    // ImGui::SFML::UpdateFontTexture(); 또는 io.Fonts->Build();

    ImGuiStyle& style = ImGui::GetStyle();
    // --- 스크롤바 크기 조절 ---
    // 기본값은 보통 14.0f 근처입니다.
    // 터치스크린 환경에 맞게 20.0f 또는 25.0f 정도로 키워줍니다.
    style.ScrollbarSize = 20.0f;
    // (선택사항) 스크롤바의 둥근 모서리 반경도 조절하여 더 부드럽게 만들 수 있습니다.
    style.ScrollbarRounding = 9.0f;
    // (선택사항) 창이나 프레임의 둥근 모서리도 함께 조절하면 전체적인 디자인 통일성이 높아집니다.
    style.WindowRounding = 5.0f;
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f; // 슬라이더 등 잡는 부분의 둥근 정도
    style.GrabMinSize = 40.0f;

    // ImGui 스타일 설정
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight(); 

    // 렌더러 백엔드 초기화
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    SharedMemoryConnector connector;
    std::unique_ptr<SharedMemoryQuerier> querier_ptr = nullptr;
    std::unique_ptr<SharedMemoryController> controller_ptr = nullptr;

    // [추가] 재연결 시도 간격을 제어하기 위한 변수
    const auto reconnect_interval = std::chrono::seconds(2); // 2초에 한 번씩만 재연결 시도
    auto last_connect_attempt = std::chrono::steady_clock::now() - reconnect_interval;

    auto tick = std::chrono::steady_clock::now();
    // 메인 루프
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // ImGui 새 프레임 시작
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ----------------- UI 코드 시작 -----------------

        if (rtfw::monitor::req_refresh) {
            rtfw::monitor::req_refresh = false;
            querier_ptr = nullptr;
            connector.cleanup();
        }

        if (querier_ptr) { // 1. 이미 연결된 경우
            // 상태 플래그를 확인하여 연결이 유효한지 검사
            auto current_state = querier_ptr->getHeader()->shm_state.load(std::memory_order_acquire);
            if (current_state == ShmState::UNINITIALIZED || 
                current_state == ShmState::SHUTTING_DOWN) { // 또는 다른 비정상 상태
                
                // SHUTTING_DOWN 상태일 때는 데이터를 계속 보여주되, 연결을 끊을 준비
                if(current_state == ShmState::UNINITIALIZED){
                    std::cout << "Connection lost. Cleaning up." << std::endl;
                    querier_ptr = nullptr;
                    controller_ptr = nullptr;
                    // connector.cleanup(); // cleanup은 다음에 connect가 새로 될 때 자동으로 되므로 굳이 필요 없음
                }
            }
        }

        if (!querier_ptr) { // 2. 연결되지 않은 경우
            auto now = std::chrono::steady_clock::now();
            if (now - last_connect_attempt > reconnect_interval) {
                last_connect_attempt = now; // 시도 시간 갱신
                
                std::cout << "Attempting to connect to RTFW..." << std::endl;
                void* shm_ptr = connector.connect(SHM_NAME);
                if (shm_ptr) {
                    // 연결 성공 시, Querier 인스턴스 생성
                    querier_ptr = std::make_unique<SharedMemoryQuerier>(shm_ptr);
                    controller_ptr = std::make_unique<SharedMemoryController>(shm_ptr);
                }
            }
        }

        if (querier_ptr) {
            render_main_dashboard_window(*querier_ptr);
            render_analysis_center_window(*querier_ptr, *controller_ptr);
            render_live_log_window(*querier_ptr, *controller_ptr);
            render_dependency_graph_window(*querier_ptr);
        } else {
            render_connection_window();
        }

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