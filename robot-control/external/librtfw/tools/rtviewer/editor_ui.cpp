// EditorUI.cpp (GUI 렌더링 코드)
#include "editor_ui.h"
#include "imgui_memory_editor.h" // ImGui 추가 위젯
// #include "rtfw_common/file_archive.h"
#include "rtfw_common/FileBlackbox.h"
#include <iostream>


using namespace rtfw::common;

extern rtfw::blackbox::FileBlackbox g_blackbox;
extern uint64_t log_frequency;
extern std::map<uint64_t, std::vector<LogEntryView>> data_cache;
extern bool contents_init;

static int g_target_tick = 0;
static std::vector<LogEntryView> g_current_tick_data;
static MemoryEditor g_mem_edit; // Hex 에디터 인스턴스


void render_log_file_editor_window() {

    // --- [1. 창의 위치와 크기 설정] ---
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    // 화면 좌상단에 위치, 너비는 화면의 100%, 높이는 100%
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x, viewport->WorkSize.y));

    // --- [2. 창 이동 및 크기 조절 방지 플래그] ---
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;


    ImGui::Begin("Log File Editor", nullptr, window_flags);

    static char filepath_buffer[512];
    if (g_blackbox.is_open() && strlen(filepath_buffer) == 0) {
        strncpy(filepath_buffer, g_blackbox.get_filepath().c_str(), sizeof(filepath_buffer) - 1);
    }

    ImGui::InputText("File", filepath_buffer, sizeof(filepath_buffer));
    ImGui::SameLine();
    if (ImGui::Button("Load")) {
        // "Load" 버튼을 눌렀을 때만 새로 파일을 염
        bool is_load = false;
        try {
            g_blackbox.start(filepath_buffer, rtfw::blackbox::Mode::REPLAY);
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
                    std::cout << "it.second.data.size()=" << it.second.data.size() << std::endl;
                    std::cout << "views.back().data.size()=" << views.back().data.size() << std::endl;
                }
                data_cache.insert({i, views});                
            }
            contents_init = true;
        } catch (...) {
            g_blackbox.shutdown();
        }
    }

    if (!g_blackbox.is_open()) {
        ImGui::Text("Failed to load file or no file loaded.");
        ImGui::End();
        return;
    }

    ImGui::Separator();

    // 2. 메타데이터 표시
    ImGui::Text("File: %s", g_blackbox.get_filepath().c_str()); // 이 함수는 직접 추가해야 함
    const auto& total_tick_count = g_blackbox.getLastTick()+1;
    ImGui::Text("Frequency: %d", g_blackbox.frequency());
    ImGui::Text("Last Tick: %lu", total_tick_count-1);

    if (ImGui::TreeNode("Key Mappings")) {
        for (const auto& pair : g_blackbox.getKeyMapping()) {
            ImGui::Text("  - %s (Hash: 0x%lX)", pair.second.c_str(), pair.first);
        }
        ImGui::TreePop();
    }
    
    ImGui::Separator();

    // 3. Tick 탐색기
    ImGui::Text("Tick Explorer");
    if (total_tick_count > 1) {
        bool value_changed = false;

        if (contents_init) {
            value_changed = true;
            contents_init = false;
        }

        // Editbox (InputInt)를 생성합니다. 너비를 80픽셀로 고정합니다.
        ImGui::PushItemWidth(80.0f);
        // InputInt의 값이 변경되면 true를 반환합니다.
        if (ImGui::InputInt("##TickInput", &g_target_tick, 1, 100)) {
            value_changed = true;
        }
        ImGui::PopItemWidth();

        // 같은 줄에 다음 컨트롤(슬라이더)을 배치합니다.
        ImGui::SameLine();

        // 슬라이더가 남은 너비를 모두 사용하도록 설정합니다. (-1.0f)
        ImGui::PushItemWidth(-1.0f);
        // 슬라이더의 값이 변경되면 true를 반환합니다. [1, 2, 12]
        if (ImGui::SliderInt("##TickSlider", &g_target_tick, 0, total_tick_count - 1)) {
            value_changed = true;
        }
        ImGui::PopItemWidth();
        
        // InputInt에 의해 값이 슬라이더의 범위를 벗어날 수 있으므로, 값을 범위 내로 제한합니다.
        if (g_target_tick < 0) {
            g_target_tick = 0;
        } else if (g_target_tick >= total_tick_count) {
            g_target_tick = total_tick_count - 1;
        }

        // InputInt 또는 SliderInt 중 하나의 값이라도 변경되었다면, 해당 Tick의 데이터를 로드합니다.
        if (value_changed) {
            g_current_tick_data = data_cache[g_target_tick];
        }
    }

    // 4. 현재 Tick 데이터 표시
    if (!g_current_tick_data.empty()) {
        ImGui::Text("Data for Tick: %d", g_target_tick);
        for (const auto& item : g_current_tick_data) {
            if (ImGui::CollapsingHeader(g_blackbox.getKeyName(item.key_hash).c_str())) {
                // 고유 ID 스택에 현재 아이템의 해시를 푸시합니다.
                ImGui::PushID(item.key_hash);
                
                // Hex 에디터를 위한 자식 창을 생성합니다. (너비 자동, 높이 150 고정)
                // 이 영역이 최소 높이를 보장하며, 내용이 더 크면 스크롤됩니다.
                ImGui::BeginChild("HexEditorChild", ImVec2(0, 150.0f), true);

                // Hex 에디터 위젯을 그립니다.
                g_mem_edit.DrawContents((void*)item.data.data(), item.data.size());

                // 자식 창을 닫습니다.
                ImGui::EndChild();
                
                // ID 스택에서 현재 아이템의 해시를 팝합니다.
                ImGui::PopID();
            }
        }
    } else {
        ImGui::Text("No data found for Tick: %d", g_target_tick);
    }

    ImGui::End();
}