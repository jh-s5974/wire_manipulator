#pragma once
#include <cstdint>
#include <map>
#include <imgui.h>

// UI의 전역 상태를 선언합니다.
// 'extern' 키워드는 "이 변수의 실체는 다른 어딘가에 정의되어 있다"고 컴파일러에게 알려줍니다.

namespace rtfw::monitor {

    extern bool req_refresh;
    extern bool show_dependency_graph_popup;
    extern uint32_t graph_highlight_node_id;
    extern std::map<int, ImU32> timeline_color_map;

    // 나중에 다른 전역 UI 상태(예: 선택된 타임라인)도 여기에 추가할 수 있습니다.
    // extern int selected_timeline_freq;

} // namespace rtfw::monitor

    