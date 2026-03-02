#include "ui_state.h"

namespace rtfw::monitor {

    // 변수들의 실체를 여기서 정의하고 초기화합니다.
    bool req_refresh = false;
    bool show_dependency_graph_popup = false;
    uint32_t graph_highlight_node_id = static_cast<uint32_t>(-1);
    std::map<int, ImU32> timeline_color_map;

} // namespace rtfw::monitor