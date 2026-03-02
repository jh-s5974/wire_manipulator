// apps/monitor_tool/graph_renderer.cpp
#include "graph_renderer.h"
#include "ui_state.h"
#include "rtfw_common/shm_layout.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>

#include <string>
#include <vector>
#include <numeric>
#include <map>
#include <set>
#include <queue>
#include <algorithm>
#include <cmath>

using namespace rtfw::common;

// --- 익명 네임스페이스에 헬퍼와 상태 변수 캡슐화 ---
namespace {

const static std::vector<ImU32> TIMELINE_COLOR_PALETTE = {
    IM_COL32(31, 119, 180, 255),  IM_COL32(255, 127, 14, 255),
    IM_COL32(44, 160, 44, 255),   IM_COL32(214, 39, 40, 255),
    IM_COL32(148, 103, 189, 255), IM_COL32(227, 119, 194, 255),
    IM_COL32(188, 189, 34, 255),   IM_COL32(23, 190, 207, 255)
};
const static ImU32 NON_RT_COLOR = IM_COL32(127, 127, 127, 255); // Gray

struct SelectedElement {
    enum Type { NONE, NODE, EDGE, ORPHANED_ARROW, UNRESOLVED_ARROW };
    Type type = NONE;
    uint32_t id1 = -1;
    uint32_t id2 = -1;

    bool operator==(const SelectedElement& other) const {
        return type == other.type && id1 == other.id1 && id2 == other.id2;
    }
    bool operator!=(const SelectedElement& other) const {
        return !(*this == other);
    }
};
static SelectedElement selected_element;

static ImVec2 canvas_offset = ImVec2(0.0f, 0.0f);
static float canvas_scale = 1.0f;

// --- 헬퍼 함수 ---
static float GetLineDistanceSqr(const ImVec2& p1, const ImVec2& p2, const ImVec2& p) {
    ImVec2 diff = p2 - p1;
    float l2 = diff.x * diff.x + diff.y * diff.y;
    if (l2 == 0.0f) {
        ImVec2 p_diff = p - p1;
        return p_diff.x * p_diff.x + p_diff.y * p_diff.y;
    }
    float t = ((p.x - p1.x) * diff.x + (p.y - p1.y) * diff.y) / l2;
    t = std::max(0.0f, std::min(1.0f, t));
    ImVec2 projection = p1 + diff * t;
    ImVec2 proj_diff = p - projection;
    return proj_diff.x * proj_diff.x + proj_diff.y * proj_diff.y;
}

static std::string JoinStrings(const std::vector<const char*>& vec, const std::string& delimiter) {
    if (vec.empty()) return "";
    std::string result;
    for (size_t i = 0; i < vec.size(); ++i) {
        result += vec[i];
        if (i < vec.size() - 1) result += delimiter;
    }
    return result;
}

static ImVec2 ImCubicBezier(const ImVec2& p1, const ImVec2& p2, const ImVec2& p3, const ImVec2& p4, float t) {
    float u = 1.0f - t;
    float w1 = u * u * u;
    float w2 = 3 * u * u * t;
    float w3 = 3 * u * t * t;
    float w4 = t * t * t;
    return p1 * w1 + p2 * w2 + p3 * w3 + p4 * w4;
}

static bool IsMouseHoveringCubicBezier(const ImVec2& p1, const ImVec2& p2, const ImVec2& p3, const ImVec2& p4, float thickness) {
    ImVec2 mouse_pos = ImGui::GetMousePos();
    ImVec2 min_p = ImVec2(std::min({p1.x, p2.x, p3.x, p4.x}) - thickness, std::min({p1.y, p2.y, p3.y, p4.y}) - thickness);
    ImVec2 max_p = ImVec2(std::max({p1.x, p2.x, p3.x, p4.x}) + thickness, std::max({p1.y, p2.y, p3.y, p4.y}) + thickness);
    if (!ImGui::IsMouseHoveringRect(min_p, max_p, false)) return false;

    for (int i = 0; i < 10; ++i) {
        ImVec2 p_start = ImCubicBezier(p1, p2, p3, p4, i / 10.0f);
        ImVec2 p_end = ImCubicBezier(p1, p2, p3, p4, (i + 1) / 10.0f);
        if (GetLineDistanceSqr(p_start, p_end, mouse_pos) < thickness * thickness) return true;
    }
    return false;
}

static void RenderTextCentered(ImDrawList* draw_list, ImVec2 min, ImVec2 max, const char* text, float font_size) {
    // 폰트 사이즈를 고려하여 텍스트 크기 계산
    ImVec2 text_size = ImGui::GetFont()->CalcTextSizeA(font_size, FLT_MAX, 0.0f, text);
    ImVec2 text_pos = min + (max - min - text_size) * 0.5f;
    draw_list->AddText(ImGui::GetFont(), font_size, text_pos, IM_COL32_WHITE, text);
}

static std::map<int, ImU32> get_timeline_color_map(SharedMemoryQuerier& querier) {
    auto nodes_info = querier.getGraphNodes();

    static std::map<int, ImU32> timeline_color_map;
    static size_t last_node_hash = 0;
    size_t current_node_hash = nodes_info.size(); // 간단한 해시

    if (current_node_hash != last_node_hash) {
        last_node_hash = current_node_hash;
        timeline_color_map.clear();
        std::set<int> unique_frequencies;
        for (const auto& node : nodes_info) {
            if (!node.is_non_rt && node.frequency > 0) {
                unique_frequencies.insert(node.frequency);
            }
        }
        std::vector<int> sorted_frequencies(unique_frequencies.begin(), unique_frequencies.end());
        std::sort(sorted_frequencies.begin(), sorted_frequencies.end());
        for (size_t i = 0; i < sorted_frequencies.size(); ++i) {
            timeline_color_map[sorted_frequencies[i]] = TIMELINE_COLOR_PALETTE[i % TIMELINE_COLOR_PALETTE.size()];
        }
    }

    return timeline_color_map;
}

} // end anonymous namespace

void render_timeline_sync_diagram(SharedMemoryQuerier& querier, int* selected_timeline_freq_ptr) {
    ImGui::SeparatorText("Timeline Synchronization Diagram");

    // 1. 데이터 준비: 모든 RT 주파수와 색상 맵 가져오기
    const auto& nodes = querier.getGraphNodes();
    std::set<int> unique_frequencies_set;
    for (const auto& node : nodes) {
        if (!node.is_non_rt && node.frequency > 0) {
            unique_frequencies_set.insert(node.frequency);
        }
    }
    if (unique_frequencies_set.empty()) return;

    // 높은 주파수가 위로 오도록 정렬
    std::vector<int> sorted_frequencies(unique_frequencies_set.rbegin(), unique_frequencies_set.rend());
    
    // 1. 가장 느린 주파수 찾기 (sorted_frequencies는 이미 내림차순 정렬됨)
    const int slowest_freq = sorted_frequencies.back();
    
    // 2. 가장 느린 주파수가 최소 2번의 Tick을 표시하도록 전체 시간(duration)을 결정
    //    (예: 30Hz -> 주기 ~33ms, 2번 Tick이면 ~66ms. 넉넉하게 100ms를 보여주자)
    const float slowest_period_ms = (1.0f / slowest_freq) * 1000.0f;
    const float total_duration_ms = slowest_period_ms * 3; // 느린 주파수의 3주기만큼 표시

    // 3. 가장 빠른 주파수 (base_freq)가 이 시간 동안 몇 번의 Tick을 갖는지 계산
    const int base_freq = sorted_frequencies.front();
    const int num_total_base_ticks = static_cast<int>(total_duration_ms / ((1.0f / base_freq) * 1000.0f));

    // 의존성 그래프에서 사용했던 동적 색상 맵을 그대로 가져와서 
    const auto& color_map = get_timeline_color_map(querier);

    // 2. 그리기 준비
    const float y_padding = 5.0f;
    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    // 1. 스크롤바가 생기지 않도록 필요한 전체 높이를 계산
    const float diagram_height = (line_height + y_padding * 2) * sorted_frequencies.size() + 25.0f;
    
    // 2. 좌측 레이블을 위한 여백 계산
    float max_label_width = 0.0f;
    for (int freq : sorted_frequencies) {
        char freq_buf[32];
        sprintf(freq_buf, "%d Hz", freq);
        float label_width = ImGui::CalcTextSize(freq_buf).x;
        if (label_width > max_label_width) {
            max_label_width = label_width;
        }
    }
    const float left_margin = max_label_width + 10.0f; // 레이블 너비 + 여유 공간
    const float right_margin = 10.0f;

    ImGui::BeginChild("SyncDiagramChild", ImVec2(0, diagram_height), true);
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 p_child = ImGui::GetCursorScreenPos();
    float diagram_width = ImGui::GetContentRegionAvail().x;
    
    // 실제 타임라인이 그려질 영역 계산
    ImVec2 p_timeline_start = ImVec2(p_child.x + left_margin, p_child.y);
    float timeline_width = diagram_width - left_margin - right_margin;

    // ... (각 타임라인에 대한 루프) ...
    for (size_t i = 0; i < sorted_frequencies.size(); ++i) {
        int freq = sorted_frequencies[i];
        float y_center = p_child.y + (i * (line_height + y_padding * 2)) + (line_height / 2.0f) + 2*y_padding;
        
        ImU32 line_color = color_map.count(freq) ? color_map.at(freq) : IM_COL32_WHITE;

        // 타임라인 수평선
        draw_list->AddLine(
            ImVec2(p_timeline_start.x, y_center), 
            ImVec2(p_timeline_start.x + timeline_width, y_center), 
            line_color, 1.5f
        );

        // --- [수정] 주파수 텍스트 레이블 그리기 ---
        // 계산된 왼쪽 여백 안에 그림
        char freq_buf[32];
        sprintf(freq_buf, "%d Hz", freq);
        draw_list->AddText(ImVec2(p_child.x + 5, y_center - ImGui::GetTextLineHeight() / 2.0f), line_color, freq_buf);

        // Tick 마커 '|' 그리기
        int tick_ratio = base_freq / freq;
        if (tick_ratio > 0) {
            
            // 만약 현재 주파수(freq)가 너무 촘촘하다면(예: 1000Hz),
            // 모든 Tick을 다 그리지 않고 건너뛰는 로직 (Optional)
            int tick_draw_step = 1;
            if (timeline_width / (num_total_base_ticks / tick_ratio) < 4.0f) { // 마커 간격이 4px보다 좁으면
                tick_draw_step = static_cast<int>(4.0f / (timeline_width / (num_total_base_ticks / tick_ratio)));
            }

            for (int j = 0; j <= num_total_base_ticks; ++j) {
                if ((j % tick_ratio == 0) && (j / tick_ratio) % tick_draw_step == 0) {
                    float x_pos = p_timeline_start.x + (timeline_width / (float)num_total_base_ticks) * j;
                    draw_list->AddLine(ImVec2(x_pos, y_center - 5.0f), ImVec2(x_pos, y_center + 5.0f), line_color, 2.0f);
                }
            }
        }
    }

    // X축 레이블 추가 (선택적)
    char time_label_buf[32];
    sprintf(time_label_buf, "0 ms");
    draw_list->AddText(ImVec2(p_timeline_start.x, p_timeline_start.y - 5.0f), IM_COL32_WHITE, time_label_buf);
    sprintf(time_label_buf, "%.0f ms", total_duration_ms);
    ImVec2 text_size = ImGui::CalcTextSize(time_label_buf);
    draw_list->AddText(ImVec2(p_timeline_start.x + timeline_width - text_size.x, p_timeline_start.y - 5.0f), IM_COL32_WHITE, time_label_buf);
    
    // 4. 상호작용 (보이지 않는 버튼 + 하이라이트)
    for (size_t i = 0; i < sorted_frequencies.size(); ++i) {
        int freq = sorted_frequencies[i];
        float y_min = p_child.y + i * (line_height + y_padding * 2) + y_padding;
        float y_max = y_min + line_height + 10.0f;
        
        ImGui::SetCursorScreenPos(ImVec2(p_child.x, y_min));
        if (ImGui::InvisibleButton(("##timeline_select_" + std::to_string(freq)).c_str(), ImVec2(diagram_width, line_height+10.0f))) {
            *selected_timeline_freq_ptr = freq;
        }

        if (*selected_timeline_freq_ptr == freq) {
            draw_list->AddRectFilled(ImVec2(p_child.x, y_min), ImVec2(p_child.x + diagram_width, y_max), IM_COL32(50, 120, 255, 50));
        } else if (ImGui::IsItemHovered()) {
            draw_list->AddRectFilled(ImVec2(p_child.x, y_min), ImVec2(p_child.x + diagram_width, y_max), IM_COL32(255, 255, 255, 40));
        }
    }

    ImGui::EndChild();
}

// --- 메인 렌더링 함수 ---
void render_dependency_graph_window(SharedMemoryQuerier& querier) {
    // --- 1. 팝업 열기 ---
    // show_dependency_graph_popup 플래그가 true이면, 매 프레임 팝업을 열려고 시도
    if (rtfw::monitor::show_dependency_graph_popup) {
        ImGui::OpenPopup("Dependency Graph");
        
        // 팝업이 열릴 때, 전달받은 ID로 선택된 요소를 설정
        if (rtfw::monitor::graph_highlight_node_id != static_cast<uint32_t>(-1)) {
            selected_element = {SelectedElement::NODE, rtfw::monitor::graph_highlight_node_id};
            rtfw::monitor::graph_highlight_node_id = static_cast<uint32_t>(-1); // 사용 후 초기화
        }
    }

    // --- 2. 팝업 창 설정 및 렌더링 ---
    // 화면 중앙에 위치하도록 설정
    ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    // 뷰포트의 80% 크기로 설정
    ImGui::SetNextWindowSize(ImGui::GetMainViewport()->Size * 0.9f, ImGuiCond_Appearing);

    ImGuiWindowFlags popup_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoMove; // 메뉴바 추가 가능

    if (ImGui::BeginPopupModal("Dependency Graph", &rtfw::monitor::show_dependency_graph_popup, popup_flags)) {

        // --- 1. 데이터 수집 및 분류 ---
        auto nodes_info = querier.getGraphNodes();
        auto edges = querier.getGraphEdges();
        auto data_flows = querier.getDataFlows();
        if (nodes_info.empty()) { ImGui::Text("No tasks registered..."); ImGui::End(); return; }

        constexpr uint32_t UNCONNECTED_ID = static_cast<uint32_t>(-1);
        std::map<std::pair<uint32_t, uint32_t>, std::vector<const char*>> connected_flows_map;
        // [수정] Task 중심 데이터 구조 재구성
        struct FlowInfo { const char* key; uint32_t peer_id; bool is_weak; };
        std::map<uint32_t, std::vector<FlowInfo>> inputs_map;  // reader_id -> {key, writer_id}
        std::map<uint32_t, std::vector<FlowInfo>> outputs_map; // writer_id -> {key, reader_id}

        for (size_t i = 0; i < data_flows.size(); ++i) {
            const auto& flow = data_flows[i];

            bool is_weak_flag = false;
            if (i < edges.size()) {
                // ID가 일치하는지 추가로 확인하여 데이터 무결성 보장
                if (edges[i].writer_task_id == flow.writer_task_id && edges[i].reader_task_id == flow.reader_task_id) {
                    is_weak_flag = edges[i].is_weak;
                }
            }

            if (flow.reader_task_id != UNCONNECTED_ID) {
                inputs_map[flow.reader_task_id].push_back({flow.key, flow.writer_task_id, is_weak_flag});
            }
            if (flow.writer_task_id != UNCONNECTED_ID) {
                outputs_map[flow.writer_task_id].push_back({flow.key, flow.reader_task_id, is_weak_flag});
            }
            if (flow.writer_task_id != UNCONNECTED_ID && flow.reader_task_id != UNCONNECTED_ID) {
                connected_flows_map[{flow.writer_task_id, flow.reader_task_id}].push_back(flow.key);
            }
        }
        
        std::map<uint32_t, const rtfw::common::TaskGraphNodeInfo*> node_map;
        for(const auto& node : nodes_info) node_map[node.task_id] = &node;
        std::set<std::pair<uint32_t, uint32_t>> weak_edges_set;
        for (const auto& edge : edges) if (edge.is_weak) weak_edges_set.insert({edge.writer_task_id, edge.reader_task_id});

        // --- << 동적 색상 할당 >> ---
        static std::map<int, ImU32> timeline_color_map = get_timeline_color_map(querier);

        // --- 2. 하이라이트 집합 생성 ---
        std::set<uint32_t> highlighted_nodes;
        std::set<std::pair<uint32_t, uint32_t>> highlighted_edges;

        if (selected_element.type == SelectedElement::NODE || selected_element.type == SelectedElement::ORPHANED_ARROW || selected_element.type == SelectedElement::UNRESOLVED_ARROW) {
            uint32_t id = selected_element.id1;
            highlighted_nodes.insert(id);
            for (const auto& edge : edges) {
                if (edge.writer_task_id == id) { highlighted_nodes.insert(edge.reader_task_id); highlighted_edges.insert({id, edge.reader_task_id}); }
                if (edge.reader_task_id == id) { highlighted_nodes.insert(edge.writer_task_id); highlighted_edges.insert({edge.writer_task_id, id}); }
            }
        } else if (selected_element.type == SelectedElement::EDGE) {
            highlighted_nodes.insert(selected_element.id1);
            highlighted_nodes.insert(selected_element.id2);
            highlighted_edges.insert({selected_element.id1, selected_element.id2});
        }

        // --- 레이아웃 계산 (위상 정렬) ---
        std::map<uint32_t, std::vector<uint32_t>> adj;
        std::map<uint32_t, int> in_degree;
        for (const auto& node : nodes_info) { adj[node.task_id] = {}; in_degree[node.task_id] = 0; }
        // for (const auto& edge : edges) { adj[edge.writer_task_id].push_back(edge.reader_task_id); in_degree[edge.reader_task_id]++; }
        for (const auto& edge : edges) {
            if (!edge.is_weak) { // <<< Strong 의존성일 경우에만
                adj[edge.writer_task_id].push_back(edge.reader_task_id);
                in_degree[edge.reader_task_id]++;
            }
        }
        std::queue<uint32_t> q;
        std::map<uint32_t, int> layers;
        int max_layer = 0;
        for (const auto& pair : in_degree) { if (pair.second == 0) { q.push(pair.first); layers[pair.first] = 0; } }
        while (!q.empty()) {
            uint32_t u = q.front(); q.pop();
            for (uint32_t v : adj[u]) {
                if (layers.find(v) == layers.end() && --in_degree[v] <= 0) {
                    q.push(v); layers[v] = layers[u] + 1; max_layer = std::max(max_layer, layers[v]);
                }
            }
        }
        
        // --- [리팩토링] UI 분할 및 캔버스 설정 (줌 슬라이더 추가) ---
        ImGui::BeginChild("LeftPane", ImVec2(ImGui::GetContentRegionAvail().x * 0.75f, 0), false, ImGuiWindowFlags_NoMove);
        
        // 줌 컨트롤을 위한 자식 창
        ImGui::BeginChild("ZoomControls", ImVec2(40, ImGui::GetContentRegionAvail().y), false);
        {
            // 1. 최대 배율 텍스트 (위쪽)
            ImGui::Text("3.0x");
            ImGui::Spacing(); // 약간의 여백

            // 2. 세로 슬라이더
            float old_scale = canvas_scale;
            // 슬라이더 높이를 계산하여 위/아래 텍스트 공간을 확보
            float slider_height = ImGui::GetContentRegionAvail().y - ImGui::GetTextLineHeightWithSpacing() * 2.5f;
            ImGui::VSliderFloat("##zoom", ImVec2(18, slider_height), &canvas_scale, 0.3f, 3.0f, "");
            if (canvas_scale != old_scale) {
                ImVec2 canvas_size = ImGui::GetWindowSize();
                ImVec2 center_of_view = (canvas_size * 0.5f - canvas_offset) / old_scale;
                canvas_offset = canvas_size * 0.5f - center_of_view * canvas_scale;
            }

            // 3. 최소 배율 텍스트 (아래쪽)
            ImGui::Spacing(); // 약간의 여백
            ImGui::Text("0.3x");
        }
        ImGui::EndChild(); // End ZoomControls

        ImGui::SameLine();

        // 캔버스 영역 시작
        ImGui::BeginChild("GraphCanvas", ImVec2(0, 0), true, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImGuiIO& io = ImGui::GetIO();
        
        // --- 노드 좌표 계산 ---
        const ImVec2 origin = ImGui::GetCursorScreenPos() + canvas_offset;
        const float base_node_width = 160.0f, base_node_height = 50.0f;
        const float base_x_spacing = 60.0f, base_y_spacing = 30.0f;
        
        const float node_width = base_node_width;// * canvas_scale;
        const float node_height = base_node_height;// * canvas_scale;
        const float x_spacing = base_x_spacing;// * canvas_scale;
        const float y_spacing = base_y_spacing;// * canvas_scale;

        
        const ImU32 connection_highlight_color = IM_COL32(0, 255, 255, 255); // 밝은 청록색

        std::vector<int> layer_counts(max_layer + 1, 0);
        for(const auto& pair : layers) { if(pair.second < layer_counts.size()) layer_counts[pair.second]++; }
        float max_nodes_in_layer_count = layer_counts.empty() ? 1.0f : *std::max_element(layer_counts.begin(), layer_counts.end());
        float total_canvas_height = max_nodes_in_layer_count * (node_height + y_spacing) + y_spacing;
        std::map<uint32_t, ImVec2> node_world_positions;
        std::vector<int> layer_current_idx(max_layer + 1, 0);
        for (const auto& node : nodes_info) {
            if (layers.find(node.task_id) == layers.end()) continue;
            int layer = layers[node.task_id];
            int idx_in_layer = layer_current_idx[layer]++;
            float local_x = layer * (node_width + x_spacing) + x_spacing;
            float local_y = (layer_counts[layer] > 1) ? ((total_canvas_height - node_height) / (layer_counts[layer] - 1)) * idx_in_layer : (total_canvas_height / 2.0f);
            // node_positions[node.task_id] = origin + ImVec2(local_x, local_y);
            node_world_positions[node.task_id] = ImVec2(local_x, local_y);
        }

        // --- 렌더링 실행 ---
        draw_list->PushClipRect(ImGui::GetCursorScreenPos(), ImGui::GetCursorScreenPos() + ImGui::GetContentRegionAvail(), true);
        SelectedElement hovered_element;

        // 4.1 간선 렌더링
        std::map<uint32_t, int> outgoing_edge_counts, incoming_edge_counts;
        for(const auto& edge : edges) { outgoing_edge_counts[edge.writer_task_id]++; incoming_edge_counts[edge.reader_task_id]++; }
        std::map<uint32_t, int> outgoing_edge_idx, incoming_edge_idx;
        for (const auto& edge : edges) {
            if (node_world_positions.count(edge.writer_task_id) == 0 || node_world_positions.count(edge.reader_task_id) == 0) continue;

            if (edge.is_weak) continue;

            // 1. 월드 좌표를 가져옴
            ImVec2 world_pos1 = node_world_positions[edge.writer_task_id];
            ImVec2 world_pos2 = node_world_positions[edge.reader_task_id];

            // 2. 앵커 위치 계산 (월드 좌표계 기준)
            float out_anchor_y_world = world_pos1.y + base_node_height * (outgoing_edge_idx[edge.writer_task_id]++ + 1.0f) / (outgoing_edge_counts[edge.writer_task_id] + 1.0f);
            float in_anchor_y_world = world_pos2.y + base_node_height * (incoming_edge_idx[edge.reader_task_id]++ + 1.0f) / (incoming_edge_counts[edge.reader_task_id] + 1.0f);
            
            // 3. 최종 스크린 좌표 계산 (줌 & 팬 적용)
            ImVec2 p1 = origin + ImVec2(world_pos1.x + base_node_width, out_anchor_y_world) * canvas_scale;
            ImVec2 p2 = origin + ImVec2(world_pos2.x, in_anchor_y_world) * canvas_scale;
            ImVec2 cp1 = p1 + ImVec2(base_x_spacing * 0.6f * canvas_scale, 0);
            ImVec2 cp2 = p2 - ImVec2(base_x_spacing * 0.6f * canvas_scale, 0);

            if (IsMouseHoveringCubicBezier(p1, cp1, cp2, p2, 8.0f)) {
                hovered_element = {SelectedElement::EDGE, edge.writer_task_id, edge.reader_task_id};
            }

            bool is_highlighted = highlighted_edges.count({edge.writer_task_id, edge.reader_task_id});
            if (IsMouseHoveringCubicBezier(p1, cp1, cp2, p2, 8.0f)) {
                hovered_element = {SelectedElement::EDGE, edge.writer_task_id, edge.reader_task_id};
            }
            bool is_selected = (selected_element.type == SelectedElement::EDGE && selected_element.id1 == edge.writer_task_id && selected_element.id2 == edge.reader_task_id);
            bool is_hovered = (hovered_element.type == SelectedElement::EDGE && hovered_element.id1 == edge.writer_task_id && hovered_element.id2 == edge.reader_task_id);
            
            float thickness = (is_selected || is_hovered || is_highlighted) ? 3.0f : 1.5f;
            ImU32 line_color = IM_COL32(180, 180, 100, 255);
            
            // 선택되거나 하이라이트된 경우, 색상을 덮어씀 (더 높은 우선순위)
            if (is_selected || is_highlighted) {
                line_color = connection_highlight_color;
            }
            
            draw_list->AddBezierCubic(p1, cp1, cp2, p2, line_color, thickness);
        }

        // 4.2 노드 및 연결되지 않은 데이터 화살표 렌더링
        const rtfw::common::TaskStats* stats_array = querier.getTaskStatsArray();
        for (const auto& pair : node_world_positions) {
            uint32_t id = pair.first;
            ImVec2 world_pos = pair.second;
            const auto* info = node_map.at(id);

            // [핵심] 월드 좌표 -> 스크린 좌표 변환
            ImVec2 node_rect_min = origin + world_pos * canvas_scale;
            ImVec2 node_rect_max = node_rect_min + ImVec2(base_node_width, base_node_height) * canvas_scale;

            bool is_highlighted = highlighted_nodes.count(id);
            if (ImGui::IsMouseHoveringRect(node_rect_min, node_rect_max)) {
                hovered_element = {SelectedElement::NODE, id};
            }
            bool is_selected = (selected_element.type == SelectedElement::NODE && selected_element.id1 == id);
            bool is_hovered = (hovered_element.type == SelectedElement::NODE && hovered_element.id1 == id);
            
            ImU32 bg_color;
            if (info->is_non_rt) {
                bg_color = NON_RT_COLOR;
            } else if (timeline_color_map.count(info->frequency)) {
                bg_color = timeline_color_map.at(info->frequency);
            } else {
                bg_color = IM_COL32(60, 60, 120, 255); // Fallback
            }

            // 상태(Busy, Overrun)에 따른 색상 덮어쓰기 (기존 로직 유지)
            if(stats_array && id < querier.getTaskStatsCount()) {
                if (stats_array[id].has_overrun.load()) bg_color = IM_COL32(180, 40, 40, 255);
                else if (stats_array[id].is_busy.load()) bg_color = IM_COL32(40, 150, 40, 255);
            }
            ImU32 border_color = (is_selected || is_highlighted) ? IM_COL32(255, 255, 0, 255) : (is_hovered ? IM_COL32(255, 255, 255, 255) : IM_COL32(150, 150, 180, 255));

            draw_list->AddRectFilled(node_rect_min, node_rect_max, bg_color, 4.0f * canvas_scale);
            draw_list->AddRect(node_rect_min, node_rect_max, border_color, 4.0f * canvas_scale, 0, is_selected || is_hovered ? 2.5f : 1.5f);
            
            if (node_width > 20.0f) {
                float base_font_size = ImGui::GetFontSize();
                float font_size = std::clamp(ImGui::GetFontSize() * canvas_scale, 8.0f, ImGui::GetFontSize() * 1.5f);
                char buf[128];
                sprintf(buf, "%s\n(ID:%u, C:%d)", info->task_name, info->task_id, info->affinity);
                RenderTextCentered(draw_list, node_rect_min, node_rect_max, buf, font_size);
            }
            
            float arrow_y_offset_divisor = 3.0f;
            
            bool has_unused = outputs_map.count(id) && std::any_of(outputs_map.at(id).begin(), outputs_map.at(id).end(),
                                                                [](const auto& fi){ return fi.peer_id == UNCONNECTED_ID; });
            if (has_unused) {
                ImVec2 start_pos = ImVec2(node_rect_max.x, node_rect_min.y + node_height / arrow_y_offset_divisor);
                ImVec2 end_pos = start_pos + ImVec2(30.0f * canvas_scale, 0);
                if (ImGui::IsMouseHoveringRect(start_pos, end_pos + ImVec2(5, 5), false)) {
                    hovered_element = {SelectedElement::ORPHANED_ARROW, id};
                }
                ImU32 color = IM_COL32(255, 80, 80, 255);
                draw_list->AddLine(start_pos, end_pos, color, 1.5f);
                draw_list->AddTriangleFilled(end_pos, end_pos + ImVec2(-6 * canvas_scale, -4 * canvas_scale), end_pos + ImVec2(-6 * canvas_scale, 4 * canvas_scale), color);
                arrow_y_offset_divisor = 1.5f; // 다른 화살표와 겹치지 않게
            }

            bool has_unresolved = inputs_map.count(id) && std::any_of(inputs_map.at(id).begin(), inputs_map.at(id).end(),
                                                                    [](const auto& fi){ return fi.peer_id == UNCONNECTED_ID; });
            if (has_unresolved) {
                ImVec2 start_pos = ImVec2(node_rect_min.x, node_rect_min.y + node_height / arrow_y_offset_divisor);
                ImVec2 end_pos = start_pos - ImVec2(30.0f * canvas_scale, 0);
                if (ImGui::IsMouseHoveringRect(end_pos - ImVec2(5, 5), start_pos, false)) {
                    hovered_element = {SelectedElement::UNRESOLVED_ARROW, id};
                }
                ImU32 color = IM_COL32(255, 165, 0, 255);
                draw_list->AddLine(start_pos, end_pos, color, 1.5f);
                draw_list->AddTriangleFilled(start_pos, start_pos + ImVec2(-6 * canvas_scale, -4 * canvas_scale), start_pos + ImVec2(-6 * canvas_scale, 4 * canvas_scale), color);
            }

            ImU32 weak_link_color = IM_COL32(191, 0, 255, 255); // 보라색
            // 1. Weak 입력이 있는지 확인
            bool has_weak_input = std::any_of(edges.begin(), edges.end(), [&](const auto& edge){
                return edge.reader_task_id == id && edge.is_weak;
            });
            if (has_weak_input) {
                ImVec2 port_pos = ImVec2((node_rect_min.x + node_rect_max.x) / 2, node_rect_min.y - 2);
                auto p1 = port_pos;
                auto p2 = port_pos + ImVec2(-7, -7);
                auto p3 = port_pos + ImVec2(7, -7);
                draw_list->AddTriangleFilled(p1, p2, p3, weak_link_color);
                
                bool should_highlight_marker = false;
                // 이 노드(id)로 들어오는 '하이라이트된 간선' 중에 Weak 연결이 있는지 확인
                for (const auto& highlighted_pair : highlighted_edges) {
                    // highlighted_pair는 {writer, reader}
                    // 이 노드(id)가 reader이고, 이 연결이 weak 연결이라면 하이라이트!
                    if (highlighted_pair.second == id && weak_edges_set.count(highlighted_pair)) {
                        should_highlight_marker = true;
                        break; // 하나라도 찾으면 더 이상 찾을 필요 없음
                    }
                }

                if (should_highlight_marker) {
                    draw_list->AddTriangle(p1, p2, p3, connection_highlight_color, 2.0f);
                }

                // 툴팁 상호작용 추가
                ImGui::SetCursorScreenPos(port_pos + ImVec2(-6, -12));
                ImGui::InvisibleButton(("##weak_in_" + std::to_string(id)).c_str(), ImVec2(12, 12));
                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::Text("Weak Inputs (from other timelines):");
                    for (const auto& edge : edges) {
                        if (edge.reader_task_id == id && edge.is_weak) {
                            for(const auto& key : connected_flows_map.at({edge.writer_task_id, id})) {
                                ImGui::BulletText("%s <- %s", key, node_map.at(edge.writer_task_id)->task_name);
                            }
                        }
                    }
                    ImGui::EndTooltip();
                }
            }

            // 2. Weak 출력이 있는지 확인
            bool has_weak_output = std::any_of(edges.begin(), edges.end(), [&](const auto& edge){
                return edge.writer_task_id == id && edge.is_weak;
            });
            if (has_weak_output) {
                ImVec2 port_pos = ImVec2((node_rect_min.x + node_rect_max.x) / 2, node_rect_max.y + 10);
                auto p1 = port_pos;
                auto p2 = port_pos + ImVec2(-7, -7);
                auto p3 = port_pos + ImVec2(7, -7);
                draw_list->AddTriangleFilled(p1, p2, p3, weak_link_color);
                
                // --- 정밀 하이라이트 조건 ---
                bool should_highlight_marker = false;
                // 이 노드(id)에서 나가는 '하이라이트된 간선' 중에 Weak 연결이 있는지 확인
                for (const auto& highlighted_pair : highlighted_edges) {
                    // highlighted_pair는 {writer, reader}
                    // 이 노드(id)가 writer이고, 이 연결이 weak 연결이라면 하이라이트!
                    if (highlighted_pair.first == id && weak_edges_set.count(highlighted_pair)) {
                        should_highlight_marker = true;
                        break; 
                    }
                }
                
                if (should_highlight_marker) {
                    draw_list->AddTriangle(p1, p2, p3, connection_highlight_color, 2.0f);
                }

                // 툴팁 상호작용 추가
                ImGui::SetCursorScreenPos(port_pos + ImVec2(-6, -12));
                ImGui::InvisibleButton(("##weak_out_" + std::to_string(id)).c_str(), ImVec2(12, 12));
                if (ImGui::IsItemHovered()) {
                    ImGui::BeginTooltip();
                    ImGui::Text("Weak Outputs (to other timelines):");
                    for (const auto& edge : edges) {
                        if (edge.writer_task_id == id && edge.is_weak) {
                            for(const auto& key : connected_flows_map.at({id, edge.reader_task_id})) {
                                ImGui::BulletText("%s -> %s", key, node_map.at(edge.reader_task_id)->task_name);
                            }
                        }
                    }
                    ImGui::EndTooltip();
                }
            }
        }

        // --- 입력 처리 로직 ---
        if (ImGui::IsWindowHovered()) {
            // 1. 패닝: 마우스 왼쪽 버튼으로 드래그할 때. (hovered_element와 무관하게 동작)
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                canvas_offset = canvas_offset + io.MouseDelta;
            }

            // 2. 선택/선택 취소: 마우스를 놓았을 때, 그리고 드래그가 아닐 때(클릭일 때)만 동작.
            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                // 클릭으로 간주할 드래그 거리 임계값 (제곱 값으로 비교하여 sqrt 연산 방지)
                const float drag_threshold_sqr = 5.0f * 5.0f;
                if (io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < drag_threshold_sqr) {
                    // 이것은 '클릭'입니다.
                    
                    // Case 1: 아이템 위에서 클릭이 끝난 경우
                    if (hovered_element.type != SelectedElement::NONE) {
                        // 이미 선택된 아이템을 다시 클릭했다면 -> 선택 취소 (토글)
                        if (hovered_element == selected_element) {
                            selected_element = {};
                        } 
                        // 다른 아이템을 클릭했다면 -> 선택 변경
                        else {
                            selected_element = hovered_element;
                        }
                    }
                }
                // 드래그(패닝) 후 마우스를 놓은 경우에는 아무런 선택/해제 동작도 하지 않습니다.
            }
            
            // 3. 툴팁 표시
            if (hovered_element.type != SelectedElement::NONE) {
                ImGui::BeginTooltip();
                switch(hovered_element.type) {
                    case SelectedElement::NODE:
                    {
                        // hovered_element.id1 이 호버된 노드의 task_id 입니다.
                        // node_map에서 해당 태스크의 정보를 가져옵니다.
                        if (node_map.count(hovered_element.id1)) {
                            const auto* info = node_map.at(hovered_element.id1);
                            
                            // Properties 창의 요약 정보를 툴팁에 표시
                            ImGui::Text("%s", info->task_name);
                            ImGui::Separator();
                            ImGui::Text("ID: %u | Freq: %d Hz", info->task_id, info->frequency);
                            if (info->is_non_rt) {
                                ImGui::Text("Type: Non-RT | Pool: Non-RT Pool");
                            } else {
                                ImGui::Text("Type: RT | Core: %s", (info->affinity < 0 ? "Common" : std::to_string(info->affinity).c_str()));
                            }
                        }
                        break;
                    }
                    case SelectedElement::EDGE:
                        ImGui::Text("Flow:\n%s", JoinStrings(connected_flows_map.at({hovered_element.id1, hovered_element.id2}), "\n").c_str());
                        break;
                    case SelectedElement::ORPHANED_ARROW:
                    {
                        std::vector<const char*> keys;
                        for(const auto& flow : outputs_map.at(hovered_element.id1)) if(flow.peer_id == UNCONNECTED_ID) keys.push_back(flow.key);
                        ImGui::Text("Unused (Orphaned) Keys:\n%s", JoinStrings(keys, "\n").c_str());
                        break;
                    }
                    case SelectedElement::UNRESOLVED_ARROW:
                    {
                        std::vector<const char*> keys;
                        for(const auto& flow : inputs_map.at(hovered_element.id1)) if(flow.peer_id == UNCONNECTED_ID) keys.push_back(flow.key);
                        ImGui::Text("Unresolved Keys:\n%s", JoinStrings(keys, "\n").c_str());
                        break;
                    }
                    default: break;
                }
                ImGui::EndTooltip();
            }
        }


        draw_list->PopClipRect();
        ImGui::EndChild(); // End GraphCanvas
        ImGui::EndChild(); // End LeftPane

        // --- 속성 창 렌더링 ---
        ImGui::SameLine();
        ImGui::BeginChild("PropertiesChild", ImVec2(0, 0));
        ImGui::Text("Properties"); ImGui::Separator();
        const ImVec4 strong_color = ImVec4(1.0f, 1.0f, 0.2f, 1.0f); // 밝은 노란색 (그래프와 유사)
        const ImVec4 weak_color   = ImVec4(0.8f, 0.4f, 1.0f, 1.0f); // 밝은 보라색 (그래프와 유사)

        // --- << 터치스크린 드래그 스크롤 구현 >> ---
        // 1. 현재 자식 창(LogScrollingRegion) 위에 마우스 커서가 있는지 확인
        if (ImGui::IsWindowHovered()) {
            // 2. 사용자가 마우스 왼쪽 버튼을 누른 채로 드래그하고 있는지 확인
            //    (터치스크린에서는 '터치 후 이동'에 해당)
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                // 3. 마우스의 수직 이동량(MouseDelta.y)만큼 스크롤 위치(ScrollY)를 변경
                //    마우스를 아래로 내리면(-y 방향) 스크롤은 위로 올라가야 하므로, 부호를 반대로(-) 해준다.
                ImGui::SetScrollY(ImGui::GetScrollY() - ImGui::GetIO().MouseDelta.y);
            }
        }
        
        if (selected_element.type == SelectedElement::NODE && node_map.count(selected_element.id1)) {
            uint32_t id = selected_element.id1;
            const auto* info = node_map.at(id);
            
            ImGui::Text("Task: %s (ID:%u)", info->task_name, info->task_id);
            ImGui::BulletText("Type: %s", info->is_non_rt ? "Non-RT" : "RT");
            ImGui::BulletText("Core Affinity: %d", info->affinity);
            ImGui::BulletText("Frequency: %d Hz", info->frequency);
            ImGui::Separator();

            if (ImGui::TreeNodeEx("Inputs", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (inputs_map.count(id) && !inputs_map.at(id).empty()) {
                    for(const auto& flow : inputs_map.at(id)) {
                        if (flow.peer_id == UNCONNECTED_ID) { // Unresolved
                            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f), "%s [Unresolved]", flow.key);
                        } else { // Connected
                            ImGui::Text("%s", flow.key); // 1. 데이터 키는 기본 색상으로 그림
                            ImGui::SameLine();
                            const ImVec4& arrow_color = flow.is_weak ? weak_color : strong_color;
                            ImGui::TextColored(arrow_color, "<-"); 
                            ImGui::SameLine();
                            ImGui::TextDisabled("%s (ID:%u)", node_map.at(flow.peer_id)->task_name, flow.peer_id);
                        }
                    }
                } else {
                    ImGui::TextDisabled("None");
                }
                ImGui::TreePop();
            }

            if (ImGui::TreeNodeEx("Outputs", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (outputs_map.count(id) && !outputs_map.at(id).empty()) {
                    for(const auto& flow : outputs_map.at(id)) {
                        if (flow.peer_id == UNCONNECTED_ID) { // Unused (Orphaned)
                            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s [Unused]", flow.key);
                        } else { // Connected                            
                            ImGui::Text("%s", flow.key); // 1. 데이터 키는 기본 색상으로 그림
                            ImGui::SameLine();
                            const ImVec4& arrow_color = flow.is_weak ? weak_color : strong_color;
                            ImGui::TextColored(arrow_color, "->");
                            ImGui::SameLine();
                            ImGui::TextDisabled("%s (ID:%u)", node_map.at(flow.peer_id)->task_name, flow.peer_id);
                        }
                    }
                } else {
                    ImGui::TextDisabled("None");
                }
                ImGui::TreePop();
            }
        } else if (selected_element.type == SelectedElement::EDGE) {
            // 간선 선택 시 속성 표시
            uint32_t writer_id = selected_element.id1;
            uint32_t reader_id = selected_element.id2;
            
            const GraphEdge* found_edge = nullptr;
            for(const auto& e : edges) {
                if(e.writer_task_id == selected_element.id1 && e.reader_task_id == selected_element.id2) {
                    found_edge = &e;
                    break;
                }
            }

            ImGui::Text("Type: Connected Data Flow");
            if(found_edge) {
                ImGui::BulletText("Dependency: %s", found_edge->is_weak ? "Weak" : "Strong"); // << [추가]
            }
            if(node_map.count(writer_id) && node_map.count(reader_id)) {
                ImGui::BulletText("From: %s (ID:%u)", node_map.at(writer_id)->task_name, writer_id);
                ImGui::BulletText("To:   %s (ID:%u)", node_map.at(reader_id)->task_name, reader_id);
                ImGui::Separator();
                ImGui::Text("Data Keys:"); ImGui::Indent();
                for(const auto& key : connected_flows_map.at({writer_id, reader_id})) {
                    ImGui::TextUnformatted(key);
                }
                ImGui::Unindent();
            }
        } else {
            ImGui::Text("Select a Task or an Edge to see details.");
        }
        ImGui::EndChild();
        ImGui::End();
        return;
    }
}