// apps/monitor_tool/stats_renderer.cpp
#include "stats_renderer.h"
#include "graph_renderer.h"

#include <chrono>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <algorithm>
#include <cmath>
#include <functional>
#include "ui_state.h" // <<< [핵심] 공유 상태 헤더 포함


using namespace rtfw::common;

// --- 이 파일에서만 사용하는 static 변수 및 구조체 ---
namespace {

// 뷰 모드 및 시간 간격 정의
enum class StatsViewMode { RECENT, LIFETIME };
enum class RecentInterval : int { ONE_SEC = 1, TEN_SEC = 10 };
static StatsViewMode current_view_mode = StatsViewMode::RECENT;
static RecentInterval current_interval = RecentInterval::ONE_SEC;

static auto last_stats_update_time = std::chrono::steady_clock::now();


// 선택된 타임라인의 주파수 (간트 차트 표시용)
static int selected_timeline_freq = -1; 
// 풀 부하율 계산을 위한 이전 측정값
static auto last_pool_update_time = std::chrono::steady_clock::now();
static std::map<int, long long> last_pool_busy_ns; // Key: pool_stats_index


// System Stats 관련
struct TimelineRecentStats {
    long long last_tick_count = -1;
    long long last_total_busy_ns = 0;
    long long last_total_squared_busy_ns = 0;
    double avg_busy_us = 0.0;
    double stdev_busy_us = 0.0;
};
static std::map<int, TimelineRecentStats> timeline_recent_stats_map;

struct PoolRecentStats {
        // 델타 계산을 위한 이전 값
        long long last_total_busy_ns = -1; // Self-Priming을 위해 -1로 초기화

        // 'Recent' 구간 동안의 통계 누적을 위한 값
        double total_usage_sum = 0.0;
        double total_usage_squared_sum = 0.0;
        int update_count = 0;
        double max_usage_percentage_in_recent = 0.0;

        // [최종 결과] 렌더링 함수가 사용할 계산 완료된 값
        double avg_usage_percentage = 0.0;
        double stdev_usage_percentage = 0.0;
};
static std::map<int, PoolRecentStats> pool_recent_stats_map; // Key: pool_stats_index

// Task List 관련
struct TaskRecentStats {
    long long last_exec_count = -1;
    long long last_total_exec_ns = 0;
    long long last_total_squared_exec_ns = 0;
    long long last_total_latency_ns = 0;
    long long last_total_squared_latency_ns = 0;

    double avg_exec_us = 0.0;
    double stdev_exec_us = 0.0;
    double avg_lat_us = 0.0;
    double stdev_lat_us = 0.0;
};
static std::map<uint32_t, TaskRecentStats> recent_stats_map;

// 통계 계산 헬퍼 함수
static long long get_avg(const std::atomic<long long>& total, const std::atomic<long long>& count) {
    long long c = count.load(std::memory_order_relaxed);
    return (c > 0) ? total.load(std::memory_order_relaxed) / c : 0;
};

static long long get_std_dev(const std::atomic<long long>& total_sq, const std::atomic<long long>& total, const std::atomic<long long>& count) {
    long long c = count.load(std::memory_order_relaxed);
    if (c < 2) return 0;
    long long total_ns = total.load(std::memory_order_relaxed);
    long long total_sq_ns = total_sq.load(std::memory_order_relaxed);
    double mean = static_cast<double>(total_ns) / c;
    double variance = static_cast<double>(total_sq_ns) / c - (mean * mean);
    return (variance > 0) ? static_cast<long long>(std::sqrt(variance)) : 0;
};

static void update_recent_timeline_stats(SharedMemoryQuerier& querier, long long elapsed_ns) {
    const auto timeline_stats_array = querier.getTimelineStatsArray();
    if (!timeline_stats_array) return;
    const size_t timeline_count = querier.getTimelineStatsCount();

    for (size_t i = 0; i < timeline_count; ++i) {
        const auto& current_shm_stats = timeline_stats_array[i];
        const int freq = current_shm_stats.frequency;
        auto& recent_stats = timeline_recent_stats_map[freq];

        long long current_tick_count = current_shm_stats.tick_count.load(std::memory_order_relaxed);

        // --- Self-Priming 로직 ---
        if (recent_stats.last_tick_count == -1) {
            // 첫 업데이트: 통계를 0으로 설정하고 베이스라인 저장
            recent_stats.avg_busy_us = 0.0;
            recent_stats.stdev_busy_us = 0.0;
        } else {
            // 두 번째 이후 업데이트: 정상적으로 델타 계산
            long long delta_count = current_tick_count - recent_stats.last_tick_count;
            if (delta_count > 0) {
                long long delta_busy_ns = current_shm_stats.total_busy_ns.load() - recent_stats.last_total_busy_ns;
                long long delta_sq_busy = current_shm_stats.total_squared_busy_ns.load() - recent_stats.last_total_squared_busy_ns;

                recent_stats.avg_busy_us = (double)delta_busy_ns / delta_count / 1000.0;
                
                double mean = (double)delta_busy_ns / delta_count;
                double mean_of_squares = (double)delta_sq_busy / delta_count;
                double variance_ns2 = mean_of_squares - (mean * mean);
                if (variance_ns2 < 0) variance_ns2 = 0;
                recent_stats.stdev_busy_us = std::sqrt(variance_ns2) / 1000.0;
            } else {
                recent_stats.avg_busy_us = 0.0;
                recent_stats.stdev_busy_us = 0.0;
            }
        }
        
        // 다음 프레임을 위해 'last' 값을 현재 값으로 갱신
        recent_stats.last_tick_count = current_tick_count;
        recent_stats.last_total_busy_ns = current_shm_stats.total_busy_ns.load();
        recent_stats.last_total_squared_busy_ns = current_shm_stats.total_squared_busy_ns.load();
    }
};

static void update_recent_pool_stats(SharedMemoryQuerier& querier, long long elapsed_ns) {
    const auto pool_stats_array = querier.getPoolStatsArray();
    if (!pool_stats_array) return;
    const size_t pool_count = querier.getPoolStatsCount();

    if (!pool_stats_array || pool_count == 0) {
        return;
    }

    for (size_t i = 0; i < pool_count; ++i) {
        auto& recent_stats = pool_recent_stats_map[i];
        const auto& shm_stats = pool_stats_array[i];

        long long current_total_busy = shm_stats.total_busy_ns.load();
        long long current_sq_busy = shm_stats.total_squared_busy_ns.load();
        long long current_samples = shm_stats.sample_count.load();

        if (recent_stats.last_total_busy_ns == -1) {
            // 첫 업데이트 (Self-Priming)
            recent_stats.avg_usage_percentage = 0.0;
            recent_stats.stdev_usage_percentage = 0.0;
            recent_stats.max_usage_percentage_in_recent = 0.0;
        } else {
            // --- Usage (%) 계산 ---
            long long delta_busy_ns = current_total_busy - recent_stats.last_total_busy_ns;
            int workers = pool_stats_array[i].num_workers;
            double current_usage = (elapsed_ns > 0) ? (double)delta_busy_ns / elapsed_ns * 100.0 : 0.0;
            current_usage = std::min(100.0, std::max(0.0, current_usage / workers)); // 워커 수로 정규화

            // --- 'Recent' 구간 동안의 통계 누적 ---
            recent_stats.total_usage_sum += current_usage;
            recent_stats.total_usage_squared_sum += (current_usage * current_usage);
            recent_stats.update_count++;
            recent_stats.max_usage_percentage_in_recent = std::max(recent_stats.max_usage_percentage_in_recent, current_usage);

            // --- 최종 평균 및 표준편차 계산 ---
            recent_stats.avg_usage_percentage = recent_stats.total_usage_sum / recent_stats.update_count;
            if (recent_stats.update_count > 1) {
                double mean_sq = recent_stats.total_usage_squared_sum / recent_stats.update_count;
                double avg_sq = recent_stats.avg_usage_percentage * recent_stats.avg_usage_percentage;
                double variance = mean_sq - avg_sq;
                recent_stats.stdev_usage_percentage = (variance > 0) ? std::sqrt(variance) : 0.0;
            } else {
                recent_stats.stdev_usage_percentage = 0.0;
            }
        }
        recent_stats.last_total_busy_ns = current_total_busy;
    }
}

static void update_recent_task_stats(SharedMemoryQuerier& querier, long long elapsed_ns) {
    auto nodes = querier.getGraphNodes();
    const auto stats_array = querier.getTaskStatsArray();
    if (nodes.empty() || !stats_array) return;

    for (const auto& node : nodes) {
        const auto& current_shm_stats = stats_array[node.task_id];
        auto& recent_stats = recent_stats_map[node.task_id];

        long long current_count = current_shm_stats.exec_count.load(std::memory_order_relaxed);

        // --- Self-Priming 로직 ---
        if (recent_stats.last_exec_count == -1) {
            // 첫 업데이트: 모든 통계를 0으로 설정
            recent_stats.avg_exec_us = 0.0; recent_stats.stdev_exec_us = 0.0;
            recent_stats.avg_lat_us = 0.0; recent_stats.stdev_lat_us = 0.0;
        } else {
            // 두 번째 이후 업데이트: 정상적으로 델타 계산
            long long delta_count = current_count - recent_stats.last_exec_count;
            if (delta_count > 0) {
                // Exec Time 통계 계산
                long long delta_total_exec = current_shm_stats.total_exec_time_ns.load() - recent_stats.last_total_exec_ns;
                long long delta_total_sq_exec = current_shm_stats.total_squared_exec_time_ns.load() - recent_stats.last_total_squared_exec_ns;
                
                recent_stats.avg_exec_us = (double)delta_total_exec / delta_count / 1000.0;
                double mean_exec = (double)delta_total_exec / delta_count;
                double variance_exec = (double)delta_total_sq_exec / delta_count - (mean_exec * mean_exec);
                recent_stats.stdev_exec_us = (variance_exec > 0) ? std::sqrt(variance_exec) / 1000.0 : 0.0;

                // Latency 통계 계산
                long long delta_total_lat = current_shm_stats.total_latency_ns.load() - recent_stats.last_total_latency_ns;
                long long delta_total_sq_lat = current_shm_stats.total_squared_latency_ns.load() - recent_stats.last_total_squared_latency_ns;

                recent_stats.avg_lat_us = (double)delta_total_lat / delta_count / 1000.0;
                double mean_lat = (double)delta_total_lat / delta_count;
                double variance_lat = (double)delta_total_sq_lat / delta_count - (mean_lat * mean_lat);
                recent_stats.stdev_lat_us = (variance_lat > 0) ? std::sqrt(variance_lat) / 1000.0 : 0.0;
            } else {
                recent_stats.avg_exec_us = 0.0; recent_stats.stdev_exec_us = 0.0;
                recent_stats.avg_lat_us = 0.0; recent_stats.stdev_lat_us = 0.0;
            }
        }

        // 다음 프레임을 위해 'last' 값을 현재 값으로 갱신
        recent_stats.last_exec_count = current_count;
        recent_stats.last_total_exec_ns = current_shm_stats.total_exec_time_ns.load();
        recent_stats.last_total_squared_exec_ns = current_shm_stats.total_squared_exec_time_ns.load();
        recent_stats.last_total_latency_ns = current_shm_stats.total_latency_ns.load();
        recent_stats.last_total_squared_latency_ns = current_shm_stats.total_squared_latency_ns.load();
    }
}


// [수정] 모든 'Recent' 통계를 한 번에 업데이트하는 최상위 함수
void update_all_recent_stats(SharedMemoryQuerier& querier) {
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_update_time).count() < static_cast<int>(current_interval)) {
        return;
    }

    // 경과 시간 계산
    long long elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_stats_update_time).count();
    if (elapsed_ns <= 0) elapsed_ns = 1;

    // 각 업데이트 함수는 이제 스스로 첫 실행을 판단할 수 있음
    update_recent_timeline_stats(querier, elapsed_ns);
    update_recent_pool_stats(querier, elapsed_ns);
    update_recent_task_stats(querier, elapsed_ns);

    // 시간 기준점 갱신
    last_stats_update_time = now;
}



static void render_timeline_progressbar(
    const TimelineStats& shm_stats, 
    const TimelineRecentStats& recent_stats, // 'Recent' 계산 결과
    int* selected_timeline_freq_ptr)
{
    const int freq = shm_stats.frequency;
    if (freq <= 0) return;

    ImGui::PushID(freq);
    ImGui::BeginGroup();

    // --- 1. 인디케이터 공간 확보 ---
    const float indicator_width = 5.0f;
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    ImGui::Indent(indicator_width + spacing);

    // --- 2. 데이터 계산 (Peak 추가) ---
    const float period_us = 1000000.0f / freq;
    const float avg_ratio = (period_us > 0) ? (float)(recent_stats.avg_busy_us / period_us) : 0.0f;
    const float avg_plus_stdev_us = (float)(recent_stats.avg_busy_us + recent_stats.stdev_busy_us);
    const float stdev_ratio = (period_us > 0) ? avg_plus_stdev_us / period_us : 0.0f;
    const double peak_busy_us = shm_stats.max_busy_ns.load(std::memory_order_relaxed) / 1000.0;
    const float peak_ratio = (period_us > 0) ? (float)(peak_busy_us / period_us) : 0.0f;

    // 클리핑
    float avg_r = std::min(1.0f, std::max(0.0f, avg_ratio));
    float stdev_r = std::min(1.0f, std::max(0.0f, stdev_ratio));
    float peak_r = std::min(1.0f, std::max(0.0f, peak_ratio));

    // --- 3. ProgressBar 렌더링 (텍스트와 색상은 나중에 덮어쓸 것) ---
    ImGui::ProgressBar(avg_r, ImVec2(-1.0f, 0.0f), ""); // 텍스트를 비워서 공간만 차지

    // --- << [핵심 추가] DrawList를 이용한 시각적 요소 덧그리기 >> ---
    ImVec2 p_min = ImGui::GetItemRectMin();
    ImVec2 p_max = ImGui::GetItemRectMax();
    float bar_width = p_max.x - p_min.x;
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // 신호등 색상 결정
    ImVec4 avg_color_v4 = ImVec4(0.2f, 0.7f, 0.2f, 1.0f); // Green
    if (avg_ratio > 0.9f) avg_color_v4 = ImVec4(1.0f, 0.3f, 0.3f, 1.0f); // Red
    else if (avg_ratio > 0.7f) avg_color_v4 = ImVec4(0.9f, 0.7f, 0.0f, 1.0f); // Yellow
    ImVec4 stdev_color_v4 = avg_color_v4;
    stdev_color_v4.w = 0.4f; // 더 투명하게
    
    // Layer 1: ProgressBar 배경 위에 표준편차 음영 그리기
    draw_list->AddRectFilled(p_min, ImVec2(p_min.x + bar_width * stdev_r, p_max.y), ImGui::ColorConvertFloat4ToU32(stdev_color_v4));
    // Layer 2: 평균 바 겹쳐 그리기 (원래 ProgressBar 색상 대신 이걸 사용)
    draw_list->AddRectFilled(p_min, ImVec2(p_min.x + bar_width * avg_r, p_max.y), ImGui::ColorConvertFloat4ToU32(avg_color_v4));
    // Layer 3: Peak 마커 그리기
    float peak_x = p_min.x + bar_width * peak_r;
    draw_list->AddLine(ImVec2(peak_x, p_min.y), ImVec2(peak_x, p_max.y), IM_COL32(255, 100, 100, 200), 2.0f);
    // Layer 4: 텍스트 오버레이
    char buf[128];
    sprintf(buf, "%d Hz | Avg: %.1f%% (±%.1f%%) | Peak: %.1f%% | Miss: %lld",
        freq, avg_ratio * 100.0f, (float)(recent_stats.stdev_busy_us / period_us) * 100.0f,
        peak_ratio * 100.0f, shm_stats.deadline_miss_count.load());
    ImVec2 text_size = ImGui::CalcTextSize(buf);
    ImVec2 text_pos = ImVec2(p_min.x + 5, p_min.y + (p_max.y - p_min.y - text_size.y) * 0.5f);
    draw_list->AddText(text_pos, IM_COL32_WHITE, buf);
    // --- << 추가 종료 >> ---

    ImGui::Unindent(indicator_width + spacing);
    ImGui::EndGroup();

    // --- 3. 상호작용 및 인디케이터 그리기 ---
    bool is_selected = (freq == *selected_timeline_freq_ptr);
    bool is_hovered = ImGui::IsItemHovered(); // EndGroup() 이후 호출하여 그룹 전체에 대한 호버 감지
    
    if (ImGui::IsItemClicked()) {
        *selected_timeline_freq_ptr = freq;
    }

    if (is_selected || is_hovered) {
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 p_min = ImGui::GetItemRectMin();
        ImVec2 p_max = ImGui::GetItemRectMax();
        
        // 인디케이터 색상 결정
        ImU32 indicator_color;
        if (is_selected) {
            // 선택됨: 선명한 테마 색상 (예: 밝은 파랑)
            indicator_color = IM_COL32(50, 120, 255, 255);
        } else { // is_hovered
            // 호버: 은은한 회색
            indicator_color = IM_COL32(255, 255, 255, 80);
        }

        // 들여쓰기로 만든 공간에 인디케이터 사각형 그리기
        draw_list->AddRectFilled(
            ImVec2(p_min.x, p_min.y), 
            ImVec2(p_min.x + indicator_width, p_max.y), 
            indicator_color,
            2.0f // 모서리 둥글게
        );
    }
    
    ImGui::PopID();
};

static void render_pool_progressbar(
    const std::string& pool_name,
    int workers,
    const PoolRecentStats& recent_stats)
{
     // --- 1. 데이터 가져오기 (계산 없음!) ---
    float avg_ratio = (float)recent_stats.avg_usage_percentage / 100.0f;
    float max_ratio = (float)recent_stats.max_usage_percentage_in_recent / 100.0f;
    float stdev_upper_bound_percent = (float)(recent_stats.avg_usage_percentage + recent_stats.stdev_usage_percentage);
    float stdev_ratio = stdev_upper_bound_percent / 100.0f;

    // 클리핑
    avg_ratio = std::min(1.0f, std::max(0.0f, avg_ratio));
    max_ratio = std::min(1.0f, std::max(0.0f, max_ratio));
    stdev_ratio = std::min(1.0f, std::max(0.0f, stdev_ratio));


    // --- 2. 색상 결정 (Timeline과 동일) ---
    ImVec4 avg_color = ImVec4(0.2f, 0.7f, 0.2f, 1.0f); // Green
    if (avg_ratio > 0.9f) avg_color = ImVec4(1.0f, 0.3f, 0.3f, 1.0f); // Red
    else if (avg_ratio > 0.7f) avg_color = ImVec4(0.9f, 0.7f, 0.0f, 1.0f); // Yellow
    ImVec4 std_dev_color = avg_color;
    std_dev_color.w = 0.4f; // 더 투명하게

    // --- 3. ImGui 렌더링 (Timeline과 동일한 기법 사용) ---
    // ProgressBar 배경 그리기 (높이 확보용)

    const float indicator_width = 5.0f;
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    ImGui::Indent(indicator_width + spacing); // 인디케이터 너비 + 간격만큼 들여쓰기
    ImGui::ProgressBar(0.0f, ImVec2(-1.0f, 0.0f), "");

    ImVec2 p_min = ImGui::GetItemRectMin();
    ImVec2 p_max = ImGui::GetItemRectMax();
    float bar_width = p_max.x - p_min.x;
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // 1. (평균 + 표준편차) 음영 영역 그리기
    draw_list->AddRectFilled(p_min, ImVec2(p_min.x + bar_width * stdev_ratio, p_max.y), ImGui::ColorConvertFloat4ToU32(std_dev_color));

    // 2. 평균 영역 겹쳐 그리기
    draw_list->AddRectFilled(p_min, ImVec2(p_min.x + bar_width * avg_ratio, p_max.y), ImGui::ColorConvertFloat4ToU32(avg_color));
    
    // 3. Max 마커(수직선) 그리기
    float max_x = p_min.x + bar_width * max_ratio;
    draw_list->AddLine(ImVec2(max_x, p_min.y), ImVec2(max_x, p_max.y), IM_COL32(255, 100, 100, 200), 2.0f);

    // 4. 텍스트 오버레이
    char buf[256];
    sprintf(buf, "%s (%dw) | Avg: %.1f%% (±%.1f%%) | Max: %.1f%%", 
        pool_name.c_str(), 
        workers,
        recent_stats.avg_usage_percentage,
        recent_stats.stdev_usage_percentage,
        recent_stats.max_usage_percentage_in_recent);
    
    ImVec2 text_size = ImGui::CalcTextSize(buf);
    ImVec2 text_pos = ImVec2(p_min.x + 5, p_min.y + (p_max.y - p_min.y - text_size.y) * 0.5f);
    draw_list->AddText(text_pos, IM_COL32_WHITE, buf);
    
    ImGui::Unindent(indicator_width + spacing); // 인디케이터 너비 + 간격만큼 들여쓰기
};

static void render_task_stats_tab(SharedMemoryQuerier& querier, SharedMemoryController& controller, std::function<bool(const TaskGraphNodeInfo&)> filter=[](const TaskGraphNodeInfo&){return true;}) {
    auto nodes = querier.getGraphNodes();
    const TaskStats* stats_array = querier.getTaskStatsArray();
    if (nodes.empty() || !stats_array) {
        ImGui::Text("No task data available.");
        return;
    }

    if (recent_stats_map.empty())
        return;

    if (ImGui::BeginTable("task_stats_table", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("ID");
        ImGui::TableSetupColumn("Name (Core)");
        ImGui::TableSetupColumn("Exec Time (us)\navg | max | stdev"); // jitter -> stdev
        ImGui::TableSetupColumn("Sched Latency (us)\navg | max | stdev"); // jitter -> stdev
        ImGui::TableSetupColumn("Count");
        ImGui::TableHeadersRow();


        for (const auto& node : nodes) {
            if (!filter(node)) 
                continue;

            ImGui::TableNextRow();
            const TaskStats& shm_stats = stats_array[node.task_id];
            
            // ID, Name 컬럼 (기존과 동일)
            ImGui::TableSetColumnIndex(0); ImGui::Text("%u", node.task_id);
            ImGui::TableSetColumnIndex(1);
            std::string name_core = std::string(node.task_name) + (node.is_non_rt? " (NRT)" : (" (C" + (node.affinity >= 0 ? std::to_string(node.affinity) : "-") + ")"));
            if (shm_stats.has_overrun.load(std::memory_order_relaxed)) {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.2f, 0.2f, 1.0f));
                ImGui::Text("%s [OVERRUN]", name_core.c_str());
                ImGui::PopStyleColor();

                ImGui::SameLine();
                // 각 버튼에 고유한 ID를 부여하기 위해 Task ID를 사용
                ImGui::PushID(node.task_id); 
                if (ImGui::SmallButton("Clear")) {
                    controller.clear_task_overrun_flag(node.task_id);
                }
                ImGui::PopID();
            } else {
                ImGui::Text("%s", name_core.c_str());
            }

            // --- << [핵심 추가] "Show on Graph" 버튼 >> ---
            ImGui::SameLine();
            ImGui::PushID(node.task_id); // 버튼 ID 중복 방지
            // 아이콘 폰트(FontAwesome 등)가 있다면 아이콘을, 없다면 텍스트 버튼 사용
            if (ImGui::SmallButton("Graph")) { 
                rtfw::monitor::show_dependency_graph_popup = true; // 팝업을 열도록 플래그 설정
                rtfw::monitor::graph_highlight_node_id = node.task_id; // 이 노드를 하이라이트하도록 ID 설정
            }
            ImGui::PopID();

            // Exec Time 컬럼
            ImGui::TableSetColumnIndex(2);
            if (current_view_mode == StatsViewMode::RECENT) {
                const auto& recent = recent_stats_map.at(node.task_id);
                ImGui::Text("%.3f |   -   | %.3f", recent.avg_exec_us, recent.stdev_exec_us);
            } else { // LIFETIME
                ImGui::Text("%.3f | %.3f | %.3f", 
                    get_avg(shm_stats.total_exec_time_ns, shm_stats.exec_count) / 1000.0,
                    shm_stats.max_exec_time_ns.load() / 1000.0,
                    get_std_dev(shm_stats.total_squared_exec_time_ns, shm_stats.total_exec_time_ns, shm_stats.exec_count) / 1000.0);
            }

            // Sched Latency 컬럼
            ImGui::TableSetColumnIndex(3);
            if (current_view_mode == StatsViewMode::RECENT) {
                const auto& recent = recent_stats_map.at(node.task_id);
                ImGui::Text("%.3f |   -   | %.3f", recent.avg_lat_us, recent.stdev_lat_us);
            } else { // LIFETIME
                ImGui::Text("%.3f | %.3f | %.3f", 
                    get_avg(shm_stats.total_latency_ns, shm_stats.exec_count) / 1000.0,
                    shm_stats.max_latency_ns.load() / 1000.0,
                    get_std_dev(shm_stats.total_squared_latency_ns, shm_stats.total_latency_ns, shm_stats.exec_count) / 1000.0);
            }
            
            // Count 컬럼
            ImGui::TableSetColumnIndex(4);
            ImGui::Text("%lld", shm_stats.exec_count.load());
        }
        ImGui::EndTable();
    }
};

// --- 간트 차트 렌더링 헬퍼 함수 ---
static void render_gantt_chart_for_timeline(SharedMemoryQuerier& querier, int frequency, StatsViewMode view_mode) {
    if (frequency <= 0) {
        ImGui::TextWrapped("No timeline selected. Click a timeline in the Main Dashboard to see its details.");
        return;
    }

    const float period_us = 1000000.0f / frequency;
    ImGui::Text("Last Tick Gantt Chart (Period: %.2f us)", period_us);
    ImGui::Separator();

    // 간트 차트 그리기 영역 설정
    ImVec2 chart_size = ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetTextLineHeightWithSpacing() * 15); // 최대 15개 태스크 높이
    ImGui::BeginChild("GanttChartChild", chart_size, true, ImGuiWindowFlags_HorizontalScrollbar);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 p = ImGui::GetCursorScreenPos();
    float chart_width = std::max(ImGui::GetContentRegionAvail().x, 400.0f)*0.95; // 최소 너비 확보

    // 배경 및 시간 축
    draw_list->AddRectFilled(p, ImVec2(p.x + chart_width, p.y + chart_size.y), IM_COL32(20, 20, 20, 255));
    // ... (시간 눈금 그리기 로직: 0%, 25%, 50%, 75%, 100% 위치에 수직선과 텍스트) ...


    // --- 1. 배경 및 시간 축 그리기 ---
    draw_list->AddRectFilled(p, ImVec2(p.x + chart_width, p.y + chart_size.y), IM_COL32(20, 20, 20, 255));
    // ... (50% 지점 등에 회색 눈금선 그리기) ...
    float mid_x = p.x + chart_width * 0.00f;
    draw_list->AddLine(ImVec2(mid_x, p.y), ImVec2(mid_x, p.y + chart_size.y), IM_COL32(255, 255, 255, 40));
    mid_x = p.x + chart_width * 0.25f;
    draw_list->AddLine(ImVec2(mid_x, p.y), ImVec2(mid_x, p.y + chart_size.y), IM_COL32(255, 255, 255, 40));
    draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize(), ImVec2(mid_x, p.y + 5), IM_COL32(255,255,255,100), "25%");
    mid_x = p.x + chart_width * 0.5f;
    draw_list->AddLine(ImVec2(mid_x, p.y), ImVec2(mid_x, p.y + chart_size.y), IM_COL32(255, 255, 255, 40));
    draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize(), ImVec2(mid_x, p.y + 5), IM_COL32(255,255,255,100), "50%");
    mid_x = p.x + chart_width * 0.75f;
    draw_list->AddLine(ImVec2(mid_x, p.y), ImVec2(mid_x, p.y + chart_size.y), IM_COL32(255, 255, 255, 40));
    draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize(), ImVec2(mid_x, p.y + 5), IM_COL32(255,255,255,100), "75%");

    // --- 2. [핵심 추가] 데드라인(100%) 위치에 수직선 그리기 ---
    float deadline_x = p.x + chart_width; // 오른쪽 끝
    
    draw_list->AddLine(
        ImVec2(deadline_x, p.y),                    // 시작점 (상단)
        ImVec2(deadline_x, p.y + chart_size.y),     // 끝점 (하단)
        IM_COL32(255, 80, 80, 150),                 // 빨간색 계열의 데드라인 색상
        2.0f                                        // 두께
    );
    // 데드라인 선 위에 텍스트 추가 (선택적)
    draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize(), ImVec2(deadline_x - 50, p.y + 5), IM_COL32_WHITE, "Deadline");


    // --- 태스크 막대 그리기 ---
    const auto& nodes = querier.getGraphNodes();
    const TaskStats* task_stats_array = querier.getTaskStatsArray();
    float y_offset = 5.0f;
    const float bar_height = 15.0f;

    y_offset += 20.0f;
    for (const auto& node : nodes) {
        if (node.frequency != frequency || node.is_non_rt) continue;

        const auto& stats = task_stats_array[node.task_id];
        
        // Live 데이터 읽기 (ns -> us)
        float queued_offset_us = stats.last_pushed_to_queue_offset_ns.load() / 1000.0f;
        float start_offset_us = stats.last_start_offset_ns.load() / 1000.0f;
        float completion_offset_us = stats.last_completion_offset_ns.load() / 1000.0f;

        // X축 위치(비율) 계산
        float queued_x_ratio = queued_offset_us / period_us;
        float start_x_ratio = start_offset_us / period_us;
        float end_x_ratio = completion_offset_us / period_us;

        // X축 화면 좌표 계산
        float queued_x = p.x + chart_width * queued_x_ratio;
        float start_x = p.x + chart_width * start_x_ratio;
        float end_x = p.x + chart_width * end_x_ratio;
        
        // Y축 화면 좌표 계산
        ImVec2 bar_p_min = ImVec2(p.x, p.y + y_offset);
        ImVec2 bar_p_max = ImVec2(p.x, p.y + y_offset + bar_height);

        // --- << [핵심] Latency와 Execution 바를 분리하여 그리기 >> ---
        
        // 1. Latency 바 (흐린 선 또는 옅은 막대)
        //    - 큐에 들어간 시점부터 실제 실행 시작 시점까지
        if (start_x > queued_x) {
            draw_list->AddRectFilled(
                ImVec2(queued_x, bar_p_min.y + bar_height * 0.3f), // 중앙에 얇게
                ImVec2(start_x, bar_p_max.y - bar_height * 0.3f), 
                IM_COL32(255, 165, 0, 150) // 주황색 계열 (대기)
            );
        }

        // 2. Execution 바 (굵은 막대)
        //    - 실제 실행 시작 시점부터 완료 시점까지
        if (end_x > start_x) {
            draw_list->AddRectFilled(
                ImVec2(start_x, bar_p_min.y), 
                ImVec2(end_x, bar_p_max.y), 
                IM_COL32(50, 120, 255, 200), // 파란색 계열 (실행)
                3.0f
            );
        }
        // ----------------------------------------------------

        // 태스크 이름 표시
        draw_list->AddText(ImVec2(p.x + 5, bar_p_min.y + 1), IM_COL32_WHITE, node.task_name);

        // 툴팁: 전체 바 영역에 대해
        if (ImGui::IsMouseHoveringRect(ImVec2(queued_x, bar_p_min.y), ImVec2(end_x, bar_p_max.y))) {
            float latency_us = start_offset_us - queued_offset_us;
            float exec_time_us = completion_offset_us - start_offset_us;
            ImGui::SetTooltip("Task: %s\nLatency: %.2f us\nExec Time: %.2f us\nStart Offset: %.2f us", 
                node.task_name, latency_us, exec_time_us, start_offset_us);
        }

        y_offset += (bar_height + 5.0f);
    }

    ImGui::EndChild();
};

static void render_timeline_details_tab(SharedMemoryQuerier& querier, SharedMemoryController& controller, int selected_freq, StatsViewMode view_mode) {
    if (selected_freq <= 0) {
        ImGui::TextWrapped("No timeline selected. Click a timeline in the Main Dashboard to see its details.");
        return;
    }

    // --- 1. 요약 통계 테이블 ---
    ImGui::SeparatorText((std::to_string(selected_freq) + " Hz Timeline Summary").c_str());
    if (ImGui::BeginTable("timeline_summary_table", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Period(us)");
        ImGui::TableSetupColumn("Avg Busy(us)");
        ImGui::TableSetupColumn("Peak Busy(us)");
        ImGui::TableSetupColumn("Load(%)");
        ImGui::TableSetupColumn("Misses");
        ImGui::TableHeadersRow();

        const auto& recent_stats = timeline_recent_stats_map.at(selected_freq);
        const TimelineStats* shm_stats = nullptr;
        for (auto i=0; i<querier.getTimelineStatsCount(); i++) {
            if (querier.getTimelineStatsArray()[i].frequency == selected_freq)
                shm_stats = &querier.getTimelineStatsArray()[i];
        }

        if(shm_stats) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%.2f", 1000000.0 / selected_freq);
            ImGui::TableNextColumn(); 
            if (view_mode == StatsViewMode::RECENT) {
                ImGui::Text("%.2f (±%.2f)", recent_stats.avg_busy_us, recent_stats.stdev_busy_us);
            } else {
                // Lifetime 평균 계산 및 표시
            }
            ImGui::TableNextColumn(); ImGui::Text("%.2f", shm_stats->max_busy_ns.load() / 1000.0);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", (recent_stats.avg_busy_us / (1000000.0 / selected_freq)) * 100.0);
            ImGui::TableNextColumn(); ImGui::Text("%lld", shm_stats->deadline_miss_count.load());
        }

        ImGui::EndTable();
    }
    ImGui::Spacing();

    // --- 2. 소속 태스크 목록 테이블 ---
    ImGui::SeparatorText("Member Tasks");
    // 기존 render_task_stats_tab 함수를 재활용하되, 필터링 기능 추가
    render_task_stats_tab(querier, controller, 
        [selected_freq](const TaskGraphNodeInfo& node){
            return node.frequency == selected_freq;
        }
    );
    ImGui::Spacing();

    // --- 3. 간트 차트 ---
    render_gantt_chart_for_timeline(querier, selected_freq, view_mode);
}

inline ImVec4 get_log_level_color(LogLevel level) {
        switch (level) {
            case LogLevel::TRACE:    return ImVec4(0.5f, 0.5f, 0.5f, 1.0f);    // Dark Gray (가장 덜 중요)
            case LogLevel::DEBUG:    return ImVec4(0.7f, 0.7f, 0.7f, 1.0f);    // Light Gray
            case LogLevel::INFO:     return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);    // White (일반 정보)
            case LogLevel::WARN:     return ImVec4(1.0f, 1.0f, 0.0f, 1.0f);    // Yellow (경고)
            case LogLevel::ERROR:    return ImVec4(1.0f, 0.2f, 0.2f, 1.0f);    // Red (오류)
            case LogLevel::CRITICAL: return ImVec4(1.0f, 0.0f, 1.0f, 1.0f);    // Magenta (치명적 오류)
            default:                       return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        }
    }

}; // end anonymous namespace





void render_main_dashboard_window(SharedMemoryQuerier& querier) {
    const SharedMemoryHeader* header = querier.getHeader();
    if (!header) return;

    // --- 창 설정 (화면 좌측) ---
    // --- [1. 창의 위치와 크기 설정] ---
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    // 화면 좌상단에 위치, 너비는 화면의 30%, 높이는 50%
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.4f, viewport->WorkSize.y * 0.5f));

    // --- [2. 창 이동 및 크기 조절 방지 플래그] ---
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    update_all_recent_stats(querier);

    if (ImGui::Begin("Main Dashboard", nullptr, window_flags)) {
        // --- 섹션 1: Framework Status (기존 코드 재활용) ---
        ImGui::Text("Framework Tick: %lu", header->framework_tick_count.load());
        ImGui::Separator();

        // --- 섹션 2: Timeline Load Overview ---
        ImGui::SeparatorText("Timeline Overview");
        const TimelineStats* timelines = querier.getTimelineStatsArray();
        size_t timeline_count = querier.getTimelineStatsCount();

        for (size_t i = 0; i < timeline_count; ++i) {
            const auto& stats = timelines[i];
            if (stats.tick_count.load() == 0) continue;

            // 해당 타임라인의 'Recent' 통계를 맵에서 가져옴
            const auto& recent_stats = timeline_recent_stats_map[stats.frequency];
            
            // 헬퍼 함수 호출
            render_timeline_progressbar(stats, recent_stats, &selected_timeline_freq);
        }
        ImGui::Spacing();

        // --- 섹션 3: Thread Pool Usage Overview (신규 핵심 로직) ---
        ImGui::SeparatorText("Thread Pool Usage Overview");
        auto now = std::chrono::steady_clock::now();
        long long elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_stats_update_time).count();
        if (elapsed_ns <= 0) elapsed_ns = 1;

        const PoolStats* pool_stats_array = querier.getPoolStatsArray();
        const size_t pool_count = querier.getPoolStatsCount();

        for (auto i=0; i<pool_count; i++) {
            const auto& shm_stats = pool_stats_array[i];
            const auto& recent_stats = pool_recent_stats_map[i]; // 인덱스 0 사용
            const PoolStats& stats = pool_stats_array[i];
            render_pool_progressbar(stats.name, stats.num_workers, recent_stats);
        }

        // --- << [핵심 수정] 섹션 4: Controls & Status >> ---
        ImGui::Separator();

        // 1. 그래프 보기 버튼
        // 버튼을 창 너비에 꽉 채워서 더 보기 좋게 만듦
        if (ImGui::Button("Show Dependency Graph", ImVec2(-1, 0))) {
            rtfw::monitor::show_dependency_graph_popup = true;
        }

        ImGui::Spacing();
        
        // 2. 연결 상태 표시 (기존 Status 창 로직 통합)
        const SharedMemoryHeader* header = querier.getHeader(); // 이미 위에서 가져왔음
        ShmState state = header ? header->shm_state.load(std::memory_order_relaxed) : ShmState::UNINITIALIZED;

        const char* status_text = "Unknown";
        ImVec4 status_color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow (기본)

        switch (state) {
            case ShmState::RUNNING:
                status_text = "CONNECTED (RUNNING)";
                status_color = ImVec4(0.0f, 1.0f, 0.0f, 1.0f); // Green
                break;
            case ShmState::INITIALIZING:
                status_text = "CONNECTING (INITIALIZING...)";
                status_color = ImVec4(0.5f, 0.5f, 1.0f, 1.0f); // Blue
                break;
            case ShmState::SHUTTING_DOWN:
                status_text = "CONNECTED (SHUTTING DOWN)";
                status_color = ImVec4(1.0f, 0.5f, 0.0f, 1.0f); // Orange
                break;
            case ShmState::UNINITIALIZED:
            default:
                status_text = "DISCONNECTED";
                status_color = ImVec4(1.0f, 0.2f, 0.2f, 1.0f); // Red
                break;
        }

        ImGui::Text("Status: ");
        ImGui::SameLine();
        ImGui::TextColored(status_color, "%s", status_text);
        ImGui::SameLine();
        if (ImGui::Button("Refresh")) {
            rtfw::monitor::req_refresh = true;
        }
    }
    ImGui::End();
}


void render_analysis_center_window(SharedMemoryQuerier& querier, SharedMemoryController& controller) {

    // --- 창 설정 (화면 좌측) ---
    // --- [1. 창의 위치와 크기 설정] ---
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    // 화면 좌상단에 위치, 너비는 화면의 30%, 높이는 50%
    ImGui::SetNextWindowPos(viewport->WorkPos + ImVec2(viewport->WorkSize.x * 0.4f, 0.0f));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.6f, viewport->WorkSize.y * 1.0f));

    // --- [2. 창 이동 및 크기 조절 방지 플래그] ---
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("Analysis Center", nullptr, window_flags)) {

        // --- << [신규] 전역 통계 뷰 컨트롤러 >> ---
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 3));
        if (ImGui::RadioButton("Recent", current_view_mode == StatsViewMode::RECENT)) { 
            current_view_mode = StatsViewMode::RECENT; 
        } 
        ImGui::SameLine();
        if (ImGui::RadioButton("Lifetime", current_view_mode == StatsViewMode::LIFETIME)) { 
            current_view_mode = StatsViewMode::LIFETIME; 
        }
        
        // Recent 모드일 때만 시간 간격 선택 UI 활성화
        ImGui::BeginDisabled(current_view_mode != StatsViewMode::RECENT);
        ImGui::SameLine(); ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 20);
        ImGui::Text("Interval:"); ImGui::SameLine();
        if (ImGui::RadioButton("1s", current_interval == RecentInterval::ONE_SEC)) { 
            current_interval = RecentInterval::ONE_SEC; 
        } 
        ImGui::SameLine();
        if (ImGui::RadioButton("10s", current_interval == RecentInterval::TEN_SEC)) { 
            current_interval = RecentInterval::TEN_SEC; 
        }
        ImGui::EndDisabled();
        ImGui::PopStyleVar();
        ImGui::Separator();
        // --- << 컨트롤러 종료 >> ---


        // --- 탭 바 시작 ---
        if (ImGui::BeginTabBar("AnalysisTabs")) {
            
            if (ImGui::BeginTabItem("Task Stats")) {
                // 이제 이 테이블 렌더링 함수는 current_view_mode를 확인하여
                // Recent 통계를 보여줄지 Lifetime 통계를 보여줄지 결정해야 합니다.
                // render_task_stats_table(querier, controller, current_view_mode);
                render_task_stats_tab(querier, controller);
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Timeline Details")) { // << 이름 변경 제안
                // Timeline의 상세 정보 테이블
                // render_timeline_stats_table(querier, current_view_mode);
                render_timeline_sync_diagram(querier, &selected_timeline_freq);
                render_timeline_details_tab(querier, controller, selected_timeline_freq, current_view_mode);
                ImGui::EndTabItem();
            }

            // if (ImGui::BeginTabItem("Dependency Graph")) {
            //     // 의존성 그래프
            //     render_dependency_graph_window(querier);
            //     ImGui::EndTabItem();
            // }

            // ... Pool Stats, Gantt Chart 탭들 ...
            // Gantt Chart도 view_mode에 따라 Recent 평균 또는 Lifetime 평균을 기반으로 그릴 수 있습니다.

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}

static std::vector<LogEntry> log_history; // 로그를 저장할 버퍼
void render_live_log_window(SharedMemoryQuerier& querier, SharedMemoryController& controller) {
    // ... 창 설정 ...
    const SharedMemoryHeader* header = querier.getHeader();
    if (!header) return;

    // --- 창 설정 (화면 좌측) ---
    // --- [1. 창의 위치와 크기 설정] ---
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    // 화면 좌상단에 위치, 너비는 화면의 30%, 높이는 50%
    ImGui::SetNextWindowPos(viewport->WorkPos + ImVec2(0.0f, viewport->WorkSize.y * 0.5f));
    ImGui::SetNextWindowSize(ImVec2(viewport->WorkSize.x * 0.4f, viewport->WorkSize.y * 0.5f));

    // --- [2. 창 이동 및 크기 조절 방지 플래그] ---
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("Live Log", nullptr, window_flags)) {
        static bool is_log_paused = false;
        static char log_text_filter[256] = ""; // 텍스트 필터 입력 버퍼
        static int current_control_level_idx = 2; // INFO가 기본값이라고 가정
        const char* log_level_names[] = { "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "CRITICAL" };
        // 순서: TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL
        static bool log_level_filters[6] = { false, false, true, true, true, true };

        // --- << [핵심] Pause 버튼 UI >> ---
        if (ImGui::Button(is_log_paused ? "Resume" : "Pause")) {
            is_log_paused = !is_log_paused;
        }
        // ImGui::SameLine();
        // if (ImGui::Button("Clear")) {
        //     log_history.clear();
        // }

        // --- [핵심] 로그 레벨 제어 ComboBox ---
        ImGui::SameLine();
        ImGui::Text("Level:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80); // 콤보박스 너비 설정
        if (ImGui::Combo("##LogLevelControl", &current_control_level_idx, log_level_names, IM_ARRAYSIZE(log_level_names))) {
            // 콤보박스 값이 변경되었을 때, 컨트롤러를 통해 프레임워크에 알림
            controller.setLogLevel(static_cast<LogLevel>(current_control_level_idx));
        }

        // --- 레벨 필터링 체크박스 ---
        ImGui::SameLine();
        ImGui::Text("Show:");
        ImGui::BeginDisabled(current_control_level_idx > 0);
        ImGui::SameLine(); ImGui::Checkbox("T", &log_level_filters[0]); // TRACE
        ImGui::EndDisabled();
        ImGui::BeginDisabled(current_control_level_idx > 1);
        ImGui::SameLine(); ImGui::Checkbox("D", &log_level_filters[1]); // DEBUG
        ImGui::EndDisabled();
        ImGui::BeginDisabled(current_control_level_idx > 2);
        ImGui::SameLine(); ImGui::Checkbox("I", &log_level_filters[2]); // INFO
        ImGui::EndDisabled();
        ImGui::BeginDisabled(current_control_level_idx > 3);
        ImGui::SameLine(); ImGui::Checkbox("W", &log_level_filters[3]); // WARN
        ImGui::EndDisabled();
        ImGui::BeginDisabled(current_control_level_idx > 4);
        ImGui::SameLine(); ImGui::Checkbox("E", &log_level_filters[4]); // ERROR
        ImGui::EndDisabled();
        ImGui::BeginDisabled(current_control_level_idx > 5);
        ImGui::SameLine(); ImGui::Checkbox("C", &log_level_filters[5]); // CRITICAL
        ImGui::EndDisabled();

        // --- << 필터 바 UI >> ---
        // 1. 텍스트 필터
        ImGui::PushItemWidth(-1); // 너비를 꽉 채움
        ImGui::InputTextWithHint("##Filter", "Filter logs... (e.g., 'Sensor' or 'ERROR')", log_text_filter, IM_ARRAYSIZE(log_text_filter));
        ImGui::PopItemWidth();

        // ---------------------------------
        // ... 기타 필터링 UI ...
        ImGui::Separator();
        
        // ------------------------------------

        ImGui::BeginChild("LogScrollingRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

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
            // 수평 스크롤
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                ImGui::SetScrollX(ImGui::GetScrollX() - ImGui::GetIO().MouseDelta.x);
            }
        }


        // --- << Pause 상태에 따른 로직 분기 >> ---
        // 일시정지 상태가 아닐 때만 새로운 로그를 가져옵니다.
        if (!is_log_paused) {
            auto new_logs = querier.getAllLogEntries();
            log_history.clear();
            if (!new_logs.empty()) {
                log_history.insert(log_history.end(), 
                                   std::make_move_iterator(new_logs.begin()), 
                                   std::make_move_iterator(new_logs.end()));
                // 버퍼 크기 제한 로직
                if (log_history.size() > 2048) {
                    log_history.erase(log_history.begin(), log_history.begin() + (log_history.size() - 2048));
                }
            }
        }
        // ----------------------------------------------------

        // 2. 로그 렌더링
        for (const auto& log : log_history) {
            // --- << [핵심] 필터링 로직 >> ---
            // 1. 레벨 필터: 해당 레벨이 꺼져있으면 건너뜀
            if (!log_level_filters[static_cast<int>(log.level)]) {
                continue;
            }
            // 2. 텍스트 필터: 필터 텍스트가 비어있지 않고,
            //    컴포넌트 이름이나 메시지에 해당 텍스트가 없으면 건너뜀
            if (log_text_filter[0] != '\0' &&
                strstr(log.task_name, log_text_filter) == nullptr &&
                strstr(log.message, log_text_filter) == nullptr)
            {
                continue;
            }

            // 1. 로그 레벨에 따른 색상과 텍스트를 결정
            ImVec4 level_color;
            const char* level_text;
            switch (log.level) {
                case LogLevel::TRACE:    level_text = "[TRACE]"; level_color = ImVec4(0.5f, 0.5f, 0.5f, 1.0f); break;
                case LogLevel::DEBUG:    level_text = "[DEBUG]"; level_color = ImVec4(0.7f, 0.7f, 0.7f, 1.0f); break;
                case LogLevel::INFO:     level_text = "[INFO] "; level_color = ImVec4(0.2f, 0.8f, 0.2f, 1.0f); break; // 초록색
                case LogLevel::WARN:     level_text = "[WARN] "; level_color = ImVec4(1.0f, 1.0f, 0.0f, 1.0f); break; // 노란색
                case LogLevel::ERROR:    level_text = "[ERROR]"; level_color = ImVec4(1.0f, 0.2f, 0.2f, 1.0f); break; // 빨간색
                case LogLevel::CRITICAL: level_text = "[CRIT] "; level_color = ImVec4(1.0f, 0.0f, 1.0f, 1.0f); break; // 자홍색
                default:                 level_text = "[UNKWN]"; level_color = ImVec4(1.0f, 1.0f, 1.0f, 1.0f); break;
            }

            // 2. 여러 부분으로 나누어 텍스트를 그림
            ImGui::PushID(&log); // 각 라인에 고유 ID 부여 (성능에 거의 영향 없음)
            
            // Part 2: 로그 레벨 (색상 적용)
            ImGui::TextColored(level_color, "%s", level_text);
            
            ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x); // 줄바꿈 없이 바로 옆에 이어서 그림
            
            // // Part 1: 컴포넌트 이름 (기본 색상)
            // ImGui::TextColored(level_color, "["); 
            // ImGui::SameLine(0, 0);
            // ImGui::TextColored(level_color, "%s", log.task_name); // task_name은 이미 [Framework] 포맷이 아님
            // ImGui::SameLine(0, 0);
            // ImGui::TextColored(level_color, "]");

            // ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);

            // Part 3: 메시지 본문 (기본 색상)
            ImGui::TextUnformatted(log.message);

            ImGui::PopID();
        }

        // --- << [핵심] Pause 상태에 따른 자동 스크롤 제어 >> ---
        // 일시정지 상태가 아니고, 사용자가 직접 스크롤하지 않았을 때만 자동 스크롤
        if (!is_log_paused && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
            ImGui::SetScrollHereY(1.0f);
        }
        // --------------------------------------------------------
        
        ImGui::EndChild();
    }
    ImGui::End();
}

void display_parameters(const SharedMemoryQuerier& querier) {
    // 1. 모든 파라미터 목록을 가져와서 UI에 표시
    std::vector<ParameterInfo> all_params = querier.getAllParameterInfos();
    
    for (const auto& info : all_params) {
        ImGui::Text("Name: %s", info.key);
        
        // 2. 현재 값을 읽어서 표시 (타입을 미리 알고 있거나, type_hash로 분기)
        // 실제로는 type_hash를 보고 분기해야 함
        auto val = querier.getParameterValue<double>(info.key);
        if (val.has_value()) {
            ImGui::SameLine();
            ImGui::Text("Value: %.2f", *val);
        }
    }
}