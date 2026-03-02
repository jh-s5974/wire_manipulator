// apps/monitor_tool/stats_renderer.h
#pragma once

#include <rtfw_connect/client.h> // Querier, Controller를 사용하기 위함
#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>


using namespace rtfw::connect;

// 시스템 및 태스크 통계 관련 렌더링 함수들을 선언합니다.
void render_main_dashboard_window(SharedMemoryQuerier& querier);
void render_analysis_center_window(SharedMemoryQuerier& querier, SharedMemoryController& controller);
void render_live_log_window(SharedMemoryQuerier& querier, SharedMemoryController& controller);

// void render_system_stats_window(SharedMemoryQuerier& querier);
// void render_task_list_window(SharedMemoryQuerier& querier, SharedMemoryController& controller);
