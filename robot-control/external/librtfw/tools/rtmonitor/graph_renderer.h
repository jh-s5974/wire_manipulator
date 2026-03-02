// apps/monitor_tool/graph_renderer.h
#pragma once

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui.h>
#include <rtfw_connect/client.h> // SharedMemoryQuerier를 사용하기 위함


using namespace rtfw::connect;

// 그래프 렌더링 함수를 선언합니다.
void render_dependency_graph_window(SharedMemoryQuerier& querier);
void render_timeline_sync_diagram(SharedMemoryQuerier& querier, int* selected_timeline_freq_ptr);