// rtfw/include/rtfw/framework.h
// version: purism

#pragma once

#define RTFW_VERSION_MAJOR 0
#define RTFW_VERSION_MINOR 2
#define RTFW_VERSION_REVISION 1

// --- 사용자가 직접 사용할 핵심 API 헤더들 ---
#include "rtfw/rt_framework.h"
#include "rtfw/task.h"
#include "rtfw/task_proxies.h"
#include "rtfw_common/blackbox.h"
#include "rtfw_common/FileBlackbox.h"
// #include "rtfw_common/data_archive.h"
// #include "rtfw_common/file_archive.h"