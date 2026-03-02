#pragma once
#include <cstdint>
#include <cmath>

// ============================================================
// test_app data types – all POD / trivially-copyable
// ============================================================

// ---------- raw / filtered 1 kHz data -----------------------
struct SampleData16f {
    float v[16];   // 64 bytes – simulates ADC burst
};

struct SampleData8f {
    float v[8];    // 32 bytes – filtered / decimated
};

// ---------- 500 Hz / 200 Hz fused & estimated data ----------
struct StateVec6d {
    double v[6];   // 48 bytes – state vector (pos, vel, acc per axis)
};

struct Vec4f {
    float x, y, z, w;  // 16 bytes – compact command / plan value
};

// ---------- diagnostics / monitoring ------------------------
struct AggregatedStats {
    double   mean;
    double   variance;
    double   min_val;
    double   max_val;
    uint64_t sample_tick;  // tick when accumulated
    uint32_t count;
    uint32_t _pad;         // keep 8-byte aligned
};  // 48 bytes

struct DiagnosticData {
    uint64_t tick;
    uint32_t error_flags;   // bit-packed health flags
    float    latency_us;    // round-trip latency estimate
};  // 16 bytes

// ---------- non-RT parameter data ---------------------------
struct ParamData {
    double gain;
    double offset;
    double scale;
};  // 24 bytes

// ---------- stateful task internal state (Task<StateT>) -----
struct RollingAccState {
    double   sum;
    double   sum_sq;
    double   min_val;
    double   max_val;
    uint64_t count;
    uint32_t window;    // rolling window size
    uint32_t _pad;
};  // 48 bytes – must be trivially copyable

// State for 500 Hz integrating estimators (checkpointed so replay can restore
// the warm-up trajectory and avoid cold-start divergence for non-archived keys).
struct EstState6d {
    float pos[6];
    float vel[6];
};  // 48 bytes – trivially copyable

struct EstStateSummary {
    double a[6];   // last blended Est_C result
    double _pad;
};  // 56 bytes

// ---------- scalar value (derived / non-archived check) -----
// Used to verify replay reproducibility of non-archived keys.
struct ScalarVal {
    double v;
    double _pad;    // keep 16-byte size, 8-byte aligned
};  // 16 bytes

// ---------- summary (archive-enabled, queryable) ------------
struct SummaryOutput {
    AggregatedStats stats[4];  // slots A–D
    uint64_t        tick;
    uint32_t        active_tasks;
    uint32_t        overrun_flags;
};  // 208 bytes
