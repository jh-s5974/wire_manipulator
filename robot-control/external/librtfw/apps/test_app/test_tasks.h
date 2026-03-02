#pragma once

#include "rtfw/task.h"
#include "test_data_types.h"
#include <cmath>
#include <algorithm>
#include <limits>

using namespace rtfw::rt;

// ============================================================
// ▌ 1 kHz – RT tasks (dedicated core)
// ▌ 8 tasks: 4 generators + 3 filters + 1 controller
// ============================================================

/// Generates synthetic sensor burst on channel A  (1 kHz)
class Gen1k_A : public ITask {
public:
    const char* getName() const override { return "Gen1k_A"; }
    void execute(void*) override {
        const double t = getExecutionLocalTick() * 0.001;
        SampleData16f d;
        for (int i = 0; i < 16; ++i)
            d.v[i] = static_cast<float>(std::sin(t * (i + 1) * 0.628318));
        w_.write(d);
    }
private:
    DataWriter<SampleData16f> w_{"raw.1k.a", ArchiveOption::Enable};
};

/// Generates synthetic sensor burst on channel B  (1 kHz)
class Gen1k_B : public ITask {
public:
    const char* getName() const override { return "Gen1k_B"; }
    void execute(void*) override {
        const double t = getExecutionLocalTick() * 0.001;
        SampleData16f d;
        for (int i = 0; i < 16; ++i)
            d.v[i] = static_cast<float>(std::cos(t * (i + 1) * 0.628318));
        w_.write(d);
    }
private:
    DataWriter<SampleData16f> w_{"raw.1k.b", ArchiveOption::Enable};
};

/// Generates synthetic sensor burst on channel C  (1 kHz)
class Gen1k_C : public ITask {
public:
    const char* getName() const override { return "Gen1k_C"; }
    void execute(void*) override {
        const double t = getExecutionLocalTick() * 0.001;
        SampleData16f d;
        for (int i = 0; i < 16; ++i)
            d.v[i] = static_cast<float>(std::sin(t * 1.41421 + i * 0.3927));
        w_.write(d);
    }
private:
    DataWriter<SampleData16f> w_{"raw.1k.c", ArchiveOption::Enable};
};

/// Generates synthetic sensor burst on channel D  (1 kHz)
class Gen1k_D : public ITask {
public:
    const char* getName() const override { return "Gen1k_D"; }
    void execute(void*) override {
        const double t = getExecutionLocalTick() * 0.001;
        SampleData16f d;
        for (int i = 0; i < 16; ++i)
            d.v[i] = static_cast<float>(std::cos(t * 2.71828 - i * 0.1963));
        w_.write(d);
    }
private:
    DataWriter<SampleData16f> w_{"raw.1k.d", ArchiveOption::Enable};
};

/// EMA filter on channel A  (1 kHz)
class Filter1k_A : public ITask {
public:
    const char* getName() const override { return "Filter1k_A"; }
    void execute(void*) override {
        r_.on_update([this](const SampleData16f& in) {
            SampleData8f d;
            for (int i = 0; i < 8; ++i) {
                state_[i] = 0.9f * state_[i] + 0.1f * in.v[i];
                d.v[i] = state_[i];
            }
            w_.write(d);
        });
    }
private:
    DataReader<SampleData16f> r_{"raw.1k.a", DependencyType::Strong};
    DataWriter<SampleData8f>  w_{"filt.1k.a", ArchiveOption::Enable};
    float state_[8]{};
};

/// EMA filter on channel B  (1 kHz)
class Filter1k_B : public ITask {
public:
    const char* getName() const override { return "Filter1k_B"; }
    void execute(void*) override {
        r_.on_update([this](const SampleData16f& in) {
            SampleData8f d;
            for (int i = 0; i < 8; ++i) {
                state_[i] = 0.9f * state_[i] + 0.1f * in.v[i];
                d.v[i] = state_[i];
            }
            w_.write(d);
        });
    }
private:
    DataReader<SampleData16f> r_{"raw.1k.b", DependencyType::Strong};
    DataWriter<SampleData8f>  w_{"filt.1k.b", ArchiveOption::Enable};
    float state_[8]{};
};

/// Cross-channel filter: blends C and prior A result  (1 kHz)
class Filter1k_C : public ITask {
public:
    const char* getName() const override { return "Filter1k_C"; }
    void execute(void*) override {
        r_c_.on_update([this](const SampleData16f& in) {
            SampleData8f d;
            for (int i = 0; i < 8; ++i) {
                float fb = r_a_ ? r_a_->v[i] : 0.0f;
                state_[i] = 0.85f * state_[i] + 0.1f * in.v[i] + 0.05f * fb;
                d.v[i] = state_[i];
            }
            w_.write(d);
        });
    }
private:
    DataReader<SampleData16f> r_c_{"raw.1k.c", DependencyType::Strong};
    DataReader<SampleData8f>  r_a_{"filt.1k.a", DependencyType::Weak};
    DataWriter<SampleData8f>  w_{"filt.1k.c", ArchiveOption::Enable};
    float state_[8]{};
};

/// PD-style controller using filtered A+B+C  (1 kHz)
class Ctrl1k : public ITask {
public:
    const char* getName() const override { return "Ctrl1k"; }
    void execute(void*) override {
        if (!r_a_ || !r_b_ || !r_c_) return;
        const auto& a = *r_a_;
        const auto& b = *r_b_;
        const auto& c = *r_c_;
        Vec4f cmd;
        cmd.x = (a.v[0] - b.v[0]) * 10.0f + c.v[0] * 2.0f;
        cmd.y = (a.v[1] - b.v[1]) * 10.0f + c.v[1] * 2.0f;
        cmd.z = (a.v[2] - b.v[2]) * 10.0f + c.v[2] * 2.0f;
        cmd.w = a.v[3] * b.v[3];
        w_.write(cmd);
    }
private:
    DataReader<SampleData8f> r_a_{"filt.1k.a", DependencyType::Strong};
    DataReader<SampleData8f> r_b_{"filt.1k.b", DependencyType::Strong};
    DataReader<SampleData8f> r_c_{"filt.1k.c", DependencyType::Strong};
    DataWriter<Vec4f>        w_{"cmd.1k.ctrl", ArchiveOption::Enable};
};


// ============================================================
// ▌ 500 Hz – RT tasks (dedicated core)
// ▌ 6 tasks: 3 fused estimators + 3 state estimators
// ============================================================

/// Fuses filtered channels A+B  (500 Hz)
class Fuse500_A : public ITask {
public:
    const char* getName() const override { return "Fuse500_A"; }
    void execute(void*) override {
        SampleData8f d;
        const auto* a = r_a_ ? &*r_a_ : nullptr;
        const auto* b = r_b_ ? &*r_b_ : nullptr;
        for (int i = 0; i < 8; ++i)
            d.v[i] = 0.5f * (a ? a->v[i] : 0.f) + 0.5f * (b ? b->v[i] : 0.f);
        w_.write(d);
    }
private:
    DataReader<SampleData8f> r_a_{"filt.1k.a", DependencyType::Weak};
    DataReader<SampleData8f> r_b_{"filt.1k.b", DependencyType::Weak};
    DataWriter<SampleData8f> w_{"fused.500.ab", ArchiveOption::Enable};
};

/// Fuses filtered channel C + raw D  (500 Hz)
class Fuse500_B : public ITask {
public:
    const char* getName() const override { return "Fuse500_B"; }
    void execute(void*) override {
        SampleData8f d;
        const auto* c = r_c_ ? &*r_c_ : nullptr;
        const auto* dw = r_d_ ? &*r_d_ : nullptr;
        for (int i = 0; i < 8; ++i) {
            float cv = c ? c->v[i] : 0.f;
            float dv = dw ? dw->v[i] : 0.f;
            d.v[i] = cv * 0.7f + dv * 0.3f;
        }
        w_.write(d);
    }
private:
    DataReader<SampleData8f>  r_c_{"filt.1k.c",  DependencyType::Weak};
    DataReader<SampleData16f> r_d_{"raw.1k.d",   DependencyType::Weak};
    DataWriter<SampleData8f>  w_{"fused.500.b",  ArchiveOption::Enable};
};

/// Further fuses AB side (Strong)  (500 Hz)
class Fuse500_C : public ITask {
public:
    const char* getName() const override { return "Fuse500_C"; }
    void execute(void*) override {
        r_ab_.on_update([this](const SampleData8f& ab) {
            StateVec6d d;
            for (int i = 0; i < 6; ++i)
                d.v[i] = ab.v[i] * 0.001;  // unit scaling example
            w_.write(d);
        });
    }
private:
    DataReader<SampleData8f> r_ab_{"fused.500.ab", DependencyType::Strong};
    DataWriter<StateVec6d>   w_{"fused.500.abc",   ArchiveOption::Enable};
};

/// State estimator A – integrates fused data  (500 Hz)
/// Uses Task<EstState6d> so pos/vel survive checkpoint → replay is warm-started.
class Est500_A : public Task<EstState6d> {
public:
    const char* getName() const override { return "Est500_A"; }
    void execute(EstState6d& s) override {
        const float dt = 0.002f;
        const auto* ab = r_ab_ ? &*r_ab_ : nullptr;
        const auto* b  = r_b_  ? &*r_b_  : nullptr;
        StateVec6d d;
        for (int i = 0; i < 6; ++i) {
            float obs = (ab ? ab->v[i % 8] : 0.f) + (b ? b->v[i % 8] : 0.f);
            s.vel[i] += obs * dt;
            s.pos[i] += s.vel[i] * dt;
            d.v[i] = s.pos[i];
        }
        w_.write(d);
    }
private:
    DataReader<SampleData8f> r_ab_{"fused.500.ab", DependencyType::Weak};
    DataReader<SampleData8f> r_b_{ "fused.500.b",  DependencyType::Weak};
    DataWriter<StateVec6d>   w_{"est.500.a"};
};

/// State estimator B – blends ctrl command  (500 Hz)
/// Uses Task<EstState6d> for checkpoint-able accumulation.
class Est500_B : public Task<EstState6d> {
public:
    const char* getName() const override { return "Est500_B"; }
    void execute(EstState6d& s) override {
        r_a_.on_update([this, &s](const StateVec6d& prev) {
            const Vec4f* cmd = r_cmd_ ? &*r_cmd_ : nullptr;
            StateVec6d d;
            for (int i = 0; i < 6; ++i) {
                float correction = cmd ? cmd->x * 0.001f : 0.f;
                s.vel[i] = static_cast<float>(prev.v[i] * 0.98 + correction);
                d.v[i] = s.vel[i];
            }
            w_.write(d);
        });
    }
private:
    DataReader<StateVec6d> r_a_  {"est.500.a",    DependencyType::Strong};
    DataReader<Vec4f>      r_cmd_{"cmd.1k.ctrl",  DependencyType::Weak};
    DataWriter<StateVec6d> w_{"est.500.b"};
};

/// State estimator C – final fusion  (500 Hz)
/// Uses Task<EstStateSummary> so last blended result is checkpoint-able.
class Est500_C : public Task<EstStateSummary> {
public:
    const char* getName() const override { return "Est500_C"; }
    void execute(EstStateSummary& s) override {
        if (!r_a_ || !r_b_) return;
        const auto& a = *r_a_;
        const auto& b = *r_b_;
        StateVec6d d;
        for (int i = 0; i < 6; ++i) {
            d.v[i] = 0.6 * a.v[i] + 0.4 * b.v[i];
            s.a[i] = d.v[i];
        }
        w_.write(d);
    }
private:
    DataReader<StateVec6d> r_a_{"est.500.a", DependencyType::Strong};
    DataReader<StateVec6d> r_b_{"est.500.b", DependencyType::Strong};
    DataWriter<StateVec6d> w_{"est.500.c",   ArchiveOption::Enable};
};


// ============================================================
// ▌ 200 Hz – RT tasks (dedicated core)
// ▌ 5 tasks: 3 planners + 2 diagnostics
// ============================================================

/// Planner A based on estimated state A  (200 Hz)
class Plan200_A : public ITask {
public:
    const char* getName() const override { return "Plan200_A"; }
    void execute(void*) override {
        const StateVec6d* s = r_est_ ? &*r_est_ : nullptr;
        Vec4f cmd;
        cmd.x = s ? static_cast<float>(-s->v[0] * 2.0) : 0.f;
        cmd.y = s ? static_cast<float>(-s->v[1] * 2.0) : 0.f;
        cmd.z = s ? static_cast<float>(-s->v[2] * 1.0) : 0.f;
        cmd.w = 1.0f;
        w_.write(cmd);
    }
private:
    DataReader<StateVec6d> r_est_{"est.500.a", DependencyType::Weak};
    DataWriter<Vec4f>      w_{"plan.200.a"};
};

/// Planner B based on estimated state B  (200 Hz)
class Plan200_B : public ITask {
public:
    const char* getName() const override { return "Plan200_B"; }
    void execute(void*) override {
        const StateVec6d* s = r_est_ ? &*r_est_ : nullptr;
        Vec4f cmd;
        cmd.x = s ? static_cast<float>(-s->v[3] * 1.5) : 0.f;
        cmd.y = s ? static_cast<float>(-s->v[4] * 1.5) : 0.f;
        cmd.z = s ? static_cast<float>(-s->v[5] * 0.8) : 0.f;
        cmd.w = 1.0f;
        w_.write(cmd);
    }
private:
    DataReader<StateVec6d> r_est_{"est.500.b", DependencyType::Weak};
    DataWriter<Vec4f>      w_{"plan.200.b"};
};

/// Planner C: combines A plan + estimated state C  (200 Hz)
class Plan200_C : public ITask {
public:
    const char* getName() const override { return "Plan200_C"; }
    void execute(void*) override {
        r_pa_.on_update([this](const Vec4f& pa) {
            const StateVec6d* s = r_est_ ? &*r_est_ : nullptr;
            Vec4f cmd;
            cmd.x = pa.x + (s ? static_cast<float>(s->v[0]) : 0.f) * 0.1f;
            cmd.y = pa.y + (s ? static_cast<float>(s->v[1]) : 0.f) * 0.1f;
            cmd.z = pa.z;
            cmd.w = pa.w;
            w_.write(cmd);
        });
    }
private:
    DataReader<Vec4f>      r_pa_ {"plan.200.a", DependencyType::Strong};
    DataReader<StateVec6d> r_est_{"est.500.c",  DependencyType::Weak};
    DataWriter<Vec4f>      w_{"plan.200.c",      ArchiveOption::Enable};
};

/// Diagnostic A: monitors planner A+B latency  (200 Hz)
class Diag200_A : public ITask {
public:
    const char* getName() const override { return "Diag200_A"; }
    void execute(void*) override {
        const Vec4f* pa = r_pa_ ? &*r_pa_ : nullptr;
        const Vec4f* pb = r_pb_ ? &*r_pb_ : nullptr;
        DiagnosticData d;
        d.tick = getExecutionLocalTick();
        d.error_flags = 0;
        if (pa && std::isnan(pa->x)) d.error_flags |= 0x01;
        if (pb && std::isnan(pb->x)) d.error_flags |= 0x02;
        d.latency_us = 0.0f; // would be measured in real system
        w_.write(d);
    }
private:
    DataReader<Vec4f>    r_pa_{"plan.200.a", DependencyType::Weak};
    DataReader<Vec4f>    r_pb_{"plan.200.b", DependencyType::Weak};
    DataWriter<DiagnosticData> w_{"diag.200.a"};
};

/// Diagnostic B: monitors plan C + state C alignment  (200 Hz)
class Diag200_B : public ITask {
public:
    const char* getName() const override { return "Diag200_B"; }
    void execute(void*) override {
        const Vec4f*      pc = r_pc_ ? &*r_pc_ : nullptr;
        const StateVec6d* sc = r_sc_ ? &*r_sc_ : nullptr;
        DiagnosticData d;
        d.tick = getExecutionLocalTick();
        float err = 0.f;
        if (pc && sc)
            err = std::abs(pc->x - static_cast<float>(sc->v[0]));
        d.error_flags = (err > 0.5f) ? 0x10 : 0;
        d.latency_us  = err * 1000.f;
        w_.write(d);
    }
private:
    DataReader<Vec4f>      r_pc_{"plan.200.c", DependencyType::Weak};
    DataReader<StateVec6d> r_sc_{"est.500.c",  DependencyType::Weak};
    DataWriter<DiagnosticData> w_{"diag.200.b"};
};


// ============================================================
// ▌ 100 Hz – RT tasks (common pool)
// ▌ 5 tasks: monitors accumulating cross-domain stats
// ============================================================

/// Monitor A: ctrl command statistics  (100 Hz)
class Monitor100_A : public ITask {
public:
    const char* getName() const override { return "Monitor100_A"; }
    void execute(void*) override {
        const Vec4f*      ctrl = r_ctrl_ ? &*r_ctrl_ : nullptr;
        const StateVec6d* est  = r_est_  ? &*r_est_  : nullptr;
        float s = (ctrl ? ctrl->x : 0.f) + (est ? static_cast<float>(est->v[0]) : 0.f);
        update_stats(s);
        w_.write(stats_);
    }
private:
    void update_stats(float s) {
        stats_.count++;
        double d = s - stats_.mean;
        stats_.mean += d / stats_.count;
        stats_.variance += d * (s - stats_.mean);
        stats_.min_val = std::min(stats_.min_val, (double)s);
        stats_.max_val = std::max(stats_.max_val, (double)s);
        stats_.sample_tick = getExecutionLocalTick();
    }
    DataReader<Vec4f>      r_ctrl_{"cmd.1k.ctrl",  DependencyType::Weak};
    DataReader<StateVec6d> r_est_ {"est.500.a",     DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"mon.100.a",     ArchiveOption::Enable};
    AggregatedStats stats_{ 0, 0, std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::lowest(), 0, 0, 0 };
};

/// Monitor B: estimated state B + planner A quality  (100 Hz)
class Monitor100_B : public ITask {
public:
    const char* getName() const override { return "Monitor100_B"; }
    void execute(void*) override {
        const StateVec6d* est = r_est_ ? &*r_est_ : nullptr;
        const Vec4f*      pa  = r_pa_  ? &*r_pa_  : nullptr;
        float s = (est ? static_cast<float>(est->v[2]) : 0.f) + (pa ? pa->z : 0.f);
        update_stats(s);
        w_.write(stats_);
    }
private:
    void update_stats(float s) {
        stats_.count++;
        double d = s - stats_.mean;
        stats_.mean += d / stats_.count;
        stats_.variance += d * (s - stats_.mean);
        stats_.min_val = std::min(stats_.min_val, (double)s);
        stats_.max_val = std::max(stats_.max_val, (double)s);
        stats_.sample_tick = getExecutionLocalTick();
    }
    DataReader<StateVec6d> r_est_{"est.500.b",  DependencyType::Weak};
    DataReader<Vec4f>      r_pa_ {"plan.200.a", DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"mon.100.b", ArchiveOption::Enable};
    AggregatedStats stats_{ 0, 0, std::numeric_limits<double>::max(),
                            std::numeric_limits<double>::lowest(), 0, 0, 0 };
};

/// Monitor C: diag A + diag B error flag accumulation  (100 Hz)
class Monitor100_C : public ITask {
public:
    const char* getName() const override { return "Monitor100_C"; }
    void execute(void*) override {
        const DiagnosticData* da = r_da_ ? &*r_da_ : nullptr;
        const DiagnosticData* db = r_db_ ? &*r_db_ : nullptr;
        float s = (float)((da ? da->error_flags : 0) | (db ? db->error_flags : 0));
        update_stats(s);
        w_.write(stats_);
    }
private:
    void update_stats(float s) {
        stats_.count++;
        double d = s - stats_.mean;
        stats_.mean += d / stats_.count;
        stats_.variance += d * (s - stats_.mean);
        stats_.sample_tick = getExecutionLocalTick();
    }
    DataReader<DiagnosticData>  r_da_{"diag.200.a", DependencyType::Weak};
    DataReader<DiagnosticData>  r_db_{"diag.200.b", DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"mon.100.c",     ArchiveOption::Enable};
    AggregatedStats stats_{ 0, 0, 0, 0, 0, 0, 0 };
};

/// Monitor D: combines A and B monitors  (100 Hz, Strong read)
class Monitor100_D : public ITask {
public:
    const char* getName() const override { return "Monitor100_D"; }
    void execute(void*) override {
        r_a_.on_update([this](const AggregatedStats& a) {
            if (!r_b_) return;
            const auto& b = *r_b_;
            AggregatedStats d;
            d.mean     = 0.5 * (a.mean + b.mean);
            d.variance = a.variance + b.variance;
            d.min_val  = std::min(a.min_val, b.min_val);
            d.max_val  = std::max(a.max_val, b.max_val);
            d.count    = a.count + b.count;
            d.sample_tick = getExecutionLocalTick();
            w_.write(d);
        });
    }
private:
    DataReader<AggregatedStats> r_a_{"mon.100.a", DependencyType::Strong};
    DataReader<AggregatedStats> r_b_{"mon.100.b", DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"mon.100.d",   ArchiveOption::Enable};
};

/// Monitor E: combines C monitor + estimated state C  (100 Hz)
class Monitor100_E : public ITask {
public:
    const char* getName() const override { return "Monitor100_E"; }
    void execute(void*) override {
        const AggregatedStats* mc = r_mc_ ? &*r_mc_ : nullptr;
        const StateVec6d* sc      = r_sc_ ? &*r_sc_ : nullptr;
        AggregatedStats d;
        d.mean     = (mc ? mc->mean : 0.0) + (sc ? sc->v[5] : 0.0);
        d.variance = mc ? mc->variance : 0.0;
        d.min_val  = mc ? mc->min_val : 0.0;
        d.max_val  = mc ? mc->max_val : 0.0;
        d.count    = mc ? mc->count    : 0;
        d.sample_tick = getExecutionLocalTick();
        w_.write(d);
    }
private:
    DataReader<AggregatedStats> r_mc_{"mon.100.c", DependencyType::Weak};
    DataReader<StateVec6d>      r_sc_{"est.500.c", DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"mon.100.e",    ArchiveOption::Enable};
};


// ============================================================
// ▌ 50 Hz – RT tasks (common pool)
// ▌ 8 tasks: 4 health checks + 4 compute-stress tasks
// ============================================================

/// Health check A based on monitor A  (50 Hz)
class Health50_A : public ITask {
public:
    const char* getName() const override { return "Health50_A"; }
    void execute(void*) override {
        const AggregatedStats* m = r_ ? &*r_ : nullptr;
        DiagnosticData d;
        d.tick = getExecutionLocalTick();
        if (m) {
            d.error_flags = (std::abs(m->mean) > 1.0) ? 0x01 : 0x00;
            d.latency_us  = static_cast<float>(m->variance);
        } else { d.error_flags = 0xFF; d.latency_us = -1.f; }
        w_.write(d);
    }
private:
    DataReader<AggregatedStats> r_{"mon.100.a", DependencyType::Weak};
    DataWriter<DiagnosticData>  w_{"health.50.a"};
};

/// Health check B based on monitor B  (50 Hz)
class Health50_B : public ITask {
public:
    const char* getName() const override { return "Health50_B"; }
    void execute(void*) override {
        const AggregatedStats* m = r_ ? &*r_ : nullptr;
        DiagnosticData d;
        d.tick = getExecutionLocalTick();
        d.error_flags = (m && m->variance > 0.5) ? 0x02 : 0x00;
        d.latency_us  = m ? static_cast<float>(m->mean) : -1.f;
        w_.write(d);
    }
private:
    DataReader<AggregatedStats> r_{"mon.100.b", DependencyType::Weak};
    DataWriter<DiagnosticData>  w_{"health.50.b"};
};

/// Health check C based on monitor C  (50 Hz)
class Health50_C : public ITask {
public:
    const char* getName() const override { return "Health50_C"; }
    void execute(void*) override {
        const AggregatedStats* m = r_ ? &*r_ : nullptr;
        DiagnosticData d;
        d.tick = getExecutionLocalTick();
        d.error_flags = (m && m->mean > 0.1) ? 0x04 : 0x00;
        d.latency_us  = 0.f;
        w_.write(d);
    }
private:
    DataReader<AggregatedStats> r_{"mon.100.c", DependencyType::Weak};
    DataWriter<DiagnosticData>  w_{"health.50.c"};
};

/// Health check D: aggregated health from D+E monitors  (50 Hz)
class Health50_D : public ITask {
public:
    const char* getName() const override { return "Health50_D"; }
    void execute(void*) override {
        r_d_.on_update([this](const AggregatedStats& md) {
            const AggregatedStats* me = r_e_ ? &*r_e_ : nullptr;
            DiagnosticData d;
            d.tick = getExecutionLocalTick();
            d.error_flags = (md.count == 0) ? 0x08 : 0x00;
            if (me) d.error_flags |= (me->count == 0) ? 0x10 : 0x00;
            d.latency_us = static_cast<float>(md.mean + (me ? me->mean : 0.0));
            w_.write(d);
        });
    }
private:
    DataReader<AggregatedStats> r_d_{"mon.100.d", DependencyType::Strong};
    DataReader<AggregatedStats> r_e_{"mon.100.e", DependencyType::Weak};
    DataWriter<DiagnosticData>  w_{"health.50.d", ArchiveOption::Enable};
};

/// Stress test A: compute-heavy cross-multiply on 1kHz filtered data  (50 Hz)
class Stress50_A : public ITask {
public:
    const char* getName() const override { return "Stress50_A"; }
    void execute(void*) override {
        const SampleData8f* a = r_a_ ? &*r_a_ : nullptr;
        const SampleData8f* b = r_b_ ? &*r_b_ : nullptr;
        SampleData8f d;
        // 8×8 outer-product accumulation (64 multiply-adds per tick)
        for (int i = 0; i < 8; ++i) {
            float acc = 0.f;
            for (int j = 0; j < 8; ++j)
                acc += (a ? a->v[j] : 0.f) * (b ? b->v[(i + j) % 8] : 0.f);
            d.v[i] = acc;
        }
        w_.write(d);
    }
private:
    DataReader<SampleData8f> r_a_{"filt.1k.a", DependencyType::Weak};
    DataReader<SampleData8f> r_b_{"filt.1k.b", DependencyType::Weak};
    DataWriter<SampleData8f> w_{"stress.50.a"};
};

/// Stress test B: compute-heavy cross-multiply on filtered C + state  (50 Hz)
class Stress50_B : public ITask {
public:
    const char* getName() const override { return "Stress50_B"; }
    void execute(void*) override {
        const SampleData8f* c  = r_c_  ? &*r_c_  : nullptr;
        const StateVec6d*   s  = r_est_ ? &*r_est_ : nullptr;
        SampleData8f d;
        for (int i = 0; i < 8; ++i) {
            float acc = c ? c->v[i] : 0.f;
            acc *= s ? static_cast<float>(s->v[i % 6]) : 1.f;
            d.v[i] = std::tanh(acc); // nonlinear op
        }
        w_.write(d);
    }
private:
    DataReader<SampleData8f> r_c_  {"filt.1k.c",  DependencyType::Weak};
    DataReader<StateVec6d>   r_est_{"est.500.a",   DependencyType::Weak};
    DataWriter<SampleData8f> w_{"stress.50.b"};
};

/// Stress test C: combines three plan outputs  (50 Hz)
class Stress50_C : public ITask {
public:
    const char* getName() const override { return "Stress50_C"; }
    void execute(void*) override {
        const Vec4f* pa = r_pa_ ? &*r_pa_ : nullptr;
        const Vec4f* pb = r_pb_ ? &*r_pb_ : nullptr;
        const Vec4f* pc = r_pc_ ? &*r_pc_ : nullptr;
        SampleData8f d;
        for (int i = 0; i < 8; ++i) {
            float v = (pa ? pa->x : 0.f) * std::sin(i * 0.785f)
                    + (pb ? pb->y : 0.f) * std::cos(i * 0.785f)
                    + (pc ? pc->z : 0.f);
            d.v[i] = v;
        }
        w_.write(d);
    }
private:
    DataReader<Vec4f>    r_pa_{"plan.200.a", DependencyType::Weak};
    DataReader<Vec4f>    r_pb_{"plan.200.b", DependencyType::Weak};
    DataReader<Vec4f>    r_pc_{"plan.200.c", DependencyType::Weak};
    DataWriter<SampleData8f> w_{"stress.50.c"};
};

/// Stress test D: final stress aggregator (Strong chain end)  (50 Hz)
class Stress50_D : public ITask {
public:
    const char* getName() const override { return "Stress50_D"; }
    void execute(void*) override {
        r_a_.on_update([this](const SampleData8f& a) {
            const SampleData8f* b = r_b_ ? &*r_b_ : nullptr;
            SampleData8f d;
            for (int i = 0; i < 8; ++i) {
                float bv = b ? b->v[i] : 0.f;
                d.v[i] = a.v[i] + bv;
            }
            w_.write(d);
        });
    }
private:
    DataReader<SampleData8f> r_a_{"stress.50.a", DependencyType::Strong};
    DataReader<SampleData8f> r_b_{"stress.50.b", DependencyType::Weak};
    DataWriter<SampleData8f> w_{"stress.50.d",   ArchiveOption::Enable};
};


// ============================================================
// ▌ 10 Hz – RT stateful tasks (common pool, Task<StateT>)
// ▌ 5 tasks: rolling statistics with checkpoint-able state
// ============================================================

/// Stateful rolling stats on health.50.a  (10 Hz)
class Stateful10_A : public Task<RollingAccState> {
public:
    const char* getName() const override { return "Stateful10_A"; }
    void execute(RollingAccState& s) override {
        if (!s.window) { s.window = 100; s.min_val = 1e9; s.max_val = -1e9; }
        const DiagnosticData* h = r_ ? &*r_ : nullptr;
        double v = h ? (double)h->error_flags : 0.0;
        s.sum    += v;  s.sum_sq += v * v;
        s.min_val = std::min(s.min_val, v);
        s.max_val = std::max(s.max_val, v);
        s.count++;
        AggregatedStats out;
        out.count       = (uint32_t)std::min(s.count, (uint64_t)s.window);
        out.mean        = s.sum  / s.count;
        out.variance    = s.sum_sq / s.count - out.mean * out.mean;
        out.min_val     = s.min_val;
        out.max_val     = s.max_val;
        out.sample_tick = getExecutionLocalTick();
        w_.write(out);
    }
private:
    DataReader<DiagnosticData>  r_{"health.50.a", DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"stat.10.a",   ArchiveOption::Enable};
};

/// Stateful rolling stats on health.50.b  (10 Hz)
class Stateful10_B : public Task<RollingAccState> {
public:
    const char* getName() const override { return "Stateful10_B"; }
    void execute(RollingAccState& s) override {
        if (!s.window) { s.window = 100; s.min_val = 1e9; s.max_val = -1e9; }
        const DiagnosticData* h = r_ ? &*r_ : nullptr;
        double v = h ? (double)h->latency_us : 0.0;
        s.sum += v;  s.sum_sq += v * v;
        s.min_val = std::min(s.min_val, v);
        s.max_val = std::max(s.max_val, v);
        s.count++;
        AggregatedStats out;
        out.count       = (uint32_t)s.count;
        out.mean        = s.sum  / s.count;
        out.variance    = s.sum_sq / s.count - out.mean * out.mean;
        out.min_val     = s.min_val;
        out.max_val     = s.max_val;
        out.sample_tick = getExecutionLocalTick();
        w_.write(out);
    }
private:
    DataReader<DiagnosticData>  r_{"health.50.b", DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"stat.10.b",   ArchiveOption::Enable};
};

/// Stateful rolling stats on health.50.c + health.50.d  (10 Hz)
class Stateful10_C : public Task<RollingAccState> {
public:
    const char* getName() const override { return "Stateful10_C"; }
    void execute(RollingAccState& s) override {
        if (!s.window) { s.window = 100; s.min_val = 1e9; s.max_val = -1e9; }
        const DiagnosticData* hc = r_c_ ? &*r_c_ : nullptr;
        const DiagnosticData* hd = r_d_ ? &*r_d_ : nullptr;
        double v = (hc ? (double)hc->error_flags : 0.0)
                 + (hd ? (double)hd->error_flags : 0.0);
        s.sum += v;  s.sum_sq += v * v;  s.count++;
        s.min_val = std::min(s.min_val, v);
        s.max_val = std::max(s.max_val, v);
        AggregatedStats out;
        out.count       = (uint32_t)s.count;
        out.mean        = s.sum  / s.count;
        out.variance    = s.sum_sq / s.count - out.mean * out.mean;
        out.min_val     = s.min_val;
        out.max_val     = s.max_val;
        out.sample_tick = getExecutionLocalTick();
        w_.write(out);
    }
private:
    DataReader<DiagnosticData>  r_c_{"health.50.c", DependencyType::Weak};
    DataReader<DiagnosticData>  r_d_{"health.50.d", DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"stat.10.c",      ArchiveOption::Enable};
};

/// Stateful rolling stats on monitor D+E  (10 Hz)
class Stateful10_D : public Task<RollingAccState> {
public:
    const char* getName() const override { return "Stateful10_D"; }
    void execute(RollingAccState& s) override {
        if (!s.window) { s.window = 200; s.min_val = 1e9; s.max_val = -1e9; }
        const AggregatedStats* md = r_d_ ? &*r_d_ : nullptr;
        const AggregatedStats* me = r_e_ ? &*r_e_ : nullptr;
        double v = (md ? md->mean : 0.0) + (me ? me->mean : 0.0);
        s.sum += v;  s.sum_sq += v * v;  s.count++;
        s.min_val = std::min(s.min_val, v);
        s.max_val = std::max(s.max_val, v);
        AggregatedStats out;
        out.count       = (uint32_t)s.count;
        out.mean        = s.sum  / s.count;
        out.variance    = s.sum_sq / s.count - out.mean * out.mean;
        out.min_val     = s.min_val;
        out.max_val     = s.max_val;
        out.sample_tick = getExecutionLocalTick();
        w_.write(out);
    }
private:
    DataReader<AggregatedStats> r_d_{"mon.100.d", DependencyType::Weak};
    DataReader<AggregatedStats> r_e_{"mon.100.e", DependencyType::Weak};
    DataWriter<AggregatedStats> w_{"stat.10.d",   ArchiveOption::Enable};
};

/// Summary task: aggregates all stat.10.* outputs  (10 Hz, archive-enabled)
struct SummaryState {
    uint64_t total_ticks;
    uint32_t cycle_count;
    uint32_t _pad;
};

class Stateful10_Summary : public Task<SummaryState> {
public:
    const char* getName() const override { return "Stateful10_Summary"; }
    void execute(SummaryState& s) override {
        s.total_ticks = getExecutionLocalTick();
        s.cycle_count++;

        const AggregatedStats* sa = r_a_ ? &*r_a_ : nullptr;
        const AggregatedStats* sb = r_b_ ? &*r_b_ : nullptr;
        const AggregatedStats* sc = r_c_ ? &*r_c_ : nullptr;
        const AggregatedStats* sd = r_d_ ? &*r_d_ : nullptr;

        SummaryOutput out;
        out.tick = s.total_ticks;
        out.active_tasks = 44;
        out.overrun_flags = 0;
        for (int i = 0; i < 4; ++i) out.stats[i] = {};
        if (sa) out.stats[0] = *sa;
        if (sb) out.stats[1] = *sb;
        if (sc) out.stats[2] = *sc;
        if (sd) out.stats[3] = *sd;

        w_.write(out);

        if (s.cycle_count % 10 == 0) {
            getLogger()->info("[Summary] tick={} mean_A={:.4f} mean_B={:.4f}",
                              s.total_ticks,
                              sa ? sa->mean : 0.0,
                              sb ? sb->mean : 0.0);
        }
    }
private:
    DataReader<AggregatedStats> r_a_{"stat.10.a", DependencyType::Weak};
    DataReader<AggregatedStats> r_b_{"stat.10.b", DependencyType::Weak};
    DataReader<AggregatedStats> r_c_{"stat.10.c", DependencyType::Weak};
    DataReader<AggregatedStats> r_d_{"stat.10.d", DependencyType::Weak};
    DataWriter<SummaryOutput>   w_{"stat.10.summary", ArchiveOption::Enable};
};


// ============================================================
// ▌ Derived-value verification tasks
// ▌ These four tasks produce NON-archived outputs for testing
// ▌ replay reproducibility.  They come in two pairs:
// ▌
// ▌  RT-origin pair  (10 Hz, common RT pool)
// ▌    DeriveRT_Pure       – pure f(archived RT data), NO tick
// ▌                          → MUST match live vs replay
// ▌    DeriveRT_TickTainted – same inputs + getExecutionLocalTick()
// ▌                          → MUST diverge  (tick differs in replay)
// ▌
// ▌  NonRT-origin pair (1 Hz, non-RT pool)
// ▌    DeriveNonRT_Pure       – pure f(archived NonRT data), NO tick
// ▌                            → MUST match live vs replay
// ▌    DeriveNonRT_TickTainted – same inputs + getExecutionLocalTick()
// ▌                            → MUST diverge
// ▌
// ▌  DerivedCheck_Logger (5 Hz, non-RT) – logs all four values
// ============================================================

/// Non-archived pure function of archived RT data  (10 Hz RT)
/// Input: stat.10.a (AggregatedStats, 10 Hz, archived) via Strong/on_update.
/// Using the same-frequency archived key eliminates cross-frequency weak-read
/// non-determinism: the framework guarantees DeriveRT_Pure executes AFTER
/// Stateful10_A commits (dependency ordering), so both live and replay read
/// the same injected value.
/// Output: mean * 2 + min_val - max_val  ← pure, no tick.
class DeriveRT_Pure : public ITask {
public:
    const char* getName() const override { return "DeriveRT_Pure"; }
    void execute(void*) override {
        r_stat_.on_update([this](const AggregatedStats& s) {
            ScalarVal d;
            d.v = s.mean * 2.0 + s.min_val - s.max_val;
            w_.write(d);
        });
    }
private:
    DataReader<AggregatedStats> r_stat_{"stat.10.a", DependencyType::Strong};
    DataWriter<ScalarVal>       w_{"derived.rt.pure"};
};

/// Non-archived tick-tainted function of archived RT data  (10 Hz RT)
/// Same input as DeriveRT_Pure, but adds getExecutionLocalTick().
/// live tick ≈ 8000+, replay tick ≈ 3000+ → factor diff ≈ 50 → clearly diverges.
class DeriveRT_TickTainted : public ITask {
public:
    const char* getName() const override { return "DeriveRT_TickTainted"; }
    void execute(void*) override {
        r_stat_.on_update([this](const AggregatedStats& s) {
            ScalarVal d;
            d.v = s.mean * (1.0 + getExecutionLocalTick() * 0.01);
            w_.write(d);
        });
    }
private:
    DataReader<AggregatedStats> r_stat_{"stat.10.a", DependencyType::Strong};
    DataWriter<ScalarVal>       w_{"derived.rt.tick"};
};

/// Non-archived pure function of archived NonRT data  (1 Hz NonRT)
/// Input: param.gain.a (archived NonRT key).
/// Output: gain² + offset  ← pure function, no tick.
/// During replay: param.gain.a is injected from file → output must reproduce.
class DeriveNonRT_Pure : public ITask {
public:
    const char* getName() const override { return "DeriveNonRT_Pure"; }
    void execute(void*) override {
        const ParamData* p = r_ ? &*r_ : nullptr;
        ScalarVal d;
        d.v = p ? (p->gain * p->gain + p->offset) : 0.0;
        w_.write(d);
    }
private:
    DataReader<ParamData> r_{"param.gain.a", DependencyType::Weak};
    DataWriter<ScalarVal> w_{"derived.nonrt.pure"};
};

/// Non-archived tick-tainted function of archived NonRT data  (1 Hz NonRT)
/// Same input as DeriveNonRT_Pure, but scales by getExecutionLocalTick().
/// NonRT and RT tick counters are independent; both will differ in replay.
/// The test verifies detected divergence (expected behavior).
class DeriveNonRT_TickTainted : public ITask {
public:
    const char* getName() const override { return "DeriveNonRT_TickTainted"; }
    void execute(void*) override {
        const ParamData* p = r_ ? &*r_ : nullptr;
        // Multiply gain by a tick-derived factor.
        // NonRT tick after 8s warm-up ≈ 8 (1 Hz × 8 s), replay ≈ 3.
        // Factor diff ≈ 0.5 >> tolerance, so divergence is clearly visible.
        ScalarVal d;
        d.v = p ? p->gain * (1.0 + getExecutionLocalTick() * 0.1) : 0.0;
        w_.write(d);
    }
private:
    DataReader<ParamData> r_{"param.gain.a", DependencyType::Weak};
    DataWriter<ScalarVal> w_{"derived.nonrt.tick"};
};

/// Logger for the four derived verification channels  (5 Hz NonRT)
/// Format: [DerivedCheck] rt_pure=<v> rt_tick=<v> nonrt_pure=<v> nonrt_tick=<v>
class DerivedCheck_Logger : public ITask {
public:
    const char* getName() const override { return "DerivedCheck_Logger"; }
    void execute(void*) override {
        const ScalarVal* rp  = r_rt_pure_  ? &*r_rt_pure_  : nullptr;
        const ScalarVal* rt  = r_rt_tick_  ? &*r_rt_tick_  : nullptr;
        const ScalarVal* nrp = r_nrt_pure_ ? &*r_nrt_pure_ : nullptr;
        const ScalarVal* nrt = r_nrt_tick_ ? &*r_nrt_tick_ : nullptr;
        getLogger()->info("[DerivedCheck] "
                          "g_tick={} "
                          "rt_pure={:.8f} rt_tick={:.8f} "
                          "nonrt_pure={:.8f} nonrt_tick={:.8f}",
                          getCurrentTick(),
                          rp  ? rp->v  : 0.0,
                          rt  ? rt->v  : 0.0,
                          nrp ? nrp->v : 0.0,
                          nrt ? nrt->v : 0.0);
    }
private:
    DataReader<ScalarVal> r_rt_pure_ {"derived.rt.pure",   DependencyType::Weak};
    DataReader<ScalarVal> r_rt_tick_ {"derived.rt.tick",   DependencyType::Weak};
    DataReader<ScalarVal> r_nrt_pure_{"derived.nonrt.pure", DependencyType::Weak};
    DataReader<ScalarVal> r_nrt_tick_{"derived.nonrt.tick", DependencyType::Weak};
};


// ============================================================
// ▌ Non-RT – parameter generators (1 Hz)
// ▌ 3 tasks: periodic gain / offset sweep for replay testing
// ============================================================

/// Sweeps gain.a over time  (Non-RT 1 Hz)
class ParamGen_A : public ITask {
public:
    const char* getName() const override { return "ParamGen_A"; }
    void execute(void*) override {
        ParamData d;
        double t = getExecutionLocalTick() / 1.0;
        d.gain   = 1.0 + 0.5 * std::sin(t * 0.1);
        d.offset = 0.0;
        d.scale  = 1.0;
        w_.write(d);
        getLogger()->debug("[ParamGen_A] gain={:.4f}", d.gain);
    }
private:
    DataWriter<ParamData> w_{"param.gain.a", ArchiveOption::Enable};
};

/// Sweeps gain.b (different phase)  (Non-RT 1 Hz)
class ParamGen_B : public ITask {
public:
    const char* getName() const override { return "ParamGen_B"; }
    void execute(void*) override {
        ParamData d;
        double t = getExecutionLocalTick() / 1.0;
        d.gain   = 0.8 + 0.2 * std::cos(t * 0.07);
        d.offset = 0.1 * std::sin(t * 0.05);
        d.scale  = 1.0 + 0.1 * std::cos(t * 0.03);
        w_.write(d);
    }
private:
    DataWriter<ParamData> w_{"param.gain.b", ArchiveOption::Enable};
};

/// Sweeps gain.c (step function profile)  (Non-RT 1 Hz)
class ParamGen_C : public ITask {
public:
    const char* getName() const override { return "ParamGen_C"; }
    void execute(void*) override {
        ParamData d;
        int slot = (getExecutionLocalTick() / 5) % 4;
        const double table[4] = {0.5, 1.0, 1.5, 2.0};
        d.gain   = table[slot];
        d.offset = 0.0;
        d.scale  = 1.0;
        w_.write(d);
    }
private:
    DataWriter<ParamData> w_{"param.gain.c", ArchiveOption::Enable};
};


// ============================================================
// ▌ Non-RT – diagnostic loggers (5 Hz)
// ▌ 4 tasks: summarize and log cross-domain outputs
// ============================================================

/// Logs summary output every 200 ms  (Non-RT 5 Hz)
class StatsLog_A : public ITask {
public:
    const char* getName() const override { return "StatsLog_A"; }
    void execute(void*) override {
        r_.on_update([this](const SummaryOutput& out) {
            getLogger()->info("[StatsLog_A] tick={} active={} "
                              "mean_A={:.4f} mean_B={:.4f}",
                              out.tick, out.active_tasks,
                              out.stats[0].mean, out.stats[1].mean);
            DiagnosticData d;
            d.tick        = out.tick;
            d.error_flags = out.overrun_flags;
            d.latency_us  = 0.f;
            w_.write(d);
        });
    }
private:
    DataReader<SummaryOutput>  r_{"stat.10.summary",  DependencyType::Weak};
    DataWriter<DiagnosticData> w_{"log.5hz.summary"};
};

/// Logs diag.200 status  (Non-RT 5 Hz)
class DiagLog_B : public ITask {
public:
    const char* getName() const override { return "DiagLog_B"; }
    void execute(void*) override {
        const DiagnosticData* da = r_a_ ? &*r_a_ : nullptr;
        const DiagnosticData* db = r_b_ ? &*r_b_ : nullptr;
        DiagnosticData d;
        d.tick        = getExecutionLocalTick();
        d.error_flags = (da ? da->error_flags : 0) | (db ? db->error_flags : 0);
        d.latency_us  = (da ? da->latency_us : 0.f) + (db ? db->latency_us : 0.f);
        if (d.error_flags) {
            getLogger()->warn("[DiagLog_B] error_flags=0x{:02X} lat={:.2f}us",
                              d.error_flags, d.latency_us);
        }
        w_.write(d);
    }
private:
    DataReader<DiagnosticData> r_a_{"diag.200.a", DependencyType::Weak};
    DataReader<DiagnosticData> r_b_{"diag.200.b", DependencyType::Weak};
    DataWriter<DiagnosticData> w_{"log.5hz.diag"};
};

/// Logs health.50.d status  (Non-RT 5 Hz)
class HealthLog_C : public ITask {
public:
    const char* getName() const override { return "HealthLog_C"; }
    void execute(void*) override {
        const DiagnosticData* h = r_ ? &*r_ : nullptr;
        DiagnosticData d;
        d.tick        = getExecutionLocalTick();
        d.error_flags = h ? h->error_flags : 0xFF;
        d.latency_us  = h ? h->latency_us  : -1.f;
        getLogger()->debug("[HealthLog_C] flags=0x{:02X}", d.error_flags);
        w_.write(d);
    }
private:
    DataReader<DiagnosticData> r_{"health.50.d", DependencyType::Weak};
    DataWriter<DiagnosticData> w_{"log.5hz.health"};
};

/// Logs stress.50.d (perf benchmark output)  (Non-RT 5 Hz)
class PerfLog_D : public ITask {
public:
    const char* getName() const override { return "PerfLog_D"; }
    void execute(void*) override {
        const SampleData8f* p = r_ ? &*r_ : nullptr;
        DiagnosticData d;
        d.tick = getExecutionLocalTick();
        d.error_flags = 0;
        float rms = 0.f;
        if (p) {
            for (int i = 0; i < 8; ++i) rms += p->v[i] * p->v[i];
            rms = std::sqrt(rms / 8.f);
        }
        d.latency_us = rms * 1000.f; // reused as RMS field
        getLogger()->debug("[PerfLog_D] stress.50.d rms={:.4f}", rms);
        w_.write(d);
    }
private:
    DataReader<SampleData8f>   r_{"stress.50.d", DependencyType::Weak};
    DataWriter<DiagnosticData> w_{"log.5hz.perf"};
};
