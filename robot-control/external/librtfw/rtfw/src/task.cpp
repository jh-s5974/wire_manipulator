#include "rtfw/task.h"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace rtfw::rt;

    thread_local std::vector<ITask*> ITask::_task_stack;

    // ITask 기본 생성자
    ITask::ITask() : _id(0), _affinity(-1), _current_tick_in_cycle(0), _execution_local_tick(0) {
        taskStack().push_back(this);
    }

    ITask::~ITask() {
        taskStack().pop_back();
    }
    
        std::vector<ITask*>& ITask::taskStack() {
            return _task_stack;
        }

    ITask* ITask::currentConstructingTask() {
        auto& s = taskStack();
        return s.empty() ? nullptr : s.back();
    }
    void ITask::_autoRegisterReadRequest(ReadRequest&& req,
                                        internal::IDataReader* reader) {
        _read_requests.push_back(std::move(req));
        _data_readers.insert({req.key_hash, reader});
        _held_proxies.push_back(reader);
    }

    void ITask::_autoRegisterWriteRequest(WriteRequest&& req,
                                        internal::IDataWriter* writer) {
        _write_requests.push_back(std::move(req));
        _data_writers.push_back(writer);
    }

    void ITask::_autoRegisterParamRequest(ParamReadRequest&& req,
                                        internal::IParameter* param) {
        _param_read_requests.push_back(std::move(req));
        _parameter_readers.push_back(param);
    }

    // 통계 계산을 위한 헬퍼 함수들
    static long long get_avg(const std::atomic<long long>& total, const std::atomic<long long>& count) {
        long long c = count.load(std::memory_order_relaxed);
        return (c > 0) ? total.load(std::memory_order_relaxed) / c : 0;
    }

    static long long get_std_dev(const std::atomic<long long>& total_sq, const std::atomic<long long>& total, const std::atomic<long long>& count) {
        long long c = count.load(std::memory_order_relaxed);
        if (c < 2) return 0;
        long long total_ns = total.load(std::memory_order_relaxed);
        long long total_sq_ns = total_sq.load(std::memory_order_relaxed);
        double mean = static_cast<double>(total_ns) / c;
        double variance = static_cast<double>(total_sq_ns) / c - (mean * mean);
        return (variance > 0) ? static_cast<long long>(std::sqrt(variance)) : 0;
    }

    void ITask::printStats(const common::TaskStats& stats) const {
        long long count = stats.exec_count.load();
        double avg_exec_us = 0.0, max_exec_us = 0.0;
        double avg_lat_us = 0.0, max_lat_us = 0.0;

        if (count > 0) {
            avg_exec_us = (stats.total_exec_time_ns.load() / (double)count) / 1000.0;
            max_exec_us = stats.max_exec_time_ns.load() / 1000.0;
            avg_lat_us = (stats.total_latency_ns.load() / (double)count) / 1000.0;
            max_lat_us = stats.max_latency_ns.load() / 1000.0;
        }

        std::string type_str = isNonRt() ? "NRT" : std::to_string(getAffinity());
        std::string name_str = std::string(getName()) + " (" + std::to_string(getID()) + ", " + type_str + ")";
        
        std::cout << "    " << std::left << std::setw(30) << name_str
                << "| " << std::fixed << std::setprecision(2) 
                << std::right << std::setw(8) << avg_exec_us << " / " << std::setw(8) << max_exec_us
                << "| " << std::right << std::setw(8) << avg_lat_us << " / " << std::setw(8) << max_lat_us
                << "| " << std::right << std::setw(8) << count << std::endl;
    }
