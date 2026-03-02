#include "task_commands.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <set>
#include <map>
#include <rtfw_connect/shm_querier.h>
#include <rtfw_connect/shm_controller.h>

namespace rtcli {

int TaskCommandHandler::execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) {
    if (!connector.isAttached()) {
        std::cerr << "Error: Failed to connect to shared memory" << std::endl;
        return 2;
    }

    if (args.empty()) {
        std::cout << help();
        return 1;
    }

    const std::string& subcmd = args[0];

    if (subcmd == "enable") {
        // task enable <task_id>
        if (args.size() < 2) {
            std::cerr << "Usage: task enable <task_id>" << std::endl;
            return 1;
        }

        uint32_t task_id;
        try {
            task_id = std::stoul(args[1]);
        } catch (const std::exception& e) {
            std::cerr << "ERROR: Invalid task_id - " << e.what() << std::endl;
            return 1;
        }

        return setTaskEnabled(connector, task_id, true);

    } else if (subcmd == "disable") {
        // task disable <task_id>
        if (args.size() < 2) {
            std::cerr << "Usage: task disable <task_id>" << std::endl;
            return 1;
        }

        uint32_t task_id;
        try {
            task_id = std::stoul(args[1]);
        } catch (const std::exception& e) {
            std::cerr << "ERROR: Invalid task_id - " << e.what() << std::endl;
            return 1;
        }
        return setTaskEnabled(connector, task_id, false);

    } else if (subcmd == "list") {
        // task list - Show all tasks
        return showTaskList(connector);

    } else if (subcmd == "info") {
        // task info <task_id> [--v | -v] - Show detailed info for specific task
        if (args.size() < 2) {
            std::cerr << "Usage: task info <task_id> [--v | -v]" << std::endl;
            return 1;
        }

        uint32_t task_id;
        try {
            task_id = std::stoul(args[1]);
        } catch (const std::exception& e) {
            std::cerr << "ERROR: Invalid task_id - " << e.what() << std::endl;
            return 1;
        }

        bool verbose = false;
        if (args.size() > 2) {
            const std::string& opt = args[2];
            if (opt == "--v" || opt == "-v") {
                verbose = true;
            }
        }

        return showTaskInfo(connector, task_id, verbose);

    } else {
        std::cerr << "Unknown subcommand: " << subcmd << std::endl;
        std::cout << help();
        return 1;
    }
}

int TaskCommandHandler::setTaskEnabled(rtfw::connect::SharedMemoryConnector& connector, uint32_t task_id, bool enabled) {
    rtfw::connect::SharedMemoryQuerier querier(connector.getBasePtr());
    if (!querier.isConnected()) {
        std::cerr << "ERROR: Cannot access shared memory" << std::endl;
        return 2;
    }

    size_t task_count = querier.getTaskStatsCount();
    if (task_id >= task_count) {
        std::cerr << "ERROR: Task ID " << task_id << " out of range (max: " 
                  << (task_count - 1) << ")" << std::endl;
        return 1;
    }

    // Use controller API to set task enabled/disabled
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    if (!controller.setTaskEnabled(task_id, enabled)) {
        std::cerr << "ERROR: Failed to set task enabled state" << std::endl;
        return 2;
    }

    std::cout << "Setting task " << task_id << " to " 
              << (enabled ? "ENABLED" : "DISABLED") << "..." << std::endl;

    // Wait for action to complete (with timeout)
    bool completed = controller.waitForActionComplete(1000);  // 1 second timeout
    if (!completed) {
        std::cerr << "WARNING: Action did not complete within timeout" << std::endl;
    }

    std::cout << "Done." << std::endl;
    return 0;
}

int TaskCommandHandler::showTaskList(rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryQuerier querier(connector.getBasePtr());
    if (!querier.isConnected()) {
        std::cerr << "ERROR: Cannot access shared memory" << std::endl;
        return 2;
    }

    const rtfw::common::TaskStats* stats_array = querier.getTaskStatsArray();
    size_t task_count = querier.getTaskStatsCount();

    std::cout << "\n[Registered Tasks]" << std::endl;
    std::cout << std::left << std::setw(4) << "ID" 
              << std::setw(32) << "Name" << std::endl;
    std::cout << std::string(36, '-') << std::endl;

    for (size_t i = 0; i < task_count; ++i) {
        std::cout << std::setw(4) << i 
                  << std::setw(32) << stats_array[i].task_name << std::endl;
    }

    std::cout << std::endl;
    return 0;
}

int TaskCommandHandler::showTaskInfo(rtfw::connect::SharedMemoryConnector& connector, uint32_t task_id, bool verbose) {
    rtfw::connect::SharedMemoryQuerier querier(connector.getBasePtr());
    if (!querier.isConnected()) {
        std::cerr << "ERROR: Cannot access shared memory" << std::endl;
        return 2;
    }

    const rtfw::common::TaskStats* stats_array = querier.getTaskStatsArray();
    size_t task_count = querier.getTaskStatsCount();

    if (task_id >= task_count) {
        std::cerr << "ERROR: Task ID " << task_id << " out of range (max: " 
                  << (task_count - 1) << ")" << std::endl;
        return 1;
    }

    // Get graph nodes for scheduling info
    std::vector<rtfw::common::TaskGraphNodeInfo> graph_nodes = querier.getGraphNodes();
    if (task_id >= graph_nodes.size()) {
        std::cerr << "ERROR: Graph node info not found" << std::endl;
        return 2;
    }

    // Get graph edges and data flows for Writer/Reader/Parameter info
    std::vector<rtfw::common::GraphEdge> graph_edges = querier.getGraphEdges();
    std::vector<rtfw::common::DataFlowInfo> data_flows = querier.getDataFlows();
    std::vector<rtfw::common::ParameterInfo> all_params = querier.getAllParameterInfos();

    const rtfw::common::TaskStats& stats = stats_array[task_id];
    const rtfw::common::TaskGraphNodeInfo& graph_node = graph_nodes[task_id];
    // Collect Writers, Readers, and Parameters for this task
    // Use set to keep only unique keys
    std::set<std::string> writer_keys;
    std::set<std::string> reader_keys;
    std::set<std::string> param_keys;

    // Get Writers and Readers from DataFlowInfo
    for (const auto& flow : data_flows) {
        if (flow.writer_task_id == task_id) {
            writer_keys.insert(flow.key);
        } else if (flow.reader_task_id == task_id) {
            reader_keys.insert(flow.key);
        }
    }

    // Get Parameters (parameters are global, but we'll show all for reference)
    for (const auto& param : all_params) {
        param_keys.insert(param.key);
    }

    std::cout << "\n[Task Information]" << std::endl;
    std::cout << "Name:           " << stats.task_name << std::endl;
    std::cout << "Task ID:        " << task_id << std::endl;
    std::cout << "\n[Scheduling]" << std::endl;
    std::cout << "Frequency:      " << graph_node.frequency << " Hz" << std::endl;
    std::cout << "Affinity:       " << (graph_node.affinity >= 0 ? std::to_string(graph_node.affinity) : "any") << std::endl;
    std::cout << "Type:           " << (graph_node.is_non_rt ? "Non-RT" : "RT") << std::endl;

    // Data I/O Summary
    std::cout << "\n[Data I/O]" << std::endl;
    std::cout << "Writers:        " << writer_keys.size() << std::endl;
    std::cout << "Readers:        " << reader_keys.size() << std::endl;
    std::cout << "Parameters:     " << all_params.size() << std::endl;

    // Verbose output
    if (verbose) {
        if (!writer_keys.empty()) {
            std::cout << "\n  [Writers]" << std::endl;
            // Group writers by key and show which tasks read them
            std::map<std::string, std::vector<std::pair<uint32_t, bool>>> key_to_readers; // key -> (reader_task_id, is_weak)
            
            for (const auto& flow : data_flows) {
                if (flow.writer_task_id == task_id) {
                    // Find the edge to determine if it's weak or strong
                    bool is_weak = false;
                    for (const auto& edge : graph_edges) {
                        if (edge.writer_task_id == task_id && edge.reader_task_id == flow.reader_task_id) {
                            is_weak = edge.is_weak;
                            break;
                        }
                    }
                    // Only add if not already in list (avoid duplicates)
                    auto& readers_list = key_to_readers[flow.key];
                    bool found = false;
                    for (const auto& existing : readers_list) {
                        if (existing.first == flow.reader_task_id) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        readers_list.push_back({flow.reader_task_id, is_weak});
                    }
                }
            }
            
            for (const auto& w : writer_keys) {
                std::cout << "    - " << w << std::endl;
                if (key_to_readers.find(w) != key_to_readers.end()) {
                    const auto& readers_list = key_to_readers[w];
                    for (size_t i = 0; i < readers_list.size(); ++i) {
                        uint32_t reader_id = readers_list[i].first;
                        bool is_weak = readers_list[i].second;
                        const auto& reader_name = graph_nodes[reader_id].task_name;
                        
                        bool is_last = (i == readers_list.size() - 1);
                        std::string prefix = is_last ? "      └── " : "      ├── ";
                        std::string edge_type = is_weak ? "(Weak)" : "(Strong)";
                        
                        std::cout << prefix << "[" << reader_id << "] " << reader_name 
                                  << " " << edge_type << std::endl;
                    }
                }
            }
        }

        if (!reader_keys.empty()) {
            std::cout << "\n  [Readers]" << std::endl;
            // For each reader key, show which task writes it
            std::map<std::string, std::pair<uint32_t, bool>> key_to_writer; // key -> (writer_task_id, is_weak)
            
            for (const auto& flow : data_flows) {
                if (flow.reader_task_id == task_id) {
                    // Find the edge to determine if it's weak or strong
                    bool is_weak = false;
                    for (const auto& edge : graph_edges) {
                        if (edge.writer_task_id == flow.writer_task_id && edge.reader_task_id == task_id) {
                            is_weak = edge.is_weak;
                            break;
                        }
                    }
                    key_to_writer[flow.key] = {flow.writer_task_id, is_weak};
                }
            }
            
            for (const auto& r : reader_keys) {
                std::cout << "    - " << r;
                if (key_to_writer.find(r) != key_to_writer.end()) {
                    uint32_t writer_id = key_to_writer[r].first;
                    bool is_weak = key_to_writer[r].second;
                    const auto& writer_name = graph_nodes[writer_id].task_name;
                    std::string edge_type = is_weak ? "(Weak)" : "(Strong)";
                    
                    std::cout << " ← [" << writer_id << "] " << writer_name 
                              << " " << edge_type;
                }
                std::cout << std::endl;
            }
        }

        if (!all_params.empty()) {
            std::cout << "\n  [Parameters]" << std::endl;
            for (const auto& param : all_params) {
                std::cout << "    - " << param.key << " (" << param.data_size << " bytes)" << std::endl;
            }
        }
    }

    std::cout << "\n[Execution Statistics]" << std::endl;
    long long exec_count = stats.exec_count.load(std::memory_order_relaxed);
    if (exec_count > 0) {
        double avg_exec_us = (stats.total_exec_time_ns.load(std::memory_order_relaxed) / (double)exec_count) / 1000.0;
        double peak_exec_us = stats.max_exec_time_ns.load(std::memory_order_relaxed) / 1000.0;
        double avg_latency_us = (stats.total_latency_ns.load(std::memory_order_relaxed) / (double)exec_count) / 1000.0;
        double peak_latency_us = stats.max_latency_ns.load(std::memory_order_relaxed) / 1000.0;

        std::cout << "Execution Count: " << exec_count << std::endl;
        std::cout << "Avg/Peak Exec:   " << std::fixed << std::setprecision(2) 
                  << avg_exec_us << " / " << peak_exec_us << " µs" << std::endl;
        std::cout << "Avg/Peak Latency: " << avg_latency_us << " / " << peak_latency_us << " µs" << std::endl;
        std::cout << "Overrun:         " << (stats.has_overrun.load(std::memory_order_relaxed) ? "Yes" : "No") << std::endl;
    } else {
        std::cout << "No execution data available" << std::endl;
    }

    std::cout << std::endl;
    return 0;
}

std::string TaskCommandHandler::help() const {
    return R"(
Task Control Commands:

  task list
    List all registered tasks with their IDs and names

  task info <task_id> [--v | -v]
    Show detailed information for a specific task:
    - Name, ID, and scheduling parameters
    - Frequency, CPU affinity, RT/Non-RT type
    - Data I/O summary: counts of Writers, Readers, Parameters
    - Execution statistics (execution count, latency, overrun status)

    With --v or -v option:
    - Display full list of Writers (data keys this task writes)
    - Display full list of Readers (data keys this task reads)
    - Display full list of Parameters (all available parameters)

  task enable <task_id>
    Enable a task
    The change takes effect at the next tick with strong chain propagation

  task disable <task_id>
    Disable a task (downstream tasks are automatically disabled via strong chain)
    The change takes effect at the next tick

    Examples:
      task list                # Show all task names and IDs
      task info 0              # Show task 0 info with data I/O counts
      task info 0 --v          # Show task 0 info with detailed I/O lists
      task disable 0           # Disable task 0
      task enable 0            # Re-enable task 0
)";
}

} // namespace rtcli

