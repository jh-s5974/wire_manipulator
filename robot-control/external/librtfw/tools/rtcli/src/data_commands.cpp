#include "data_commands.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <rtfw_connect/shm_querier.h>

namespace rtcli {

int DataCommandHandler::execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) {
    if (!connector.isAttached()) {
        std::cerr << "Error: Failed to connect to shared memory" << std::endl;
        return 2;
    }

    if (args.empty()) {
        std::cout << help();
        return 1;
    }

    const std::string& subcmd = args[0];

    if (subcmd == "list") {
        // data list - Show all data keys
        return showDataList(connector);

    } else if (subcmd == "info") {
        // data info <key> - Show detailed info for specific data key
        if (args.size() < 2) {
            std::cerr << "Usage: data info <key>" << std::endl;
            return 1;
        }
        return showDataInfo(connector, args[1]);

    } else {
        std::cerr << "Unknown subcommand: " << subcmd << std::endl;
        std::cout << help();
        return 1;
    }
}

int DataCommandHandler::showDataList(rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryQuerier querier(connector.getBasePtr());
    if (!querier.isConnected()) {
        std::cerr << "ERROR: Cannot access shared memory" << std::endl;
        return 2;
    }

    auto descriptors = querier.getAllDescriptors();
    if (descriptors.empty()) {
        std::cout << "No data keys registered." << std::endl;
        return 0;
    }

    // Sort by key name for better readability
    std::sort(descriptors.begin(), descriptors.end(), 
        [](const auto& a, const auto& b) { 
            return std::string(a.key) < std::string(b.key); 
        });

    std::cout << "\n[Data Keys]" << std::endl;
    std::cout << std::left 
              << std::setw(40) << "Key" 
              << std::setw(12) << "Size (B)" 
              << std::setw(12) << "Writer ID"
              << std::endl;
    std::cout << std::string(64, '-') << std::endl;

    for (const auto& desc : descriptors) {
        std::cout << std::setw(40) << desc.key 
                  << std::setw(12) << desc.data_size 
                  << std::setw(12) << desc.task_id
                  << std::endl;
    }

    std::cout << "\nTotal: " << descriptors.size() << " data key(s)" << std::endl;
    return 0;
}

int DataCommandHandler::showDataInfo(rtfw::connect::SharedMemoryConnector& connector, const std::string& key) {
    rtfw::connect::SharedMemoryQuerier querier(connector.getBasePtr());
    if (!querier.isConnected()) {
        std::cerr << "ERROR: Cannot access shared memory" << std::endl;
        return 2;
    }

    const auto* desc = querier.getDescriptor(key);
    if (!desc) {
        std::cerr << "ERROR: Data key '" << key << "' not found" << std::endl;
        return 1;
    }

    // Get task name from task stats
    const auto* task_stats = querier.getTaskStatsArray();
    size_t task_count = querier.getTaskStatsCount();
    std::string writer_name = "Unknown";
    if (desc->task_id < task_count) {
        writer_name = task_stats[desc->task_id].task_name;
    }

    std::cout << "\n[Data Key Info: " << key << "]" << std::endl;
    std::cout << "  Writer Task ID    : " << desc->task_id << " (" << writer_name << ")" << std::endl;
    std::cout << "  Key Hash          : 0x" << std::hex << desc->key_hash << std::dec << std::endl;
    std::cout << "  Type Hash         : 0x" << std::hex << desc->type_hash << std::dec << std::endl;
    std::cout << "  Data Size         : " << desc->data_size << " bytes" << std::endl;
    std::cout << "  Buffer Count      : " << desc->block_count << std::endl;
    std::cout << "  Data Alignment    : " << desc->data_alignment << std::endl;
    std::cout << "  Data Stride       : " << desc->data_stride << std::endl;
    std::cout << "  Buffer Offset     : 0x" << std::hex << desc->buffer_header_offset << std::dec << std::endl;
    std::cout << std::endl;

    return 0;
}

std::string DataCommandHandler::help() const {
    return
        "data                    - Inspect data keys\n"
        "  list                  - List all registered data keys\n"
        "  info <key>            - Show detailed info for a specific data key\n";
}

} // namespace rtcli
