#include "record_commands.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <unordered_map>
#include <cstring>

#include <rtfw_common/log_format.h>
#include <rtfw_connect/shm_controller.h>

namespace rtcli {

// ============================================================================
// RecordCommandHandler
// ============================================================================

int RecordCommandHandler::execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) {
    if (!connector.isAttached()) {
        std::cerr << "Error: Failed to connect to shared memory" << std::endl;
        return 2;
    }

    if (args.empty()) {
        std::cerr << "record: missing subcommand" << std::endl;
        std::cout << help();
        return 1;
    }

    const auto& cmd = args[0];
    if (cmd == "start") {
        if (args.size() < 2) {
            std::cerr << "record start: missing path argument" << std::endl;
            return 1;
        }
        return handle_record_start(args[1], connector);
    } else if (cmd == "stop") {
        return handle_record_stop(connector);
    } else {
        std::cerr << "record: unknown subcommand '" << cmd << "'" << std::endl;
        std::cout << help();
        return 1;
    }
}

int RecordCommandHandler::handle_record_start(const std::string& path, rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.startRecord(path);
    std::cout << "Wrote START_RECORD -> " << path << std::endl;
    return 0;
}

int RecordCommandHandler::handle_record_stop(rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.stopRecord();
    std::cout << "Wrote STOP_RECORD" << std::endl;
    return 0;
}

std::string RecordCommandHandler::help() const {
    return
        "record                  - Recording control\n"
        "  start <path>          - Start recording to file\n"
        "  stop                  - Stop recording\n";
}

// ============================================================================
// ReplayCommandHandler
// ============================================================================

int ReplayCommandHandler::execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) {
    if (!connector.isAttached()) {
        std::cerr << "Error: Failed to connect to shared memory" << std::endl;
        return 2;
    }

    if (args.empty()) {
        std::cerr << "replay: missing subcommand" << std::endl;
        std::cout << help();
        return 1;
    }

    const auto& cmd = args[0];
    if (cmd == "start") {
        if (args.size() < 2) {
            std::cerr << "replay start: missing path argument" << std::endl;
            return 1;
        }
        return handle_replay_start(args[1], connector);
    } else if (cmd == "stop") {
        return handle_replay_stop(connector);
    } else {
        std::cerr << "replay: unknown subcommand '" << cmd << "'" << std::endl;
        std::cout << help();
        return 1;
    }
}

int ReplayCommandHandler::handle_replay_start(const std::string& path, rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.startReplay(path);
    std::cout << "Wrote START_REPLAY -> " << path << std::endl;
    return 0;
}

int ReplayCommandHandler::handle_replay_stop(rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.stopReplay();
    std::cout << "Wrote STOP_REPLAY" << std::endl;
    return 0;
}

std::string ReplayCommandHandler::help() const {
    return
        "replay                  - Replay control\n"
        "  start <path>          - Start replay from file\n"
        "  stop                  - Stop replay\n";
}

// ============================================================================
// TraceCommandHandler
// ============================================================================

int TraceCommandHandler::execute(const std::vector<std::string>& args, rtfw::connect::SharedMemoryConnector& connector) {
    if (!connector.isAttached()) {
        std::cerr << "Error: Failed to connect to shared memory" << std::endl;
        return 2;
    }

    if (args.empty()) {
        std::cerr << "trace: missing subcommand" << std::endl;
        std::cout << help();
        return 1;
    }

    const auto& cmd = args[0];
    if (cmd == "start") {
        if (args.size() < 3) {
            std::cerr << "trace start: missing path arguments (need replay and record paths)" << std::endl;
            return 1;
        }
        return handle_trace_start(args[1], args[2], connector);
    } else if (cmd == "stop") {
        return handle_trace_stop(connector);
    } else {
        std::cerr << "trace: unknown subcommand '" << cmd << "'" << std::endl;
        std::cout << help();
        return 1;
    }
}

int TraceCommandHandler::handle_trace_start(const std::string& replay_path,
                                            const std::string& record_path,
                                            rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.startTrace(replay_path, record_path);
    std::cout << "Wrote START_TRACE -> replay=" << replay_path
              << " record=" << record_path << std::endl;
    return 0;
}

int TraceCommandHandler::handle_trace_stop(rtfw::connect::SharedMemoryConnector& connector) {
    rtfw::connect::SharedMemoryController controller(connector.getBasePtr());
    controller.stopTrace();
    std::cout << "Wrote STOP_TRACE" << std::endl;
    return 0;
}

std::string TraceCommandHandler::help() const {
    return
        "trace                   - Replay + record simultaneously\n"
        "  start <replay> <rec>  - Start trace (replay + record)\n"
        "  stop                  - Stop trace\n";
}

// ============================================================================
// CheckRecordCommandHandler
// ============================================================================

int CheckRecordCommandHandler::execute(const std::vector<std::string>& args,
                                        rtfw::connect::SharedMemoryConnector& /*connector*/) {
    if (args.empty()) {
        std::cerr << "check_record: missing path argument" << std::endl;
        std::cout << help();
        return 1;
    }

    bool show_hash = false;
    if (args.size() >= 2 && args[1] == "--show-hash") {
        show_hash = true;
    }

    return validate_record_file(args[0].c_str(), show_hash);
}

int CheckRecordCommandHandler::validate_record_file(const char* path, bool show_hash) {
    std::ifstream is(path, std::ios::binary);
    if (!is) {
        std::cerr << "Error: Failed to open record file: " << path << std::endl;
        return 2;
    }

    rtfw::common::FileHeader file_header{};
    is.read(reinterpret_cast<char*>(&file_header), sizeof(file_header));
    if (!is || std::memcmp(file_header.magic, "RTFA", 4) != 0) {
        std::cerr << "Error: Invalid file header" << std::endl;
        return 3;
    }

    // 헤더 정보 출력
    std::cout << "=== File Header Info ===" << std::endl;
    std::cout << "  base_frequency: " << file_header.base_frequency << " Hz" << std::endl;
    std::cout << "  last_tick: " << file_header.last_tick << std::endl;
    std::cout << "  metadata_offset: " << file_header.metadata_offset << std::endl;
    std::cout << std::endl;

    rtfw::common::MetadataHeader meta_header{};
    is.seekg(file_header.metadata_offset, std::ios::beg);
    is.read(reinterpret_cast<char*>(&meta_header), sizeof(meta_header));
    if (!is || std::memcmp(meta_header.magic, "RTMD", 4) != 0) {
        std::cerr << "Error: Invalid metadata header" << std::endl;
        return 4;
    }

    if (meta_header.total_metadata_size < sizeof(rtfw::common::MetadataHeader)) {
        std::cerr << "Error: Metadata size invalid" << std::endl;
        return 5;
    }

    is.seekg(0, std::ios::end);
    const auto file_size = static_cast<uint64_t>(is.tellg());
    if (file_header.metadata_offset + meta_header.total_metadata_size > file_size) {
        std::cerr << "Error: Metadata exceeds file size" << std::endl;
        return 6;
    }

    is.seekg(file_header.metadata_offset + sizeof(rtfw::common::MetadataHeader), std::ios::beg);
    std::unordered_map<uint64_t, std::string> key_names;
    key_names[rtfw::common::CHECKPOINT_KEY_HASH] = "<CHECKPOINT>";
    uint64_t meta_bytes_read = sizeof(rtfw::common::MetadataHeader);
    std::vector<char> key_buffer;

    for (uint32_t i = 0; i < meta_header.entry_count; ++i) {
        if (meta_bytes_read + sizeof(rtfw::common::KeyMappingEntry) > meta_header.total_metadata_size) {
            std::cerr << "Error: Metadata entry header exceeds metadata size" << std::endl;
            return 7;
        }
        rtfw::common::KeyMappingEntry entry{};
        is.read(reinterpret_cast<char*>(&entry), sizeof(entry));
        if (!is) {
            std::cerr << "Error: Failed to read metadata entry" << std::endl;
            return 8;
        }
        meta_bytes_read += sizeof(entry);

        if (entry.key_length == 0) {
            std::cerr << "Error: Metadata key length invalid" << std::endl;
            return 9;
        }
        if (meta_bytes_read + entry.key_length > meta_header.total_metadata_size) {
            std::cerr << "Error: Metadata key data exceeds metadata size" << std::endl;
            return 10;
        }

        key_buffer.resize(entry.key_length);
        is.read(key_buffer.data(), entry.key_length);
        if (!is) {
            std::cerr << "Error: Failed to read metadata key" << std::endl;
            return 11;
        }
        meta_bytes_read += entry.key_length;

        const size_t key_len = entry.key_length > 0 ? static_cast<size_t>(entry.key_length - 1) : 0;
        key_names[entry.key_hash] = std::string(key_buffer.data(), key_len);
    }

    const uint64_t data_start = file_header.metadata_offset + meta_header.total_metadata_size;
    is.seekg(data_start, std::ios::beg);

    struct KeyStats {
        uint64_t first_tick = 0;
        uint64_t last_tick = 0;
        uint64_t count = 0;
        uint64_t bytes = 0;
        uint64_t fixed_size = 0;
        uint64_t last_size = 0;
        uint64_t type_hash = 0;
        bool type_conflict = false;
        bool size_conflict = false;
        bool has_tick = false;
    };

    std::unordered_map<uint64_t, uint64_t> last_tick_per_key;
    std::unordered_map<uint64_t, KeyStats> stats_per_key;
    uint64_t total_entries = 0;
    uint64_t total_bytes = 0;
    uint64_t bad_entries = 0;

    while (is && is.peek() != EOF) {
        const auto entry_pos = is.tellg();
        rtfw::common::LogEntryHeader header{};
        is.read(reinterpret_cast<char*>(&header), sizeof(header));
        if (!is) break;

        const auto after_header_pos = is.tellg();
        is.seekg(0, std::ios::end);
        const auto file_end_pos = is.tellg();
        if (after_header_pos == std::streampos(-1) || file_end_pos == std::streampos(-1)) {
            std::cerr << "Error: Failed to determine file size during scan" << std::endl;
            return 12;
        }

        const uint64_t remaining = static_cast<uint64_t>(file_end_pos - after_header_pos);
        if (header.data_size > remaining) {
            std::cerr << "Error: Entry data_size exceeds remaining file bytes at offset "
                      << static_cast<uint64_t>(entry_pos) << std::endl;
            return 13;
        }
        is.seekg(after_header_pos, std::ios::beg);

        if (header.end_tick < header.start_tick) {
            std::cerr << "Error: Entry has end_tick < start_tick at offset "
                      << static_cast<uint64_t>(entry_pos) << std::endl;
            return 14;
        }

        auto it = last_tick_per_key.find(header.key_hash);
        if (it != last_tick_per_key.end() && header.start_tick < it->second) {
            std::cerr << "Error: Non-monotonic start_tick for key_hash=" << header.key_hash
                      << " at offset " << static_cast<uint64_t>(entry_pos) << std::endl;
            return 15;
        }
        last_tick_per_key[header.key_hash] = header.start_tick;

        auto& stats = stats_per_key[header.key_hash];
        if (!stats.has_tick) {
            stats.first_tick = header.start_tick;
            stats.last_tick = header.start_tick;
            stats.has_tick = true;
        } else {
            if (header.start_tick < stats.first_tick) stats.first_tick = header.start_tick;
            if (header.start_tick > stats.last_tick) stats.last_tick = header.start_tick;
        }

        if (stats.count == 0) {
            stats.type_hash = header.type_hash;
            stats.fixed_size = header.data_size;
        } else if (stats.type_hash != header.type_hash) {
            stats.type_conflict = true;
        }

        if (stats.fixed_size != header.data_size) {
            stats.size_conflict = true;
        }
        stats.last_size = header.data_size;
        stats.count += 1;
        stats.bytes += header.data_size;

        is.seekg(header.data_size, std::ios::cur);
        if (!is) {
            std::cerr << "Error: Failed to skip entry data at offset "
                      << static_cast<uint64_t>(entry_pos) << std::endl;
            return 16;
        }

        if (key_names.find(header.key_hash) == key_names.end()) {
            ++bad_entries;
        }

        total_entries++;
        total_bytes += sizeof(rtfw::common::LogEntryHeader) + header.data_size;
    }

    std::cout << "Record OK: entries=" << total_entries
              << ", bytes=" << total_bytes
              << ", keys=" << key_names.size()
              << ", unknown_key_entries=" << bad_entries
              << std::endl;

    std::cout << "\nKey Statistics:\n";
    constexpr int kNameWidth = 32;
    constexpr int kHashWidth = 20;
    constexpr int kTypeWidth = 20;
    constexpr int kSizeWidth = 12;
    constexpr int kTickWidth = 14;
    constexpr int kCountWidth = 10;
    constexpr int kBytesWidth = 14;
    constexpr int kAvgWidth = 12;

    if (show_hash) {
        std::cout << std::left
                  << std::setw(kHashWidth) << "key_hash"
                  << std::setw(kNameWidth) << "name"
                  << std::setw(kTypeWidth) << "type_hash"
                  << std::setw(kSizeWidth) << "fixed_size"
                  << std::setw(kTickWidth) << "first_tick"
                  << std::setw(kTickWidth) << "last_tick"
                  << std::setw(kCountWidth) << "count"
                  << std::setw(kBytesWidth) << "bytes"
                  << std::setw(kAvgWidth) << "avg_size"
                  << std::setw(kSizeWidth) << "last_size"
                  << "\n";
    } else {
        std::cout << std::left
                  << std::setw(kNameWidth) << "name"
                  << std::setw(kTickWidth) << "first_tick"
                  << std::setw(kTickWidth) << "last_tick"
                  << std::setw(kCountWidth) << "count"
                  << std::setw(kBytesWidth) << "bytes"
                  << std::setw(kAvgWidth) << "avg_size"
                  << std::setw(kSizeWidth) << "last_size"
                  << "\n";
    }

    for (const auto& [key_hash, stats] : stats_per_key) {
        const auto name_it = key_names.find(key_hash);
        const std::string& name = (name_it != key_names.end()) ? name_it->second : std::string("<UNKNOWN>");
        const uint64_t avg_size = stats.count > 0 ? (stats.bytes / stats.count) : 0;

        if (show_hash) {
            std::cout << std::left
                      << std::setw(kHashWidth) << key_hash
                      << std::setw(kNameWidth) << name;
            if (stats.type_conflict) {
                std::cout << std::setw(kTypeWidth) << "<MIXED>";
            } else {
                std::cout << std::setw(kTypeWidth) << stats.type_hash;
            }
            if (stats.size_conflict) {
                std::cout << std::setw(kSizeWidth) << "<MIXED>";
            } else {
                std::cout << std::setw(kSizeWidth) << stats.fixed_size;
            }
            if (stats.has_tick) {
                std::cout << std::setw(kTickWidth) << stats.first_tick
                          << std::setw(kTickWidth) << stats.last_tick;
            } else {
                std::cout << std::setw(kTickWidth) << "-"
                          << std::setw(kTickWidth) << "-";
            }
            std::cout << std::setw(kCountWidth) << stats.count
                      << std::setw(kBytesWidth) << stats.bytes
                      << std::setw(kAvgWidth) << avg_size
                      << std::setw(kSizeWidth) << stats.last_size
                      << "\n";
        } else {
            std::cout << std::left
                      << std::setw(kNameWidth) << name;
            if (stats.has_tick) {
                std::cout << std::setw(kTickWidth) << stats.first_tick
                          << std::setw(kTickWidth) << stats.last_tick;
            } else {
                std::cout << std::setw(kTickWidth) << "-"
                          << std::setw(kTickWidth) << "-";
            }
            std::cout << std::setw(kCountWidth) << stats.count
                      << std::setw(kBytesWidth) << stats.bytes
                      << std::setw(kAvgWidth) << avg_size
                      << std::setw(kSizeWidth) << stats.last_size
                      << "\n";
        }
    }

    return 0;
}

std::string CheckRecordCommandHandler::help() const {
    return
        "check_record <path> ... - Validate and analyze record files\n"
        "  [--show-hash]         - Show key_hash values in output\n";
}

} // namespace rtcli
