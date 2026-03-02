// FileBlackbox.cpp
#include "rtfw_common/FileBlackbox.h" // FileBlackbox 클래스 정의 헤더
#include "rtfw_common/log_format.h" // FileHeader, LogEntryHeader 등
#include <fstream>
#include <iostream>
#include <memory>
#include <chrono>
#include <cstring>
#include <unordered_set>

using namespace rtfw::common;


namespace rtfw::blackbox {
    // --- 생성자 및 소멸자 ---
    FileBlackbox::FileBlackbox(size_t queue_capacity) 
        : _mode(Mode::NONE),
          _record_queue_capacity(queue_capacity),
          _record_queue(queue_capacity),
          _free_items(0)  // will be reserved in initialize_record_pool
    {}

    FileBlackbox::~FileBlackbox() {
        shutdown();
        cleanup_record_pool();
    };

    // --- Public API 구현 ---

    bool FileBlackbox::start(std::string path, Mode mode) {
        shutdown(); // 이전 상태 정리
        _has_tick_offset = false;
        _has_file_base = false;
        _tick_offset = 0;
        _mode = mode;
        _filepath = path;

        switch (mode) {
            case Mode::RECORD: {
                _ostream.open(_filepath, std::ios::binary | std::ios::trunc);
                if (!_ostream) {
                    std::cerr << "Error: Failed to open file for recording: " << _filepath << std::endl;
                    return false;
                }
                
                return true;
            }
            case Mode::REPLAY: {
                _istream.open(_filepath, std::ios::binary);
                if (!_istream) {
                    std::cerr << "Error: Failed to open file for simulation: " << _filepath << std::endl;
                    return false;
                }
                
                return true;
            }
            default: {
                std::cerr << "Error: Invalid mode selected" << std::endl;
                return false;
            }

        }
    }


    void FileBlackbox::shutdown() {
        if (_ostream.is_open()) {
            // Recording 모드 종료 시, 최종 헤더 정보 업데이트
            _stop_writer_thread = true;
            if (_writer_thread.joinable()) {
                _writer_thread.join();
            }

            if (_last_written_tick > 0) {
                _file_header.last_tick = _last_written_tick;
                _ostream.seekp(0, std::ios::beg);
                _ostream.write(reinterpret_cast<const char*>(&_file_header), sizeof(_file_header));
            }
            _ostream.close();
            std::cout << _filepath << " saved." << std::endl;
        }
        if (_istream.is_open()) {
            _istream.close();
        }

        // NOTE: Do not clear _cache_keymap here. Cache slots can still be
        // referenced by DataWriter proxies during runtime; clearing would
        // free their storage and risk use-after-free. Slots are kept for the
        // lifetime of the backend and reused across start/stop cycles.
        _mode = Mode::NONE;
    }


    Mode FileBlackbox::getMode() const {
        return _mode;
    }

    size_t FileBlackbox::getLastTick() const {
        return _file_header.last_tick;
    }

    uint32_t FileBlackbox::frequency() const {
        return _file_header.base_frequency;
    }

    std::string FileBlackbox::getKeyName(uint64_t keyhash) const {
        auto it = _keyname.find(keyhash);
        if (it == _keyname.end())
            return "";
        return it->second;
    }
    // ... 기타 유틸리티 함수 구현 ...

    // --- Object Pool Management (RT-safe allocation) ---
    void FileBlackbox::initialize_record_pool(size_t pool_size, size_t max_data_size) {
        cleanup_record_pool();  // Clean up any existing pool
        
        _pool_size = pool_size;
        _pool_alloc_failures.store(0);
        
        // Pre-allocate all work items
        _work_item_pool.resize(pool_size);
        
        // Pre-allocate data buffers for each item
        for (size_t i = 0; i < pool_size; ++i) {
            _work_item_pool[i].data.reserve(max_data_size);
        }
        
        // Reserve capacity for lock-free stack (boost >= 1.53)
        _free_items.reserve(pool_size);
        
        // Push all items to free list
        for (size_t i = 0; i < pool_size; ++i) {
            _free_items.push(&_work_item_pool[i]);
        }
        
        std::cout << "[FileBlackbox] Initialized object pool: " << pool_size 
                  << " items, max_data_size=" << max_data_size << " bytes" << std::endl;
    }

    void FileBlackbox::cleanup_record_pool() {
        // Drain the free list
        RecordWorkItem* item = nullptr;
        while (_free_items.pop(item)) {
            // Items are owned by _work_item_pool vector, no delete needed
        }
        _work_item_pool.clear();
        _pool_size = 0;
        
        if (_pool_alloc_failures.load() > 0) {
            std::cerr << "[FileBlackbox] Warning: " << _pool_alloc_failures.load() 
                      << " pool allocation failures occurred during recording" << std::endl;
        }
    }

    RecordWorkItem* FileBlackbox::acquire_work_item() {
        RecordWorkItem* item = nullptr;
        if (_free_items.pop(item)) {
            return item;
        }
        // Pool exhausted - should not happen with proper sizing
        _pool_alloc_failures.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }

    void FileBlackbox::release_work_item(RecordWorkItem* item) {
        if (item) {
            // Do NOT clear data: keep size intact so that resize() in onTick
            // is skipped on reuse (same-size keys avoid zero-init overhead).
            // Capacity is preserved from pool init (reserve(max_data_size)).
            _free_items.push(item);
        }
    }

    // --- private + friend API 구현 ---

    bool FileBlackbox::initialize_metadata(const std::map<uint64_t, std::string>& target_keys,
                                           uint32_t base_frequency,
                                           const std::map<uint64_t, size_t>& key_sizes,
                                           const std::vector<uint8_t>& checkpoint,
                                           uint64_t start_tick) {
        if (_mode == Mode::RECORD) {
            // 파일에 메타데이터 기록
            _file_header.base_frequency = base_frequency;
            _file_header.start_timestamp_ns = std::chrono::high_resolution_clock::now().time_since_epoch().count();
            _file_header.metadata_offset = sizeof(FileHeader);
            _ostream.write(reinterpret_cast<const char*>(&_file_header), sizeof(_file_header));

            uint64_t metadata_chunk_size = sizeof(MetadataHeader);
            for (const auto& pair : target_keys) {
                metadata_chunk_size += sizeof(KeyMappingEntry) + pair.second.length()+1;
            }

            _meta_header = MetadataHeader{};
            _meta_header.entry_count = target_keys.size();
            _meta_header.total_metadata_size = metadata_chunk_size;
            _ostream.write(reinterpret_cast<const char*>(&_meta_header), sizeof(_meta_header));
            
            _entry_headers.clear();
            _keyname.clear();
            for (const auto& pair : target_keys) {
                const size_t key_len_with_null = pair.second.length() + 1;
                if (key_len_with_null > 255) {
                    std::cerr << "Error: Key name too long for metadata (max 254 chars + null). Key: " << pair.second << std::endl;
                    return false;
                }
                KeyMappingEntry entry_header;
                _keyname.insert(pair);
                entry_header.key_hash = pair.first;
                entry_header.key_length = static_cast<uint8_t>(key_len_with_null);
                _entry_headers.push_back(entry_header);
                _ostream.write(reinterpret_cast<const char*>(&entry_header), sizeof(entry_header));
                _ostream.write(pair.second.c_str(), key_len_with_null);
            }

            _data_start_offset = _ostream.tellp();
            
            // 기록할 키 목록에 대한 CacheSlot 생성
            size_t max_data_size = 0;
            for (const auto& pair : target_keys) {
                auto &slot = _cache_keymap[pair.first];
                slot.update.store(false, std::memory_order_relaxed);
                auto it = key_sizes.find(pair.first);
                if (it != key_sizes.end() && slot.data.size() == 0) {
                    slot.data.resize(it->second);
                    max_data_size = std::max(max_data_size, it->second);
                }
            }
            
            // Initialize object pool for RT-safe recording
            // Pool size: queue_capacity (typically 16384) to match record queue depth
            // This ensures we never run out of work items if queue isn't full
            size_t pool_size = _record_queue_capacity;
            if (max_data_size == 0) max_data_size = 4096;  // Default if no sizes specified
            initialize_record_pool(pool_size, max_data_size);

            _stop_writer_thread = false;
            _writer_thread = std::thread(&FileBlackbox::writer_thread_func, this);

            _last_written_tick = 0;
            if (!checkpoint.empty()) {
                // For checkpoint-based recordings, make file ticks relative to start_tick (file base = 0)
                _file_base_tick = start_tick;
                _has_file_base = true;

                auto* item = acquire_work_item();
                if (!item) {
                    std::cerr << "Error: Failed to acquire work item from pool for checkpoint" << std::endl;
                    return false;
                }
                item->start_tick = 0;
                item->end_tick = 0;
                item->key_hash = rtfw::common::CHECKPOINT_KEY_HASH;
                item->type_hash = 0;
                item->data.assign(reinterpret_cast<const char*>(checkpoint.data()),
                                  reinterpret_cast<const char*>(checkpoint.data()) + checkpoint.size());

                if (!_record_queue.push(item)) {
                    std::cerr << "Error: record queue full when pushing checkpoint" << std::endl;
                    release_work_item(item);  // Return to pool instead of delete
                    return false;
                }
            }
            return true;

        } else if (_mode == Mode::REPLAY) {
            _istream.seekg(0, std::ios::beg);


            // 파일에서 메타데이터를 읽고, target_keys와 호환되는지 검증
            _istream.read(reinterpret_cast<char*>(&_file_header), sizeof(_file_header));
            if (std::memcmp(_file_header.magic, "RTFA", 4) != 0) {
                std::cout << "file header invalid" << std::endl;
                return false;
            }

            if (base_frequency > 0 && _file_header.base_frequency != base_frequency) {
                std::cout << "base frequency not matched." << std::endl;
                return false;
            }

            _istream.seekg(_file_header.metadata_offset, std::ios::beg);

            _istream.read(reinterpret_cast<char*>(&_meta_header), sizeof(_meta_header));
            if (std::memcmp(_meta_header.magic, "RTMD", 4) != 0) {
                std::cout << "metadata header invalid" << std::endl;
                return false;
            }

            if (_meta_header.total_metadata_size < sizeof(MetadataHeader)) {
                std::cout << "metadata size invalid" << std::endl;
                return false;
            }

            const auto meta_start = _file_header.metadata_offset;
            _istream.seekg(0, std::ios::end);
            const auto file_size = static_cast<uint64_t>(_istream.tellg());
            if (meta_start + _meta_header.total_metadata_size > file_size) {
                std::cout << "metadata extends beyond file size" << std::endl;
                return false;
            }
            _istream.seekg(_file_header.metadata_offset + sizeof(MetadataHeader), std::ios::beg);
            
            std::unordered_set<uint64_t> keys_in_log_hash;
            std::vector<char> key_buffer;
            _entry_headers.clear();
            _keyname.clear();
            uint64_t meta_bytes_read = sizeof(MetadataHeader);
            for (uint32_t i = 0; i < _meta_header.entry_count; ++i) {
                if (meta_bytes_read + sizeof(KeyMappingEntry) > _meta_header.total_metadata_size) {
                    std::cout << "metadata entry header exceeds metadata size" << std::endl;
                    return false;
                }
                KeyMappingEntry entry_header;
                _istream.read(reinterpret_cast<char*>(&entry_header), sizeof(entry_header));
                meta_bytes_read += sizeof(KeyMappingEntry);

                if (entry_header.key_length == 0) {
                    std::cout << "metadata key length invalid" << std::endl;
                    return false;
                }
                if (meta_bytes_read + entry_header.key_length > _meta_header.total_metadata_size) {
                    std::cout << "metadata key data exceeds metadata size" << std::endl;
                    return false;
                }

                key_buffer.resize(entry_header.key_length);
                _istream.read(key_buffer.data(), entry_header.key_length);
                meta_bytes_read += entry_header.key_length;

                keys_in_log_hash.insert(entry_header.key_hash);
                _entry_headers.push_back(entry_header);
                const size_t key_len = entry_header.key_length > 0 ? static_cast<size_t>(entry_header.key_length - 1) : 0;
                _keyname.insert({entry_header.key_hash, std::string(key_buffer.data(), key_len)});
            }

            if (!target_keys.empty()) {
                for (const auto& required : target_keys) {
                    if (keys_in_log_hash.find(required.first) == keys_in_log_hash.end()) {
                        std::cerr << "Replay Error: Required key '" << required.second << "' not found in log file." << std::endl;
                        return false;
                    }
                    _stream_buffers[required.first];
                    auto &slot = _cache_keymap[required.first];
                    slot.update.store(false, std::memory_order_relaxed);
                    auto it = key_sizes.find(required.first);
                    if (it != key_sizes.end() && slot.data.size() == 0) {
                        slot.data.resize(it->second);
                    }
                }
            } else {
                for (const auto& keyhash : keys_in_log_hash) {
                    _stream_buffers[keyhash];
                    auto &slot = _cache_keymap[keyhash];
                    slot.update.store(false, std::memory_order_relaxed);
                    auto it = key_sizes.find(keyhash);
                    if (it != key_sizes.end() && slot.data.size() == 0) {
                        slot.data.resize(it->second);
                    }
                }
            }

            // Always track the reserved checkpoint key stream.
            // CHECKPOINT records are intentionally not part of normal metadata keymap,
            // but replay needs to read and extract the initial checkpoint payload.
            _stream_buffers[rtfw::common::CHECKPOINT_KEY_HASH];

            // 데이터 시작점으로 파일 포인터 이동
            _data_start_offset = _file_header.metadata_offset + _meta_header.total_metadata_size;
            _istream.seekg(_data_start_offset, std::ios::beg);
            scan_and_fill_buffers();

            // If file contained a special checkpoint key, extract the first checkpoint entry
            // from the stream buffers (if any) and cache it for quick access.
            auto it = _stream_buffers.find(rtfw::common::CHECKPOINT_KEY_HASH);
            if (it != _stream_buffers.end() && !it->second.empty()) {
                // take the earliest / first checkpoint entry
                const auto& entry = it->second.front();
                _initial_checkpoint.assign(entry.data.begin(), entry.data.end());
                // remember the recorded tick for the checkpoint so callers can map ticks
                _file_checkpoint_tick = entry.start_tick;
                // clear buffer for checkpoint so normal replay doesn't try to treat it as stream data
                it->second.clear();
            }

            // Note: earliest-file-tick tracking removed. Tick-offset will be
            // initialized lazily at first onTick using checkpoint or first
            // buffered entry.
            return true;
        }
        return false;
    }

    CacheSlot* FileBlackbox::getCacheSlot(uint64_t keyhash) {
        auto it = _cache_keymap.find(keyhash);
        if (it == _cache_keymap.end())
            return nullptr;

        return &(it->second);
    }

    void FileBlackbox::onTick(uint64_t current_global_tick) {
        if (_mode == Mode::RECORD) {
            // Recording: 캐시 슬롯을 순회하며 '기록할 데이터'를 파일에 씀
            for (auto& [key_hash, slot] : _cache_keymap) {
                if (!slot.update.load(std::memory_order_acquire))
                    continue;

                // Acquire work item first so we can copy directly into its
                // pre-allocated buffer, avoiding an intermediate heap allocation.
                auto* item = acquire_work_item();
                if (!item) {
                    // Pool exhausted - data will be dropped
                    // Failure is counted in _pool_alloc_failures
                    continue;
                }

                // Ensure item buffer has the right size (it is pre-reserved to
                // max_data_size at pool init, so assign only sets size, no alloc).
                const size_t data_size = slot.data.size();
                if (item->data.size() != data_size) {
                    item->data.resize(data_size);
                }

                // Seqlock-style read: copy directly into item->data without
                // an intermediate data_snapshot vector.
                uint32_t seq_before = 0;
                uint32_t seq_after = 0;
                uint64_t start_tick = 0;
                uint64_t type_hash = 0;
                do {
                    seq_before = slot.seq.load(std::memory_order_acquire);
                    if (seq_before & 1U) continue; // writer in progress

                    start_tick = slot.start_tick.load(std::memory_order_relaxed);
                    type_hash = slot.type_hash;
                    memcpy(item->data.data(), slot.data.data(), data_size);

                    seq_after = slot.seq.load(std::memory_order_acquire);
                } while (seq_before != seq_after || (seq_after & 1U));

                // Clear update flag now that we hold a consistent copy.
                slot.update.store(false, std::memory_order_release);

                uint64_t rel_start = start_tick;
                uint64_t rel_end = current_global_tick;
                if (_has_file_base) {
                    rel_start = (start_tick >= _file_base_tick) ? start_tick - _file_base_tick : 0;
                    rel_end   = (current_global_tick >= _file_base_tick) ? current_global_tick - _file_base_tick : 0;
                }

                item->start_tick = rel_start;
                item->end_tick   = rel_end;
                item->key_hash   = key_hash;
                item->type_hash  = type_hash;
                
                if (!_record_queue.push(item)) {
                    std::cout << "Queue FULL!!" << std::endl;
                    release_work_item(item);  // ✅ Return to pool instead of delete
                    // 큐가 꽉 참. 데이터 드롭.
                }
            }
            // last written tick stored as file-relative tick
            // (_last_written_tick already updated in writer thread when flushing items too)
            // Here we conservatively update to max seen relative start tick
            // (some entries may still be in queue and updated in writer thread)
            // No global to relative conversion needed if _has_file_base is false.
            if (_has_file_base) {
                _last_written_tick = current_global_tick - _file_base_tick;
            } else {
                _last_written_tick = current_global_tick;
            }

        } else if (_mode == Mode::REPLAY) {
            // Tick offset will be initialized on first buffered entry
            // (see below). Checkpoint is used only to restore task state,
            // not for tick mapping.

            // 1. 모든 리플레이 대상 채널(_cache_keymap)을 순회합니다.
            for (auto& [key_hash, slot] : _cache_keymap) {
                auto& buffer = _stream_buffers[key_hash];

                // 2. 해당 채널의 버퍼가 비어있다면, 파일에서 데이터를 더 읽어옵니다.
                if (buffer.empty()) {
                    scan_and_fill_buffers();
                    // 다시 채웠는데도 비어있다면, 파일 끝이라 더 이상 데이터가 없는 것.
                    if (buffer.empty()) {
                        continue; // 다음 채널로 넘어감
                    }
                }
                
                // 3. 버퍼의 맨 앞 데이터를 확인하여 tick을 비교합니다.
                //    If offset isn't set yet (no checkpoint), initialize it here
                //    using the first available file tick so that replay aligns
                //    to the current global tick.
                if (!_has_tick_offset && !buffer.empty()) {
                    int64_t file_tick = static_cast<int64_t>(buffer.front().start_tick);
                    int64_t offset = static_cast<int64_t>(current_global_tick) - file_tick;
                    _tick_offset = offset;
                    _has_tick_offset = true;
                    std::cout << "[FileBlackbox] Initialized tick offset from first buffered entry: " << offset << std::endl;
                }

                // Apply tick-offset mapping when comparing file ticks to global ticks.
                // This will drop entries that are earlier than the current global tick.
                while (!buffer.empty()) {
                    int64_t file_tick = static_cast<int64_t>(buffer.front().start_tick);
                    int64_t mapped_tick = file_tick + (_has_tick_offset ? _tick_offset : 0);
                    if (mapped_tick < static_cast<int64_t>(current_global_tick)) {
                        buffer.pop_front();
                        continue;
                    }
                    break;
                }
                
                // 4. 버퍼가 비었는지 다시 확인 (지나간 데이터를 모두 버린 후)
                if (buffer.empty()) {
                    continue;
                }
                // 5. 정확한 tick이 일치하는 데이터를 찾았습니다 (after mapping).
                {
                    const auto& item = buffer.front();
                    int64_t file_tick = static_cast<int64_t>(item.start_tick);
                    int64_t mapped_tick = file_tick + (_has_tick_offset ? _tick_offset : 0);
                    if (mapped_tick == static_cast<int64_t>(current_global_tick)) {
                        // CacheSlot을 업데이트합니다.
                        slot.type_hash = item.type_hash;
                        slot.data = item.data;
                        slot.start_tick.store(static_cast<uint64_t>(mapped_tick));
                        slot.end_tick.store(static_cast<uint64_t>(item.end_tick + (_has_tick_offset ? _tick_offset : 0)));
                        slot.update.store(true, std::memory_order_release);

                        // 처리한 데이터를 버퍼에서 제거합니다.
                        buffer.pop_front();
                    }
                }
                // else (buffer.front().start_tick > current_global_tick)
                //    -> 이 경우는 '아직 처리할 때가 아니므로' 아무것도 하지 않고 스킵합니다.
            }
        }
    }

    void FileBlackbox::scan_and_fill_buffers() {
        // 버퍼가 특정 크기(예: 10)에 도달할 때까지 또는 파일 끝까지 스캔합니다.
        const size_t BUFFER_TARGET_SIZE = 2;
        std::streampos last_pos;

        while (_istream && _istream.peek() != EOF) {
            // 모든 버퍼가 목표치 이상 채워졌는지 확인
            bool all_buffers_full = true;
            for (const auto& [key, buffer] : _stream_buffers) {
                if (buffer.size() < BUFFER_TARGET_SIZE) {
                    all_buffers_full = false;
                    break;
                }
            }
            if (all_buffers_full) {
                return; // 모든 버퍼가 충분히 찼으므로 스캔 중단
            }

            last_pos = _istream.tellg();
            LogEntryHeader header;
            _istream.read(reinterpret_cast<char*>(&header), sizeof(header));
            if (!_istream) { // 읽기 실패 (파일 끝 등)
                _istream.clear();
                _istream.seekg(last_pos);
                return;
            }

            // Validate data_size against remaining file bytes to avoid heap corruption
            const auto after_header_pos = _istream.tellg();
            _istream.seekg(0, std::ios::end);
            const auto file_end_pos = _istream.tellg();
            if (after_header_pos == std::streampos(-1) || file_end_pos == std::streampos(-1)) {
                _istream.clear();
                _istream.seekg(last_pos);
                return;
            }
            const uint64_t remaining_bytes = static_cast<uint64_t>(file_end_pos - after_header_pos);
            if (header.data_size > remaining_bytes) {
                std::cerr << "Replay Error: log entry data_size exceeds remaining file bytes (data_size="
                          << header.data_size << ", remaining=" << remaining_bytes << ")" << std::endl;
                _istream.clear();
                _istream.seekg(last_pos);
                return;
            }
            if (header.data_size == 0) {
                std::cerr << "Replay Error: log entry data_size is zero at offset "
                          << static_cast<uint64_t>(last_pos) << std::endl;
                _istream.clear();
                _istream.seekg(last_pos);
                return;
            }
            _istream.seekg(after_header_pos, std::ios::beg);

            // 해당하는 스트림 버퍼에 데이터 추가
            auto it = _stream_buffers.find(header.key_hash);
            if (it != _stream_buffers.end()) {
                auto cache_it = _cache_keymap.find(header.key_hash);
                if (cache_it != _cache_keymap.end() && !cache_it->second.data.empty()) {
                    const size_t expected_size = cache_it->second.data.size();
                    if (header.data_size != expected_size) {
                        std::cerr << "Replay Error: data_size mismatch for key_hash=" << header.key_hash
                                  << " (expected=" << expected_size << ", got=" << header.data_size << ")" << std::endl;
                        _istream.clear();
                        _istream.seekg(last_pos);
                        return;
                    }
                }
                LogEntryView item;
                item.start_tick = header.start_tick;
                item.end_tick = header.end_tick;
                item.key_hash = header.key_hash;
                item.type_hash = header.type_hash;
                item.data.resize(header.data_size);
                _istream.read(item.data.data(), header.data_size);
                it->second.push_back(std::move(item));
            } else {
                _istream.seekg(header.data_size, std::ios::cur); // 무시할 키
            }
        }
    }

    bool FileBlackbox::get_initial_checkpoint(std::vector<uint8_t>& out) {
        if (_initial_checkpoint.empty()) return false;
        out.resize(_initial_checkpoint.size());
        memcpy(out.data(), _initial_checkpoint.data(), _initial_checkpoint.size());
        return true;
    }

    bool FileBlackbox::is_replay_finished(uint64_t current_global_tick) const {
        if (_mode != Mode::REPLAY) return false;
        if (!_has_tick_offset) return false;
        
        // Replay finished when current_global_tick has passed the last recorded tick
        // getLastTick() returns file-relative tick; map to global tick via offset
        uint64_t last_global_tick = getLastTick() + _tick_offset;
        return current_global_tick >= last_global_tick;
    }

    void FileBlackbox::writer_thread_func() {
        // 1. 파일 포인터를 메타데이터 바로 다음, 즉 데이터가 시작될 위치로 이동
        if (_ostream.is_open()) {
            _ostream.seekp(_data_start_offset);
        } else {
            std::cerr << "[Blackbox ERROR] Writer thread started but output file is not open." << std::endl;
            return;
        }

        // 2. 파일 I/O 효율을 위한 로컬 쓰기 버퍼 선언
        const size_t WRITE_BUFFER_SIZE = 4 * 1024 * 1024; // 4MB 버퍼
        std::vector<char> write_buffer(WRITE_BUFFER_SIZE);
        size_t buffer_offset = 0;

        // 주기적인 flush를 위한 시간 관리
        auto last_flush_time = std::chrono::steady_clock::now();
        const auto flush_interval = std::chrono::seconds(1);

        // 3. 메인 루프: 중단 신호가 오거나, 큐가 빌 때까지 계속 실행
        while (!_stop_writer_thread.load() || !_record_queue.empty()) {
            RecordWorkItem* item = nullptr;
            bool work_done = false;

            // 4. 큐에서 작업 꺼내오기
            if (_record_queue.pop(item)) {
                work_done = true;
                
                const size_t header_size = sizeof(LogEntryHeader);
                const size_t data_size = item->data.size();
                const size_t total_entry_size = header_size + data_size;

                // 5. 매우 큰 단일 엔트리는 버퍼를 넘길 수 있으므로 직접 기록
                if (total_entry_size > WRITE_BUFFER_SIZE) {
                    if (buffer_offset > 0) {
                        _ostream.write(write_buffer.data(), buffer_offset);
                        buffer_offset = 0;
                    }

                    LogEntryHeader header = {item->start_tick, item->end_tick, item->key_hash, item->type_hash, (uint32_t)data_size};
                    _ostream.write(reinterpret_cast<const char*>(&header), header_size);
                    _ostream.write(item->data.data(), data_size);
                    last_flush_time = std::chrono::steady_clock::now();
                } else {
                    // 6. 로컬 쓰기 버퍼가 꽉 찼으면 파일에 flush
                    if (buffer_offset > 0 && buffer_offset + total_entry_size > WRITE_BUFFER_SIZE) {
                        _ostream.write(write_buffer.data(), buffer_offset);
                        buffer_offset = 0;
                        last_flush_time = std::chrono::steady_clock::now();
                    }

                    // 7. 로컬 쓰기 버퍼에 데이터 기록 (헤더 + 실제 데이터)
                    LogEntryHeader header = {item->start_tick, item->end_tick, item->key_hash, item->type_hash, (uint32_t)data_size};
                    
                    memcpy(write_buffer.data() + buffer_offset, &header, header_size);
                    buffer_offset += header_size;
                    
                    memcpy(write_buffer.data() + buffer_offset, item->data.data(), data_size);
                    buffer_offset += data_size;
                }
                
                // 마지막으로 성공적으로 기록된 tick 갱신
                _last_written_tick = std::max(item->start_tick, _last_written_tick);
                
                // ✅ Return item to pool for reuse (instead of delete)
                release_work_item(item);
            }

            // 7. 일정 시간이 지났거나, 루프가 끝나가는데 버퍼에 데이터가 남아있다면 flush
            auto now = std::chrono::steady_clock::now();
            bool should_flush = (buffer_offset > 0) && (now - last_flush_time > flush_interval);
            // [중요] 스레드 종료 직전, 큐는 비었지만 버퍼는 찬 경우를 위한 조건
            bool final_flush = (buffer_offset > 0) && _stop_writer_thread.load() && _record_queue.empty();

            if (should_flush || final_flush) {
                _ostream.write(write_buffer.data(), buffer_offset);
                buffer_offset = 0;
                last_flush_time = now;
            }

            // 8. 처리할 작업이 없었다면, CPU 자원을 낭비하지 않도록 잠시 대기
            if (!work_done) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // [중요] 루프가 완전히 끝난 후, 버퍼에 남아있을 수 있는 마지막 데이터를 최종적으로 flush
        if (buffer_offset > 0) {
            _ostream.write(write_buffer.data(), buffer_offset);
        }
    }
} // namespace rtfw::blackbox