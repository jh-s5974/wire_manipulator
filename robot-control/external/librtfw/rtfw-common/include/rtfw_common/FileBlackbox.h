// FileBlackbox.h
#include "rtfw_common/blackbox.h"
#include "rtfw_common/log_format.h"
#include <fstream>
#include <thread>
#include <deque>
#include <unordered_map>
#include <boost/lockfree/spsc_queue.hpp>
#include <boost/lockfree/stack.hpp>


namespace rtfw {
    namespace blackbox {

        struct RecordWorkItem {
            uint64_t start_tick;
            uint64_t end_tick;
            uint64_t key_hash;
            uint64_t type_hash;
            std::vector<char> data;
        };
        
        class FileBlackbox: public IBlackbox {
        public:
            FileBlackbox(size_t queue_capacity = 16384);
            ~FileBlackbox();

            bool start(std::string path, Mode mode) override;
            void shutdown() override;

            Mode getMode() const override;
            size_t getLastTick() const override;
            uint32_t frequency() const override;
            std::string getKeyName(uint64_t keyhash) const override;

            bool initialize_metadata(const std::map<uint64_t, std::string>& target_keys,
                                     uint32_t base_frequency,
                                     const std::map<uint64_t, size_t>& key_sizes = {},
                                     const std::vector<uint8_t>& checkpoint = {},
                                     uint64_t start_tick = 0) override;
            void onTick(uint64_t current_global_tick) override;
            CacheSlot* getCacheSlot(uint64_t keyhash) override;
            std::unordered_map<uint64_t, CacheSlot>* getKeymapCache() {return &_cache_keymap;}

            bool is_open() const { return _istream.is_open(); };
            std::string get_filepath() const {return _filepath; };
            std::map<uint64_t, std::string> getKeyMapping() const {return _keyname; };
            // Check if replay has finished: current tick has passed the last recorded tick
            bool is_replay_finished(uint64_t current_global_tick) const;
            // Getter for tick offset (used for replay finished check)
            int64_t get_tick_offset() const { return _tick_offset; }

        private:
            friend class rtfw::RealTimeFramework;

            void writer_thread_func();
            void scan_and_fill_buffers();
            
            // Object pool management for RT-safe recording
            void initialize_record_pool(size_t pool_size, size_t max_data_size);
            void cleanup_record_pool();
            RecordWorkItem* acquire_work_item();
            void release_work_item(RecordWorkItem* item);
        
            Mode _mode;
            std::string _filepath;
            std::unordered_map<uint64_t, CacheSlot> _cache_keymap;
            std::map<uint64_t, std::deque<common::LogEntryView>> _stream_buffers;
            std::map<uint64_t, std::string> _keyname;
            std::ofstream _ostream;
            std::ifstream _istream;
            common::FileHeader _file_header;
            common::MetadataHeader _meta_header;
            std::vector<common::KeyMappingEntry> _entry_headers;
            std::streamoff _data_start_offset;
            uint64_t _last_written_tick;
            // If a checkpoint record exists in the file, it is cached here during initialize_metadata
            std::vector<char> _initial_checkpoint;

            // --- 비동기 Recording을 위한 멤버 ---
            std::thread _writer_thread;
            std::atomic<bool> _stop_writer_thread{false};
            
            size_t _record_queue_capacity;  // Store queue capacity for pool sizing
            boost::lockfree::spsc_queue<RecordWorkItem*> _record_queue;
            
            // --- Object pool for RT-safe recording ---
            std::vector<RecordWorkItem> _work_item_pool;  // Pre-allocated pool storage
            boost::lockfree::stack<RecordWorkItem*> _free_items;  // Lock-free free list
            size_t _pool_size{0};
            std::atomic<size_t> _pool_alloc_failures{0};  // Statistics
            // Tick mapping for replay: file_tick + _tick_offset == global_tick
            int64_t _tick_offset{0};
            bool _has_tick_offset{false};
            // (removed) earliest tick observed in opened file
            // checkpoint tick read from file (if present)
            uint64_t _file_checkpoint_tick{0};
            // When recording dynamically, store the global tick that corresponds
            // to file tick 0. Recorded entries will have their ticks written
            // relative to this base.
            uint64_t _file_base_tick{0};
            bool _has_file_base{false};
            // Flag to track when replay has finished
            mutable bool _replay_finished{false};
        public:
            // Return initial checkpoint if available (used by framework to restore task states)
            bool get_initial_checkpoint(std::vector<uint8_t>& out);
            // configure mapping between file ticks and framework global ticks
            void set_tick_offset(int64_t offset) { _tick_offset = offset; _has_tick_offset = true; }
        };
    };
};