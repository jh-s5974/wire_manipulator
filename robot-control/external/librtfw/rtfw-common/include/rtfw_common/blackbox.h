// IBlackbox.h

#pragma once
#include <string>
#include <map>
#include <memory> // std::shared_ptr
#include <vector>
#include <atomic>



namespace rtfw {
    class RealTimeFramework;
};

namespace rtfw {
    namespace blackbox {
        /**
         * @brief Blackbox의 키맵에 저장될 데이터 슬롯.
         *        하나의 키(key_hash)에 대한 최신 데이터를 담고 있으며,
         *        DataWriter(게시자)와 ReplayHook(구독자)이 이 슬롯을 통해 상호작용합니다.
         */
        struct CacheSlot {
            std::atomic<uint64_t> start_tick{0};
            std::atomic<uint64_t> end_tick{0};
            uint64_t type_hash;
            std::vector<char> data;
            std::atomic<bool> update{false};
            // Sequence counter for lock-free snapshotting (seqlock style).
            // Even: stable. Odd: writer in progress.
            std::atomic<uint32_t> seq{0};
        };

        enum class Mode { NONE, RECORD, REPLAY };

        class IBlackbox {
        public:
            virtual ~IBlackbox() = default;

            // --- 설정 API (변경 없음) ---
            virtual bool start(std::string path, Mode mode) = 0;
            virtual void shutdown() = 0;

            // --- 유틸리티 API ---
            virtual Mode getMode() const = 0;
            virtual size_t getLastTick() const = 0;
            virtual uint32_t frequency() const = 0;
            virtual std::string getKeyName(uint64_t keyhash) const = 0;
            // ...
        private:
            friend class rtfw::RealTimeFramework;
            virtual bool initialize_metadata(const std::map<uint64_t, std::string>& target_keys,
                                             uint32_t base_frequency,
                                             const std::map<uint64_t, size_t>& key_sizes = {},
                                             const std::vector<uint8_t>& checkpoint = {},
                                             uint64_t start_tick = 0) = 0;
            // ...

            // --- [핵심] 키맵(Keymap) 제공 API ---
            /**
             * @brief Simulation 모드에서, 재현할 데이터 키들에 대한 캐시 슬롯 맵을 반환합니다.
             *        프레임워크는 초기화 시 이 맵에 대한 포인터를 각 DataWriter에 전달합니다.
             * @return key_hash를 키로, BlackboxCacheSlot에 대한 포인터를 값으로 갖는 맵.
             */
            virtual CacheSlot* getCacheSlot(uint64_t keyhash) = 0;


            // --- [핵심] 스케줄러 루프 연동 API ---
            /**
             * @brief 스케줄러 루프가 매 Tick마다 호출하는 메인 업데이트 함수.
             * @param current_global_tick 현재 프레임워크의 Global Tick.
             */
            virtual void onTick(uint64_t current_global_tick) = 0;
            // If the recording being opened for replay contained an initial checkpoint
            // (written with CHECKPOINT_KEY_HASH), this API will return it.
            virtual bool get_initial_checkpoint(std::vector<uint8_t>& out) { (void)out; return false; }
            // When replaying a file that was recorded at a different global tick
            // epoch, the framework can instruct the backend to apply an integer
            // tick offset so that file ticks map to framework global ticks:
            //   global_tick == file_tick + tick_offset
            virtual void set_tick_offset(int64_t offset) { (void)offset; }
            // Check if replay has finished: current tick has passed the last recorded tick
            virtual bool is_replay_finished(uint64_t current_global_tick) const { (void)current_global_tick; return false; }
            

        };
    };
};