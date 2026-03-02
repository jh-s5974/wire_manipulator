// rtfw_common/log_format.h
#pragma once

#include <cstdint>
#include <atomic>
#include <vector>

namespace rtfw::common {

    // ======================================================================
    //          로깅 및 리플레이를 위한 파일 포맷 정의
    // ======================================================================

    enum class LogLevel : uint8_t { 
        TRACE = 0, // spdlog::level::trace
        DEBUG = 1, // spdlog::level::debug
        INFO = 2,  // spdlog::level::info
        WARN = 3,  // spdlog::level::warn
        ERROR = 4, // spdlog::level::err
        CRITICAL = 5 // spdlog::level::critical
    };

    #pragma pack(push, 1) // 구조체 패딩 비활성화

    /**
     * @brief 로그 파일의 맨 처음에 위치하는 헤더.
     * 파일의 유효성과 버전 정보를 담는다.
     */
    struct FileHeader {
        // 파일 시그니처 ("RTFA" - Real-Time Framework Archive)
        uint8_t magic[4] = {'R', 'T', 'F', 'A'};

        // 로그 포맷 버전. 상위/하위 바이트로 주/부 버전 표현 가능.
        // 예: 0x0100 -> v1.0
        uint16_t format_version = 0x0100;
        uint8_t reserved[2] = {0}; // 정렬을 위해 2바이트로 변경
        uint64_t last_tick = 0;
        uint32_t base_frequency = 0;

        // (옵션) 로깅 시작 시점의 Unix 타임스탬프 (나노초)
        uint64_t start_timestamp_ns = 0;
        
        // (옵션) 메타데이터 블록의 시작 위치 (파일 오프셋)
        // 파일 끝에 메타데이터를 저장할 경우 유용.
        uint64_t metadata_offset = 0;
    };

    static_assert(sizeof(FileHeader) == 36, "FileHeader size mismatch!");

    /**
     * @brief 각 로그 데이터 항목의 앞에 붙는 헤더.
     * 데이터의 종류, 시점, 크기를 기술한다.
     */
    struct LogEntryHeader {
        uint64_t start_tick;
        uint64_t end_tick;
        uint64_t key_hash;
        uint64_t type_hash;
        uint32_t data_size;
    };

    static_assert(sizeof(LogEntryHeader) == 36, "LogEntryHeader size mismatch!");

    struct LogEntryView {
        uint64_t start_tick;
        uint64_t end_tick;
        uint64_t key_hash;
        uint64_t type_hash;
        std::vector<char> data;
    };

    /**
     * @brief (옵션) 파일 끝에 저장될 수 있는 메타데이터 항목.
     * key_hash와 원본 문자열 키를 매핑한다.
     */
    // struct MetadataEntry {
    //     uint64_t key_hash;
    //     uint8_t key_length; // 문자열 길이 (최대 255)
    //     // 뒤이어 key_length 만큼의 문자열 데이터가 온다.
    //     // char key_string[key_length]; 
    // };

    // 메타데이터 항목
    struct KeyMappingEntry {
        uint64_t key_hash;
        uint8_t key_length; // 문자열 길이 (최대 255)
        // 뒤이어 key_length 만큼의 문자열 데이터(key)가 옵니다.
        // char key_string[key_length];
    };

    // 파일 끝 메타데이터 영역의 시작을 알리는 헤더
    struct MetadataHeader {
        // 시그니처 ("RTMD" - Real-Time Metadata)
        uint8_t magic[4] = {'R', 'T', 'M', 'D'};
        uint32_t entry_count; // KeyMappingEntry의 개수
        uint64_t total_metadata_size; // 이 헤더를 포함한 메타데이터 영역의 총 크기
    };

    struct RecordWorkItem {
        uint64_t global_tick;
        uint64_t key_hash;
        uint64_t type_hash; // log_format.h에 추가 필요
        size_t data_offset_in_pool;
        uint32_t data_size;
    };

    #pragma pack(pop) // 기본 패딩으로 복원

    // ... (BufferMetadata 등 다른 정의들) ...

    // Special reserved key hash values for file internal records
    static constexpr uint64_t CHECKPOINT_KEY_HASH = 0xFFFFFFFFFFFFFFFFULL;
}