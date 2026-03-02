// rtfw-common/parameter_utils.h (신규 파일 제안)

#pragma once
#include "shm_utils.h"
#include <cstring>
#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"


namespace rtfw::common {

    inline constexpr size_t kParamVectorMaxElements = 64;
    inline constexpr size_t kParamStringMaxBytes = 1024;
    inline constexpr size_t kParamVectorStringElementMaxBytes = 128;

    template<typename T>
    struct param_layout {
    static constexpr size_t size = sizeof(T);
    static constexpr size_t alignment = alignof(T);
    };

    template<>
    struct param_layout<std::string> {
    static constexpr size_t size = kParamStringMaxBytes;      // cap
    static constexpr size_t alignment = alignof(char);
    };

    template<>
    struct param_layout<std::vector<int>> {
        static constexpr size_t size = sizeof(uint32_t) + kParamVectorMaxElements * sizeof(int);
        static constexpr size_t alignment = alignof(int);
    };

    template<>
    struct param_layout<std::vector<double>> {
        static constexpr size_t size = sizeof(uint32_t) + kParamVectorMaxElements * sizeof(double);
        static constexpr size_t alignment = alignof(double);
    };

    template<>
    struct param_layout<std::vector<std::string>> {
        static constexpr size_t size = sizeof(uint32_t) + kParamVectorMaxElements * kParamVectorStringElementMaxBytes;
        static constexpr size_t alignment = alignof(char);
    };


    template<typename T>
    static inline bool write_parameter(void* shm_base_ptr, const ParameterInfo* info, const T& value) {
        if (!shm_base_ptr || !info) return false;
        if (sizeof(T) != info->data_size) return false;

        auto* shm_header = static_cast<SharedMemoryHeader*>(shm_base_ptr);
        uint64_t current_tick = shm_header->framework_tick_count.load(std::memory_order_acquire);


        auto* p_block_header = get_param_block_header(shm_base_ptr);
        if (!p_block_header) return false;

        // 1. Working 버퍼 인덱스 계산
        int stable_idx = p_block_header->stable_index.load(std::memory_order_acquire);
        int working_idx = 1 - stable_idx;
        
        // 2. Working 버퍼의 주소 가져오기
        char* working_buffer = get_param_data_buffer(shm_base_ptr, working_idx);
        if (!working_buffer) return false;

        // 3. 값 쓰기 (memcpy)
        memcpy(working_buffer + info->offset_in_buffer, &value, sizeof(T));
        
        // 4. 버전 번호 증가 (가장 중요!)
        p_block_header->version[working_idx].store(current_tick + 1, std::memory_order_release);
        
        return true;
    };


    static inline bool write_parameter(void* shm_base_ptr, const ParameterInfo* info, void* value, size_t size) {
        if (!shm_base_ptr || !info) return false;
        if (size != info->data_size) return false;

        auto* shm_header = static_cast<SharedMemoryHeader*>(shm_base_ptr);
        uint64_t current_tick = shm_header->framework_tick_count.load(std::memory_order_acquire);


        auto* p_block_header = get_param_block_header(shm_base_ptr);
        if (!p_block_header) return false;

        // 1. Working 버퍼 인덱스 계산
        int stable_idx = p_block_header->stable_index.load(std::memory_order_acquire);
        int working_idx = 1 - stable_idx;
        
        // 2. Working 버퍼의 주소 가져오기
        char* working_buffer = get_param_data_buffer(shm_base_ptr, working_idx);
        if (!working_buffer) return false;

        // 3. 값 쓰기 (memcpy)
        memcpy(working_buffer + info->offset_in_buffer, value, size);
        
        // 4. 버전 번호 증가 (가장 중요!)
        p_block_header->version[working_idx].store(current_tick + 1, std::memory_order_release);
        
        return true;
    };


    template<typename T, typename Enable = void>
    struct param_codec;

    // 2-1) trivial 타입: memcpy 기반
    template<typename T>
    struct param_codec<T, std::enable_if_t<std::is_trivially_copyable_v<T>>> {
        static void encode(const T& v, uint8_t* dst, size_t cap) {
            (void)cap; std::memcpy(dst, &v, sizeof(T));
        }
        static T decode(const uint8_t* src, size_t cap) {
            (void)cap; T v; std::memcpy(&v, src, sizeof(T)); return v;
        }
        static T parse_yaml(const YAML::Node& n) { return n.as<T>(); }
        static bool write(void* base, const common::ParameterInfo* info, const T& v) {
            return common::write_parameter<T>(base, info, v); // 타입 안전 경로 :contentReference[oaicite:3]{index=3}
        }
    };

    template<>
    struct param_codec<std::string, void> {
        static constexpr size_t cap = common::param_layout<std::string>::size;

        static void encode(const std::string& s, uint8_t* dst, size_t) {
            const size_t L = std::min(s.size(), cap ? cap - 1 : 0);
            if (L) std::memcpy(dst, s.data(), L);
            if (cap) dst[L] = '\0';
            if (cap > L + 1) std::memset(dst + L + 1, 0, cap - (L + 1));
        }
        static std::string decode(const uint8_t* src, size_t) {
            size_t L = 0; while (L < cap && src[L] != '\0') ++L;
            return std::string(reinterpret_cast<const char*>(src), L);
        }
        static std::string parse_yaml(const YAML::Node& n) { return n.as<std::string>(); }
        static bool write(void* base, const common::ParameterInfo* info, const std::string& s) {
            std::array<uint8_t, cap> buf{}; encode(s, buf.data(), buf.size());
            // 바이너리 경로 사용 (size 일치 체크 포함) :contentReference[oaicite:4]{index=4}
            return common::write_parameter(base, info, buf.data(), buf.size());
        }
    };

    template<>
    struct param_codec<std::vector<int>, void> {
        static constexpr size_t max_count = kParamVectorMaxElements;
        static constexpr size_t cap = common::param_layout<std::vector<int>>::size;

        static void encode(const std::vector<int>& values, uint8_t* dst, size_t) {
            const uint32_t count = static_cast<uint32_t>(std::min(values.size(), max_count));
            std::memset(dst, 0, cap);
            std::memcpy(dst, &count, sizeof(uint32_t));
            if (count > 0) {
                std::memcpy(dst + sizeof(uint32_t), values.data(), count * sizeof(int));
            }
        }

        static std::vector<int> decode(const uint8_t* src, size_t) {
            uint32_t count = 0;
            std::memcpy(&count, src, sizeof(uint32_t));
            count = std::min<uint32_t>(count, static_cast<uint32_t>(max_count));
            std::vector<int> out(count);
            if (count > 0) {
                std::memcpy(out.data(), src + sizeof(uint32_t), count * sizeof(int));
            }
            return out;
        }

        static std::vector<int> parse_yaml(const YAML::Node& n) {
            if (n.IsSequence()) return n.as<std::vector<int>>();
            if (n.IsScalar()) return {n.as<int>()};
            throw YAML::BadConversion(n.Mark());
        }

        static bool write(void* base, const common::ParameterInfo* info, const std::vector<int>& values) {
            std::array<uint8_t, cap> buf{};
            encode(values, buf.data(), buf.size());
            return common::write_parameter(base, info, buf.data(), buf.size());
        }
    };

    template<>
    struct param_codec<std::vector<double>, void> {
        static constexpr size_t max_count = kParamVectorMaxElements;
        static constexpr size_t cap = common::param_layout<std::vector<double>>::size;

        static void encode(const std::vector<double>& values, uint8_t* dst, size_t) {
            const uint32_t count = static_cast<uint32_t>(std::min(values.size(), max_count));
            std::memset(dst, 0, cap);
            std::memcpy(dst, &count, sizeof(uint32_t));
            if (count > 0) {
                std::memcpy(dst + sizeof(uint32_t), values.data(), count * sizeof(double));
            }
        }

        static std::vector<double> decode(const uint8_t* src, size_t) {
            uint32_t count = 0;
            std::memcpy(&count, src, sizeof(uint32_t));
            count = std::min<uint32_t>(count, static_cast<uint32_t>(max_count));
            std::vector<double> out(count);
            if (count > 0) {
                std::memcpy(out.data(), src + sizeof(uint32_t), count * sizeof(double));
            }
            return out;
        }

        static std::vector<double> parse_yaml(const YAML::Node& n) {
            if (n.IsSequence()) return n.as<std::vector<double>>();
            if (n.IsScalar()) return {n.as<double>()};
            throw YAML::BadConversion(n.Mark());
        }

        static bool write(void* base, const common::ParameterInfo* info, const std::vector<double>& values) {
            std::array<uint8_t, cap> buf{};
            encode(values, buf.data(), buf.size());
            return common::write_parameter(base, info, buf.data(), buf.size());
        }
    };

    template<>
    struct param_codec<std::vector<std::string>, void> {
        static constexpr size_t max_count = kParamVectorMaxElements;
        static constexpr size_t str_cap = kParamVectorStringElementMaxBytes;
        static constexpr size_t cap = common::param_layout<std::vector<std::string>>::size;

        static void encode(const std::vector<std::string>& values, uint8_t* dst, size_t) {
            const uint32_t count = static_cast<uint32_t>(std::min(values.size(), max_count));
            std::memset(dst, 0, cap);
            std::memcpy(dst, &count, sizeof(uint32_t));

            uint8_t* payload = dst + sizeof(uint32_t);
            for (uint32_t index = 0; index < count; ++index) {
                uint8_t* slot = payload + static_cast<size_t>(index) * str_cap;
                const std::string& value = values[index];
                const size_t length = std::min(value.size(), str_cap > 0 ? str_cap - 1 : 0);
                if (length > 0) {
                    std::memcpy(slot, value.data(), length);
                }
                if (str_cap > 0) {
                    slot[length] = '\0';
                }
            }
        }

        static std::vector<std::string> decode(const uint8_t* src, size_t) {
            uint32_t count = 0;
            std::memcpy(&count, src, sizeof(uint32_t));
            count = std::min<uint32_t>(count, static_cast<uint32_t>(max_count));

            std::vector<std::string> out;
            out.reserve(count);
            const uint8_t* payload = src + sizeof(uint32_t);
            for (uint32_t index = 0; index < count; ++index) {
                const uint8_t* slot = payload + static_cast<size_t>(index) * str_cap;
                size_t length = 0;
                while (length < str_cap && slot[length] != '\0') {
                    ++length;
                }
                out.emplace_back(reinterpret_cast<const char*>(slot), length);
            }
            return out;
        }

        static std::vector<std::string> parse_yaml(const YAML::Node& n) {
            if (n.IsSequence()) return n.as<std::vector<std::string>>();
            if (n.IsScalar()) return {n.as<std::string>()};
            throw YAML::BadConversion(n.Mark());
        }

        static bool write(void* base, const common::ParameterInfo* info, const std::vector<std::string>& values) {
            std::array<uint8_t, cap> buf{};
            encode(values, buf.data(), buf.size());
            return common::write_parameter(base, info, buf.data(), buf.size());
        }
    };
    

};
