// rtfw_common/exporter_app.h
#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include "rtfw_common/FileBlackbox.h"
#include "rtfw_common/type_utils.h" // get_persistent_type_hash()
#include "json.hpp"      // JSON 라이브러리

using json = nlohmann::json;

namespace rtfw {
namespace common {



// // C++ 구조체를 json으로 변환하기 위한 nlohmann/json의 표준 방식
// // (이 부분은 my_robot_types.h 또는 별도의 변환 헤더에 위치)
// void to_json(json& j, const MotorCommand& p) {
//     j = json{{"target_torque", p.target_torque}, {"control_mode", p.control_mode}};
// }
// // ... 다른 타입들에 대한 to_json 함수 ...

class JsonSerializerFactory {
public:
    /**
     * @brief 지원할 타입을 팩토리에 등록합니다.
     * 내부적으로 T -> type_hash 변환 및 json 변환 람다를 생성하여 맵에 저장합니다.
     */
    template<typename T>
    void register_type() {
        uint64_t type_hash = std::hash<std::string_view>{}(get_stable_type_signature<T>());
        _converters[type_hash] = [](const void* data, size_t size) -> json {
            if (sizeof(T) != size) {
                return "Error: Size mismatch";
            }
            // void*를 실제 타입 포인터로 캐스팅하고 json으로 변환
            return *static_cast<const T*>(data);
        };
    }

    /**
     * @brief type_hash와 원본 데이터를 기반으로 json 객체를 생성합니다.
     */
    json convert(uint64_t type_hash, const void* data, size_t size) const {
        auto it = _converters.find(type_hash);
        if (it != _converters.end()) {
            return it->second(data, size); // 등록된 변환 함수 호출
        }
        return "Unsupported Type"; // 지원하지 않는 타입
    }

private:
    std::map<uint64_t, std::function<json(const void*, size_t)>> _converters;
};


// 2. Exporter 애플리케이션의 핵심 로직 (보일러플레이트)
class ExporterApp {
public:
    int run(int argc, char* argv[]) {
        if (argc != 3) {
            std::cerr << "Usage: " << argv[0] << " <input.rttrace> <output.json>" << std::endl;
            return 1;
        }
        std::string input_path = argv[1];
        std::string output_path = argv[2];

        // [핵심] 팩토리 설정은 자식 클래스에게 위임 (가상 함수)
        register_domain_types(_factory);

        rtfw::blackbox::FileBlackbox blackbox;
        std::map<uint64_t, std::vector<LogEntryView>> data_cache;

        bool is_load = false;
        try {
            blackbox.start(input_path, rtfw::blackbox::Mode::REPLAY);
            is_load = blackbox.initialize_metadata(std::map<uint64_t, std::string>(), 0);
            data_cache.clear();
            auto slots = blackbox.getKeymapCache();
            for (auto i=0; i <= blackbox.getLastTick(); i++) {
                blackbox.onTick(i);
                std::vector<LogEntryView> views;
                for (auto& it: *slots) {
                    if (!it.second.update)
                        continue;
                    it.second.update = false;

                    views.push_back({
                        it.second.start_tick,
                        it.second.end_tick,
                        it.first,
                        it.second.type_hash,
                        it.second.data
                    });
                }
                data_cache.insert({i, views});                
            }
        } catch (...) {
            blackbox.shutdown();
        }

        if (!is_load) {
            std::cout << "data load failed" << std::endl;
            return -1;
        }

        // 3. 메타데이터와 데이터를 JSON 객체로 수집
        auto key_map = blackbox.getKeyMapping();
        json root_json; // 최종 결과를 담을 JSON 객체

        std::vector<std::string> keys;
        for (const auto& it: key_map) {
            keys.push_back(it.second);
        }
        root_json["frequency"] = blackbox.frequency();
        root_json["keymap"] = keys;
        root_json["last_tick"] = blackbox.getLastTick();
        root_json["data"] = {};
        for (const auto& it: key_map) {
            root_json["data"][it.second] = std::vector<json>();
        }

        for (auto i=0; i<=blackbox.getLastTick(); i++) {
            for (auto& it: data_cache[i]) {
                json entry;
                entry["raw"] = _factory.convert(it.type_hash, it.data.data(), it.data.size());
                entry["start_tick"] = it.start_tick;
                entry["end_tick"] = it.end_tick;
                root_json["data"][key_map[it.key_hash]].push_back(entry);
            }
        }
        blackbox.shutdown();

        // 4. 최종 JSON을 파일에 쓰기
        std::ofstream o(output_path);
        o << root_json.dump(4); // 4칸 들여쓰기로 예쁘게 출력
        o.close();
        
        std::cout << "Successfully exported " << input_path << " to " << output_path << std::endl;
        return 0;
    }

protected:
    // ** 개발자가 구현해야 할 유일한 부분 **
    // 순수 가상 함수로 만들어, 파생 클래스가 반드시 구현하도록 강제합니다.
    virtual void register_domain_types(JsonSerializerFactory& factory) = 0;

private:
    JsonSerializerFactory _factory;
};

} // namespace common
} // namespace rtfw