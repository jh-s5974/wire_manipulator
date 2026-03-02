// rtfw_common/type_utils.h
#pragma once

#include <string>
#include <any>



namespace rtfw::common {
    // 이 템플릿 함수는 컴파일러의 도움을 받아 타입 T에 대한
    // 고유하고 안정적인 시그니처 문자열을 반환합니다.
    template<typename T>
    inline const char* get_stable_type_signature() {
    #if defined(_MSC_VER)
        // MSVC 예시: "const char *__cdecl get_stable_type_signature<struct MyNamespace::MyType>(void)"
        return __FUNCSIG__;
    #elif defined(__GNUC__) || defined(__clang__)
        // GCC/Clang 예시: "const char* get_stable_type_signature() [with T = MyNamespace::MyType]"
        return __PRETTY_FUNCTION__;
    #else
        // 다른 컴파일러의 경우, typeid().name()을 최후의 수단으로 사용
        return typeid(T).name();
    #endif
    }


    // // --- SFINAE 헬퍼: 사용자가 convert 함수를 정의했는지 컴파일 타임에 감지 ---
    // template <typename, typename, typename = std::void_t<>>
    // struct has_convert_function : std::false_type {};

    // template <typename BaseType, typename ExportType>
    // struct has_convert_function<BaseType, ExportType, 
    //     std::void_t<decltype(convert(std::declval<const BaseType&>(), std::declval<ExportType&>()))>>
    //     : std::true_type {};

    // // --- 사용자가 특수화할 메인 traits 구조체 ---
    // template<typename BaseType, typename ExportType>
    // struct adapter_traits {
    //     // 기본적으로, convert 함수가 존재하면 변환 가능하다고 판단.
    //     // 사용자는 특정 경우에 이 값을 false로 오버라이드하여 변환을 막을 수 있음.
    //     static constexpr bool is_convertible = has_convert_function<BaseType, ExportType>::value;
    // };

};