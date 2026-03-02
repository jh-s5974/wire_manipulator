#pragma once

#include <string>
#include <stdexcept>
#include <cstddef>


namespace rtfw::internal {
    class SharedMemoryAllocator {
    public:
        SharedMemoryAllocator();
        ~SharedMemoryAllocator();

        // 복사 및 이동 금지
        SharedMemoryAllocator(const SharedMemoryAllocator&) = delete;
        SharedMemoryAllocator& operator=(const SharedMemoryAllocator&) = delete;

        /**
         * @brief 주어진 이름과 크기로 공유 메모리를 생성하고 매핑합니다.
         *        이미 존재하는 경우, 기존 것을 삭제하고 새로 생성합니다.
         * @param shm_name 공유 메모리 이름 (e.g., "/rt_framework_shm")
         * @param total_size 할당할 총 바이트 크기
         * @return 성공 시 할당된 메모리의 베이스 포인터, 실패 시 예외 발생
         */
        void* allocate(const char* shm_name, size_t total_size);

        /**
         * @brief 메모리 매핑을 해제하고, 파일 디스크립터를 닫습니다.
         *        자신이 생성한 공유 메모리인 경우에만 시스템에서 객체를 삭제합니다.
         */
        void cleanup();

        void* getBasePtr() const { return _shm_base_ptr; }
        size_t getSize() const { return _shm_size; }
        bool isAttached() const { return _shm_base_ptr != nullptr; }

    private:
        std::string _shm_name;
        int _shm_fd;
        void* _shm_base_ptr;
        size_t _shm_size;
    };
};