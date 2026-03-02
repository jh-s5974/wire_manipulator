// rtfw_connect/shm_connector.h
#pragma once

#include <string>
#include <stdexcept>
#include <cstddef>


namespace rtfw::connect {
    class SharedMemoryConnector {
    public:
        SharedMemoryConnector();
        ~SharedMemoryConnector();

        // 복사 및 이동 금지
        SharedMemoryConnector(const SharedMemoryConnector&) = delete;
        SharedMemoryConnector& operator=(const SharedMemoryConnector&) = delete;

        /**
         * @brief 이미 존재하는 공유 메모리에 읽기/쓰기 모드로 연결합니다.
         * @param shm_name 연결할 공유 메모리 이름
         * @param out_size (선택적) 연결된 공유 메모리의 실제 크기를 반환받을 포인터
         * @return 성공 시 메모리의 베이스 포인터, 실패 시 nullptr 반환
         */
        void* connect(const char* shm_name, size_t* out_size = nullptr);

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