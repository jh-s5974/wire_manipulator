// rtfw-client/shm_connector.cpp
#include "rtfw_connect/shm_connector.h"
#include "rtfw_common/shm_layout.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cstring> // for strerror
#include <cerrno>  // for errno


using namespace rtfw::common;
using namespace rtfw::connect;

SharedMemoryConnector::SharedMemoryConnector() 
    : _shm_fd(-1), _shm_base_ptr(nullptr), _shm_size(0) {}

SharedMemoryConnector::~SharedMemoryConnector() {
    cleanup();
}

void* SharedMemoryConnector::connect(const char* shm_name, size_t* out_size) {
    if (isAttached()) {
        cleanup();
    }

    _shm_name = shm_name;

    _shm_fd = shm_open(_shm_name.c_str(), O_RDWR, 0666);
    if (_shm_fd == -1) {
        return nullptr;
    }

    struct stat shm_stat;
    if (fstat(_shm_fd, &shm_stat) == -1) {
        close(_shm_fd);
        return nullptr;
    }
    _shm_size = shm_stat.st_size;
    if (out_size) {
        *out_size = _shm_size;
    }

    _shm_base_ptr = mmap(nullptr, _shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, _shm_fd, 0);
    if (_shm_base_ptr == MAP_FAILED) {
        close(_shm_fd);
        _shm_base_ptr = nullptr;
        return nullptr;
    }

    auto* header = static_cast<common::SharedMemoryHeader*>(_shm_base_ptr);
    common::ShmState current_state = header->shm_state.load(std::memory_order_acquire);

    if (current_state == common::ShmState::UNINITIALIZED) {
        // 아직 RTFW가 초기화조차 시작하지 않은 메모리
        cleanup(); // 연결 즉시 해제
        return nullptr;
    }

    return _shm_base_ptr;
}


void SharedMemoryConnector::cleanup() {
    if (_shm_base_ptr != nullptr && _shm_base_ptr != MAP_FAILED) {
        munmap(_shm_base_ptr, _shm_size);
        _shm_base_ptr = nullptr;
    }
    if (_shm_fd != -1) {
        close(_shm_fd);
        _shm_fd = -1;
    }
    _shm_name.clear();
    _shm_size = 0;
}