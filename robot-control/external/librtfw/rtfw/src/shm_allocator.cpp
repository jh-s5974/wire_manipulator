#include "rtfw/shm_allocator.h"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>
#include <cstring> // for strerror
#include <cerrno>  // for errno

using namespace rtfw::internal;


SharedMemoryAllocator::SharedMemoryAllocator() 
    : _shm_fd(-1), _shm_base_ptr(nullptr), _shm_size(0) {}

SharedMemoryAllocator::~SharedMemoryAllocator() {
    cleanup();
}

void* SharedMemoryAllocator::allocate(const char* shm_name, size_t total_size) {
    if (isAttached()) {
        cleanup();
    }

    _shm_name = shm_name;

    shm_unlink(_shm_name.c_str());

    _shm_fd = shm_open(_shm_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (_shm_fd == -1) {
        throw std::runtime_error("shm_open() failed in allocate: " + std::string(strerror(errno)));
    }

    if (ftruncate(_shm_fd, total_size) == -1) {
        close(_shm_fd);
        shm_unlink(_shm_name.c_str());
        throw std::runtime_error("ftruncate() failed in allocate: " + std::string(strerror(errno)));
    }

    _shm_base_ptr = mmap(nullptr, total_size, PROT_READ | PROT_WRITE, MAP_SHARED, _shm_fd, 0);
    if (_shm_base_ptr == MAP_FAILED) {
        close(_shm_fd);
        shm_unlink(_shm_name.c_str());
        throw std::runtime_error("mmap() failed in allocate: " + std::string(strerror(errno)));
    }

    _shm_size = total_size;
    return _shm_base_ptr;
}

void SharedMemoryAllocator::cleanup() {
    if (_shm_base_ptr != nullptr && _shm_base_ptr != MAP_FAILED) {
        munmap(_shm_base_ptr, _shm_size);
        _shm_base_ptr = nullptr;
    }
    if (_shm_fd != -1) {
        close(_shm_fd);
        _shm_fd = -1;
    }
    if (!_shm_name.empty()) {
        shm_unlink(_shm_name.c_str());
    }
    _shm_name.clear();
    _shm_size = 0;
}