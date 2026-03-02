#pragma once

#include <rtfw/task.h>
#include "interfaces/ImuData.h"
#include <eigen3/Eigen/Dense>
#include "util.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <boost/circular_buffer.hpp>

using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

    class ImuReader : public ITask {
    private:
        enum class State {
            OPERATING,
            RECOVERING
        };

    public:
        const char* getName() const override { return "ImuReader"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_state);
            r.add_dependency(dw_imu);
            r.add_dependency(p_port);
        }

        void execute() override {
            switch (state_) {
                case State::OPERATING: {
                    if (fd < 0) {
                        state_ = State::RECOVERING;
                        return;
                    }
                    
                    ParseStatus status = try_read_and_parse();

                    if (status == ParseStatus::PROCESSED_VALID || status == ParseStatus::PROCESSED_CORRUPT) {
                        if (write(fd, "*", 1) < 0) {
                            getLogger()->warn("[{}] Failed to write. Entering recovery mode.", getName());
                            serial_close();
                            state_ = State::RECOVERING;
                        }
                    }
                    // status가 NO_FRAME_FOUND인 경우, 아무것도 하지 않고 다음 주기를 기다린다.
                    break;
                }
                case State::RECOVERING: {
                    if (fd > 0) serial_close(); // 만약을 위해 한번 더 close
                    
                    serial_open(); // [변경] RECOVERING 상태의 유일한 임무: open 시도
                    
                    if (fd > 0) {
                        getLogger()->info("[{}] Reconnection successful. Sending first request.", getName());
                        // [중요] 연결 성공 후 즉시 다음 tick을 위한 요청을 보내, 
                        // OPERATING 상태가 바로 read를 시도할 수 있도록 함
                        if (write(fd, "*", 1) < 0) {
                            serial_close();
                            // 쓰기 실패 시 RECOVERING 상태 유지
                        } else {
                            state_ = State::OPERATING; // 쓰기 성공 시 정상 상태로 복귀
                        }
                    }
                    // open 실패 시, 아무것도 하지 않고 다음 주기에서 다시 시도
                    break;
                }
            }
            dw_state.write(fd > 0);
        }

    private:
        DataWriter<bool> dw_state{"imu_state", ArchiveOption::Enable};
        DataWriter<custom_types::ImuData> dw_imu{"imu", ArchiveOption::Enable};
        Parameter<std::string> p_port{"param.imu_if.port", "/dev/ttyUSB0"};

    private:
        State state_ = State::RECOVERING; // 항상 복구 시도 상태에서 시작
        int fd = -1;
        boost::circular_buffer<char> ring_buffer_{1024}; 
    
    private:
        void serial_open() {
            // O_NONBLOCK 플래그를 사용하여 read 함수가 즉시 리턴하도록 설정
            fd = open(p_port.read().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd < 0) {
                // 에러 발생 시, 다음 주기에 다시 시도하게 됨
                PERIODIC_CALL(
                    getLogger()->warn("[{}] {} open failed: {}", getName(), p_port.read(), strerror(errno));
                , 3s);
                return;
            }

            struct termios options;
            tcgetattr(fd, &options);

            // Baudrate, 8N1 등 표준 설정
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            options.c_cflag &= ~CSIZE;
            options.c_cflag |= CS8;
            options.c_cflag |= (CLOCAL | CREAD);
            
            // Raw Input을 위한 설정
            options.c_iflag &= ~(IXON | IXOFF | IXANY);
            options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
            options.c_oflag &= ~OPOST;

            // VMIN = 0, VTIME = 0: read()가 논블로킹으로 동작하는 핵심 설정.
            // 읽을 데이터가 없으면 즉시 0을 반환하며 리턴.
            options.c_cc[VMIN] = 0;
            options.c_cc[VTIME] = 0;

            tcsetattr(fd, TCSANOW, &options);
            getLogger()->info("[{}] Serial Port Opened: {}", getName(), p_port.read());
        }

        void serial_close() {
            if (fd > 0) { close(fd); getLogger()->info("[{}] Serial port connection closed", getName()); }
            fd = -1;
        }

        enum class ParseStatus {
            NO_FRAME_FOUND,     // 미완성 프레임이거나 버퍼가 비어있어 아무 작업도 하지 않음
            PROCESSED_VALID,    // 유효한 프레임을 최소 하나 이상 성공적으로 처리함
            PROCESSED_CORRUPT   // 유효한 프레임은 없었지만, 손상된 프레임을 발견하고 버퍼에서 제거함
        };

        bool parse_frame(const std::string& line, double q[4], double g[3], double a[3]) {
            int result = sscanf(line.c_str(), "*%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                                &q[0], &q[1], &q[2], &q[3],
                                &g[0], &g[1], &g[2],
                                &a[0], &a[1], &a[2]);
            return (result == 10);
        }

        void write_imu_data(const double q[4], const double g[3], const double a[3]) {
            custom_types::ImuData imu_data;
            imu_data.orient.w() = q[3];
            imu_data.orient.x() = q[2];
            imu_data.orient.y() = q[1];
            imu_data.orient.z() = q[0];
            imu_data.orient.normalize();
            for(int i = 0; i < 3; ++i) imu_data.gyro[i] = g[i];
            for(int i = 0; i < 3; ++i) imu_data.accel[i] = a[i];
            
            imu_data.gyro *= (M_PI / 180.0);
            imu_data.accel *= 9.80665;
            
            dw_imu.write(imu_data);

            PERIODIC_CALL(
                getLogger()->info("[{}] gyro=[{:.02f}, {:.02f}, {:.02f}]", getName(), imu_data.gyro.x(), imu_data.gyro.y(), imu_data.gyro.z());
                getLogger()->info("[{}] accel=[{:.02f}, {:.02f}, {:.02f}]", getName(), imu_data.accel.x(), imu_data.accel.y(), imu_data.accel.z());
            , 1s);
        }
        
        ParseStatus try_read_and_parse() {
            char read_buf[256];
            int bytes_read = read(fd, read_buf, sizeof(read_buf));

            if (bytes_read > 0) {
                ring_buffer_.insert(ring_buffer_.end(), read_buf, read_buf + bytes_read);
            } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                getLogger()->warn("[{}] Serial read error: {}", getName(), strerror(errno));
                serial_close();
                return ParseStatus::NO_FRAME_FOUND;
            }

            bool buffer_consumed = false; // 버퍼에서 무언가 제거되었는지 추적하는 플래그
            int successful_parses = 0;
            double last_q[4], last_g[3], last_a[3]; // 마지막으로 성공한 데이터를 저장할 변수

            while (true) {
                auto it_sof = std::find(ring_buffer_.begin(), ring_buffer_.end(), '*');
                if (it_sof == ring_buffer_.end()) break;

                if (it_sof != ring_buffer_.begin()) {
                    ring_buffer_.erase_begin(std::distance(ring_buffer_.begin(), it_sof));
                    buffer_consumed = true;
                    continue;
                }

                auto it_eof = std::find(ring_buffer_.begin() + 1, ring_buffer_.end(), '\n');
                if (it_eof == ring_buffer_.end()) break;

                std::string line(ring_buffer_.begin(), it_eof + 1);
                
                double q[4], g[3], a[3];
                if (parse_frame(line, q, g, a)) {
                    successful_parses++;
                    std::copy(q, q + 4, last_q);
                    std::copy(g, g + 3, last_g);
                    std::copy(a, a + 3, last_a);
                } else {
                    getLogger()->warn("[{}] Discarding corrupted frame: '{}'", getName(), line.substr(0, line.length() - 1));
                }

                ring_buffer_.erase_begin(std::distance(ring_buffer_.begin(), it_eof) + 1);
                buffer_consumed = true; // 유효하든 손상됐든, 프레임을 처리(소비)했음을 표시
            }


            // 루프가 끝난 후, 이번 주기에 성공적으로 파싱된 프레임이 하나라도 있었다면,
            // 가장 마지막에 성공했던 데이터만 시스템에 최종 반영.
            if (successful_parses > 0) {
                write_imu_data(last_q, last_g, last_a);
                return ParseStatus::PROCESSED_VALID;
            }

            if (buffer_consumed) {
                // 성공한 프레임은 없지만, 손상된 데이터라도 처리했다면
                return ParseStatus::PROCESSED_CORRUPT;
            }

            // 버퍼를 소비하지도, 성공하지도 않았다면 (미완성 프레임만 있는 경우)
            return ParseStatus::NO_FRAME_FOUND;
        }
    };
};