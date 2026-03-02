#pragma once

#include <rtfw/task.h>
#include "util.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <iostream>

#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <thread>


using namespace std::chrono_literals;
using namespace rtfw;
using namespace rtfw::rt;

namespace task_pool {

    class Joystick : public ITask {
    public:
        Joystick(std::string js="/dev/input/js0"): port(js) {}
        const char* getName() const override { return "Joystick"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_state);
            for (auto i=0; i<8; i++)
                r.add_dependency(dw_axis[i]);
            for (auto i=0; i<12; i++)
                r.add_dependency(dw_btn[i]);
        }

        void execute() override {
            if (fd > 0)
                joy_read();
            else
                joy_open();

            dw_state.write(fd > 0);            
        }
    private:
        DataWriter<bool> dw_state{"js_state", ArchiveOption::Enable};
        DataWriter<double> dw_axis[8] = {
            DataWriter<double>{"js/axis_0", ArchiveOption::Enable},
            DataWriter<double>{"js/axis_1", ArchiveOption::Enable},
            DataWriter<double>{"js/axis_2", ArchiveOption::Enable},
            DataWriter<double>{"js/axis_3", ArchiveOption::Enable},
            DataWriter<double>{"js/axis_4", ArchiveOption::Enable},
            DataWriter<double>{"js/axis_5", ArchiveOption::Enable},
            DataWriter<double>{"js/axis_6", ArchiveOption::Enable},
            DataWriter<double>{"js/axis_7", ArchiveOption::Enable},
        };
        DataWriter<bool> dw_btn[12] = {
            DataWriter<bool>{"js/btn_0", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_1", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_2", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_3", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_4", ArchiveOption::Disable}, // emg btn
            DataWriter<bool>{"js/btn_5", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_6", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_7", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_8", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_9", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_10", ArchiveOption::Enable},
            DataWriter<bool>{"js/btn_11", ArchiveOption::Enable},
        };
    private:
        std::string port;

    private:
        void joy_open() {
            fd = open(port.c_str(), O_RDONLY);
            if (fd < 0) {
                // RCLCPP_ERROR(this->get_logger(), "%s open failed", port.c_str());
                PERIODIC_CALL(
                    getLogger()->warn("[{}] {} open failed", getName(), port);
                    // std::cout << port << " open failed" << std::endl;
                , 3s);
                return;
            }


            int version;
            ioctl(fd, JSIOCGVERSION, &version);
            ioctl(fd, JSIOCGAXES, &numAxis);
            ioctl(fd, JSIOCGBUTTONS, &numButton);
            ioctl(fd, JSIOCGNAME(80), &nameJoy);

            getLogger()->info("Version: {}", version);
            getLogger()->info("Joy Connect: {}({}, {})", nameJoy, numAxis, numButton);
            // std::cout << "Version: " << version << std::endl;
            // std::cout << "Joy Connect: " << nameJoy << "(" << numAxis << ", " << numButton << ")" << std::endl;

            fcntl(fd, F_SETFL, O_NONBLOCK);	// use non-blocking methods
            init_time = std::chrono::steady_clock::now();
        }

        void joy_close() {
            if (fd > 0) {
                close(fd);
                getLogger()->info("[{}] joystick connection closed", getName());
                // std::cout << "joystick connection closed" << std::endl;
            }
            fd = -1;
            init = false;
        }

        void joy_read() {
            struct js_event event = {0};
            // read the joystick
            auto rxc = read(fd, &event, sizeof(event));
            if(sizeof(struct js_event) == rxc){
                if (!init) {
                    // dummy value pass
                    if (std::chrono::steady_clock::now() - init_time > 1s)
                        init = true;
                    else
                        return;
                }

                {
                    switch(event.type & ~JS_EVENT_INIT) {
                        case JS_EVENT_AXIS:
                            if(event.number < 8) {
                                // std::cout << "js axis=" << (int)event.number << " value=" << event.value/32767.0 << std::endl;
                                dw_axis[event.number].write(event.value/32767.0);
                            }
                            break;
                        case JS_EVENT_BUTTON:
                            if(event.number < 12) {
                            //   std::cout << "js sw" << (int)event.number << " value=" << event.value << std::endl;
                                dw_btn[event.number].write(event.value);
                            }
                            break;
                    }
                }
            } else if (rxc < 0) {
                if (errno == EAGAIN) {
                    // std::this_thread::sleep_for(10ms);
                } else {
                    getLogger()->warn("[{}] js read error={}", getName(), errno);
                    // printf("js read error=%d\n", errno);
                    joy_close();
                }
            }
        
        }

    private:
        int fd=-1;
        bool init = false;

        int		numAxis = 0;
        int		numButton = 0;
        char	nameJoy[128] = {0};
        std::chrono::steady_clock::time_point init_time;
    };
};