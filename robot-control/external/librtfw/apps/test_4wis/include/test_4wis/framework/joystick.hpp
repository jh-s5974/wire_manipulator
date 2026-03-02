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

#include "test_4wis/frame.hpp"
#include "test_4wis/framework/signal.hpp"
#include "test_4wis/framework/util.hpp"

using namespace std::chrono_literals;
using namespace project;


class JoystickNode
{
  public:
    struct {
        struct {
            signal::Tx<bool> state;
            signal::Tx<double> axis[8];
            signal::Tx<bool> btn[12];
        } tx;
        struct {
            signal::Rx<bool>::SharedPtr state;
        } rx;
    } signal;

    std::string port;
    JoystickNode(std::string js="/dev/input/js0"): port(js) {
      signal.rx.state = std::make_shared<signal::Rx<bool>>([&](const bool& data) {
        if (data) {
            if (!active) {
                active = true;
                std::cout << "Joy: state=on" << std::endl;
            }
        } else {
            if (active) {
                active = false;
                std::cout << "Joy: state=off" << std::endl;
            }
        }
      });
            
      std::make_shared<std::thread>([&](){
        while(run) {
            std::this_thread::sleep_for(500ms);
            signal.tx.state.send(fd > 0);
        }
        printf("joystick: thread finished\n");
      })->detach();

      auto task = std::thread(&JoystickNode::joy_task, this);
      task.detach();

    }

    ~JoystickNode() {
      run = false;
      joy_close();
      printf("joystick: terminated\n");
    }

  private:
    void joy_task() {

      while(run) {
        if (fd > 0)
          joy_read();
        else{
          std::this_thread::sleep_for(1s);
          joy_open();
        }
      }

    }
    void joy_open() {
      fd = open(port.c_str(), O_RDONLY);
      if (fd < 0) {
        // RCLCPP_ERROR(this->get_logger(), "%s open failed", port.c_str());
        return;
      }


        int version;
        ioctl(fd, JSIOCGVERSION, &version);
        ioctl(fd, JSIOCGAXES, &numAxis);
        ioctl(fd, JSIOCGBUTTONS, &numButton);
        ioctl(fd, JSIOCGNAME(80), &nameJoy);


        std::cout << "Version: " << version << std::endl;
        std::cout << "Joy Connect: " << nameJoy << "(" << numAxis << ", " << numButton << ")" << std::endl;

        fcntl(fd, F_SETFL, O_NONBLOCK);	// use non-blocking methods
        init_time = std::chrono::steady_clock::now();
    }

    void joy_close() {
      if (fd > 0) {
        close(fd);
        std::cout << "joystick connection closed" << std::endl;
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
        }
        else
          switch(event.type & ~JS_EVENT_INIT) {
          case JS_EVENT_AXIS:
              if(event.number < 8) {
              //   std::cout << "js axis=" << (int)event.number << " value=" << event.value/32767.0 << std::endl;
                signal.tx.axis[event.number].send(event.value/32767.0);
              }
              break;
          case JS_EVENT_BUTTON:
              if(event.number < 12) {
              //   std::cout << "js sw" << (int)event.number << " value=" << event.value << std::endl;
                signal.tx.btn[event.number].send(event.value);
              }
              break;
          }
      } else if (rxc < 0) {
        if (errno == EAGAIN) {
          std::this_thread::sleep_for(10ms);
        } else {
          printf("read error=%d\n", errno);
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

    std::shared_ptr<std::thread> hb;

    bool active = false;
    bool run = true;
    std::chrono::steady_clock::time_point init_time;
};
