#pragma once

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <cmath>


#define PERIODIC_CALL(exec, peroid) {     \
                static std::chrono::steady_clock::time_point l_stamp = std::chrono::steady_clock::now();      \
                if (std::chrono::steady_clock::now() >= l_stamp + peroid) {     \
                  exec;     \
                  l_stamp = std::chrono::steady_clock::now();     \
                }   \
              }

