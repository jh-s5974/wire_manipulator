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


bool solve_dare(const Eigen::MatrixXd& Ad, const Eigen::MatrixXd& Bd, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, Eigen::MatrixXd& P, double tol=1e-6, int max_itr=1000) {
  Eigen::MatrixXd new_P;
  for (auto itr=0; itr<max_itr; itr++) {
    new_P = Ad.transpose()*P*Ad - Ad.transpose()*P*Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad + Q;
    double diff = fabs((new_P - P).maxCoeff());
    P = new_P;
    if (diff < tol) {
      return true;
    }
  }

  return false;
}

template<int nx, int nu>
bool solve_dare_static(const Eigen::Matrix<double, nx, nx>& Ad, const Eigen::Matrix<double, nx, nu>& Bd, const Eigen::Matrix<double, nx, nx>& Q, const Eigen::Matrix<double, nu, nu>& R, Eigen::Matrix<double, nx, nx>& P, double tol=1e-6, int max_itr=1000) {
  Eigen::Matrix<double, nx, nx> new_P;
  for (auto itr=0; itr<max_itr; itr++) {
    new_P = Ad.transpose()*P*Ad - Ad.transpose()*P*Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad + Q;
    double diff = fabs((new_P - P).maxCoeff());
    P = new_P;
    if (diff < tol) {
      return true;
    }
  }

  return false;
}

void dft(double* dst, double* src, int len) {
  double p = -2.0*M_PI/len;

  for (auto i=0; i<len; i++) {
    auto& tgt = dst[i];
    tgt = 0;
    for (auto j=0; j<len; j++) {
      tgt += src[j] * std::sin(p*i*j);
    }
  }
}

double dft_freq(int idx, int len, double fs) {
  double w = 2*M_PI*fs / len;
  return idx * w;
}

