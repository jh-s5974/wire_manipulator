#pragma once
#include "eigen3/Eigen/Dense"


double lpf_gain(double sampling_freq, double cutoff_freq) {
    return std::exp(-2*M_PI*cutoff_freq/sampling_freq);
}

struct LPF {
    double z;
    double alpha;

    void set_gain(double gain) {
        alpha = gain;
    }

    void set_freq(double fc, double fs) {
        alpha = exp(-2*M_PI*fc/fs);
    }

    double& update(double signal) {
        z = z * alpha + signal * (1-alpha);
        return z;
    }

    double& reset(double signal) {
        z = signal;
        alpha = 0;
        return z;
    }

    void reset() {
        z = 0;
        alpha = 0;
    }
};

template<int N>
struct VectorLPF {
    Eigen::Vector<double, N> z;
    Eigen::Vector<double, N> alpha;

    void set_gain(double gain, int ch=-1) {
        if (ch < 0)
            alpha.setConstant(gain);
        if (ch >= 0 && ch < N)
            alpha[ch] = gain;
    }

    void set_freq(double fc, double fs, int ch=-1) {
        double gain = exp(-2*M_PI*fc/fs);
        if (ch < 0)
            alpha.setConstant(gain);
        if (ch >= 0 && ch < N)
            alpha[ch] = gain;
    }

    Eigen::Vector<double, N>& update(const Eigen::Vector<double, N>& signal) {
        z = z.array() * alpha.array() + signal.array() * (1-alpha.array());
        return z;
    }

    Eigen::Vector<double, N>& reset(const Eigen::Vector<double, N>& signal) {
        z = signal;
        alpha.setZero();
        return z;
    }

    void reset() {
        z.setZero();
        alpha.setZero();
    }
};