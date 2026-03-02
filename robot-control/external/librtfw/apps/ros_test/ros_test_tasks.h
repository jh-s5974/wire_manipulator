#pragma once

#include "rtfw/task.h"
#include "biped_data_types.h"
#include <iostream>
#include <cmath> // for std::sin
#include "spdlog/spdlog.h"

using namespace rtfw::rt;

// --- 1000Hz RT Tasks ---
class SensorProducer : public ITask {
public:
    const char* getName() const override { return "SensorProducer"; }

    inline void execute(void*) override {
        HighFreqSensors data;
        double time = getExecutionLocalTick() / 1000.0;
        for (int i = 0; i < 12; ++i) {
            data.joint_positions[i] = std::sin(time * 2 * M_PI + i);
            data.joint_velocities[i] = std::cos(time * 2 * M_PI + i) * 2 * M_PI;
        }
        sensors_writer_.write(data);
    }

private:
    DataWriter<HighFreqSensors> sensors_writer_{"sensors.high_freq", ArchiveOption::Enable};
};

class TorqueController : public ITask {
public:
    const char* getName() const override { return "TorqueController"; }

    inline void execute(void*) override {
        sensors_reader_.on_update([this](const HighFreqSensors& sensors) {
            if (state_reader_) {
                const RobotState& state = *state_reader_;

                MotorTorques data;
                for (int i = 0; i < 12; ++i) {
                    double p_term = (0.0 - sensors.joint_positions[i]) * 10.0;
                    double d_term = (0.0 - sensors.joint_velocities[i]) * 0.1;
                    data.joint_torques[i] = p_term + d_term;
                }
                torque_writer_.write(data);
            } else {
                // RobotState 데이터가 아직 준비되지 않은 경우
            }
        });
    }

private:
    DataReader<HighFreqSensors> sensors_reader_{"sensors.high_freq", DependencyType::Strong};
    DataReader<RobotState> state_reader_{"robot.state.estimated", DependencyType::Weak};
    DataWriter<MotorTorques> torque_writer_{"motor.torques.target", ArchiveOption::Enable};
};


// --- 200Hz RT Tasks ---
class StateEstimator : public ITask {
public:
    const char* getName() const override { return "StateEstimator"; }

    inline void execute(void*) override {
        if (!sensors_reader_) return;
        const auto& sensors = sensors_reader_.read();

        RobotState data;
        data.center_of_mass[0] = sensors.joint_positions[0] * 0.1;
        data.center_of_mass[1] = sensors.joint_positions[1] * 0.1;
        data.is_stable = std::abs(data.center_of_mass[0]) < 0.05;
        state_writer_.write(data);
    }

private:
    DataReader<HighFreqSensors> sensors_reader_{"sensors.high_freq", DependencyType::Weak};
    DataReader<WalkingGait> gait_reader_{"robot.gait.planned", DependencyType::Weak};
    DataWriter<RobotState> state_writer_{"robot.state.estimated"};
};


// --- 30Hz RT Tasks ---
class VisionProcessor : public ITask {
public:
    const char* getName() const override { return "VisionProcessor"; }

    inline void execute(void*) override {
        VisionObject data;
        data.id = 1;
        data.position_in_world[0] = 1.0 + std::sin(getExecutionLocalTick() / 30.0);
        vision_writer_.write(data);
    }

private:
    DataWriter<VisionObject> vision_writer_{"vision.object.detected"};
};

class GaitPlanner : public ITask {
public:
    const char* getName() const override { return "GaitPlanner"; }

    inline void execute(void*) override {
        WalkingGait data;
        double time = getExecutionLocalTick() / 30.0;
        data.step_length = 0.2 + 0.05 * std::sin(time);
        data.step_height = 0.1 + 0.02 * std::cos(time);
        data.step_frequency = 1.0 + 0.2 * std::sin(time * 0.5);
        gait_writer_.write(data);
    }

private:
    DataWriter<WalkingGait> gait_writer_{"robot.gait.planned"};
};
