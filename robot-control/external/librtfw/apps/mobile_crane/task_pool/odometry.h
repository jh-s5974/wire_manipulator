#pragma once

#include <rtfw/task.h>
#include <manif/manif.h>

using namespace rtfw;
using namespace rtfw::rt;


namespace task_pool {

    class Odometry : public ITask {
    public:
        Odometry(double dt=0.01, bool use_imu=false) {
            param.dt = dt;
            param.use_imu = use_imu;
            reset();
        }

        const char* getName() const override { return "Odometry"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dr_imu);
            r.add_dependency(dr_act_vel);
            r.add_dependency(dr_reset);
            r.add_dependency(dw_state);
            r.add_dependency(dw_est_vel);
            r.add_dependency(dw_position);
        }
        void execute() override {
            dr_reset.on_update([this]() {
                reset();
            });

            dr_act_vel.on_update([this](const manif::SE3Tangentd& data) {
                act_vel.x() = data.lin().x();
                act_vel.y() = data.lin().y();
                act_vel.z() = data.ang().z();
                
                est_vel = act_vel;

                manif::SE2Tangentd vel(data.lin().x(), data.lin().y(), data.ang().z());
                    
                update(vel, param.dt);
                manif::SE3Tangentd msg;
                msg.lin().x() = est_vel.x();
                msg.lin().y() = est_vel.y();
                msg.ang().z() = est_vel.z();
                dw_est_vel.write(msg);
                dw_position.write(pos);                


                ipos[0] += (cos(ipos[2])*act_vel[0]-sin(ipos[2])*act_vel[1])*param.dt;
                ipos[1] += (sin(ipos[2])*act_vel[0]+cos(ipos[2])*act_vel[1])*param.dt;
                ipos[2] += act_vel[2]*param.dt;

                Eigen::AngleAxisd ori(orient);
                PERIODIC_CALL(
                    getLogger()->info("[{}] x={:.03f}, y={:.03f}, angle={:.03f}", getName(), pos.x(), pos.y(), pos.angle()*180.0/M_PI);
                , 1s);

            });

            dr_imu.on_update([this](const Eigen::Quaterniond& data) {
                orient = data;
            });

            dw_state.write(true);
        }
    private:
        DataReader<Eigen::Quaterniond> dr_imu{"imu_orient"};
        DataReader<manif::SE3Tangentd> dr_act_vel{"velocity_pv"};
        DataReader<Signal> dr_reset{"odom_reset"};
        DataWriter<bool> dw_state{"od_state"};
        DataWriter<manif::SE3Tangentd> dw_est_vel{"est_velocity_pv"};
        DataWriter<manif::SE2d> dw_position{"odometry"};
        // DataWriter<Eigen::Vector3d> dw_position{"odometry"};

    private:
        struct {
            bool use_imu;
            double dt;
        } param;

    private:
        void reset() {
            pos.setIdentity();
            l_orient = orient;
        }

        manif::SE2d update(manif::SE2Tangentd vel, double dt) {
            pos = pos + vel*dt;
            return pos;
        }

        bool ori_init = false;
        Eigen::Quaterniond l_orient;
        Eigen::Quaterniond orient;
        manif::SE2d pos;
        // Eigen::Vector3d pos;
        Eigen::Vector3d ipos;
        Eigen::Vector3d act_vel;
        Eigen::Vector3d est_vel;
    };
};