#pragma once

#include <rtfw/task.h>
// #include "parameters.h"


#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
// #include <libsocketcan.h>
#include <errno.h>


using namespace rtfw;
using namespace rtfw::rt;

static const int PID_REQ_PID_DATA = 4;
static const int PID_PNT_VEL_DATA = 177;
static const int PID_PNT_VEL_CMD = 207;
static const int PID_VEL_CMD = 130;
static const int PID_MONITOR = 196;
static const int PID_MONITOR2 = 201;
static const int PID_POSI_RESET = 13;


namespace task_pool {

    class WheelIF : public ITask {
    public:
        ~WheelIF() {
            motor_stop();
            if (can_so > 0) {
                close(can_so);
            }
            can_so = -1;
        }
        const char* getName() const override { return "CAN_IF"; }
        void setup(TaskRegistry& r) override {
            r.add_dependency(dw_wm_state);
            r.add_dependency(dw_wheel);
            r.add_dependency(dw_crane_spd);
            r.add_dependency(dw_crane_pos);
            r.add_dependency(dr_wheel);
            r.add_dependency(dr_crane_spd);
            r.add_dependency(dr_crane_stop);
            r.add_dependency(dr_crane_pos_reset);

            r.add_dependency(param.can_if.port);
            r.add_dependency(param.crane.gear);
            r.add_dependency(param.crane.cpr);
            r.add_dependency(param.crane.rpm_max);
            r.add_dependency(param.crane.mm_per_round);
            r.add_dependency(param.crane.inv);
            r.add_dependency(param.mecanum.gear);
            r.add_dependency(param.mecanum.rpm_max);
            r.add_dependency(param.mecanum.ppr);
        }
        void execute() override {     
            dr_crane_spd.on_update([this](const double& data) {
                crane.vel_sv = data;
                double factor = 0.2;
                crane.vc_sv = crane.vel_sv * (1-factor) + crane.vc_sv * factor;

                auto spd_sv = crane.vc_sv * 1000; // m/s -> mm/s
                crane.rpm_sv = spd_sv * param.crane.gear.read() / param.crane.mm_per_round.read() * 60;

                if (std::isnan(crane.rpm_sv) || std::isinf(crane.rpm_sv))
                crane.rpm_sv = 0;
                if (crane.rpm_sv > param.crane.rpm_max.read())
                crane.rpm_sv = param.crane.rpm_max.read();
                if (crane.rpm_sv < -param.crane.rpm_max.read())
                crane.rpm_sv = -param.crane.rpm_max.read();

                // RCLCPP_INFO(this->get_logger(), "req spd=%f mm/s", spd_sv);
                // RCLCPP_INFO(this->get_logger(), "req rpm=[%d]", rpm_sv);
            });

            dr_crane_stop.on_update([this](){
                crane.vc_sv = 0;
            });

            dr_crane_pos_reset.on_update([this](){
                uint32_t rmid = 183;
                uint32_t tmid = 184;
                uint8_t md_id = 3;
        
                {
                    struct can_frame txb;
                    txb.can_id = (rmid << 16) | (tmid << 8) | (md_id) | CAN_EFF_FLAG;
                    txb.can_dlc = 8;
                    txb.data[0] = PID_POSI_RESET;
                    send(&txb);
                }
            });
            
            dr_wheel.on_update([this](const std::array<double, 4>& data) {

                const double lpf = 0.0;
                for (auto i=0; i<4; i++) {
                    auto req = data[i];
                    auto rpm_max = param.mecanum.rpm_max.read();
                    if (req > rpm_max) req = rpm_max;
                    if (req < -rpm_max) req = -rpm_max;
                    mecanum.rpm_sv[i] = lpf*mecanum.rpm_sv[i] + (1-lpf)*req;
                    if (std::isnan(mecanum.rpm_sv[i]) || std::isinf(mecanum.rpm_sv[i])) {
                        mecanum.rpm_sv[0] = 0;
                        mecanum.rpm_sv[1] = 0;
                        mecanum.rpm_sv[2] = 0;
                        mecanum.rpm_sv[3] = 0;
                        break;
                    }
                }
            });

            if (can_so > 0) {
                while(recv() > 0);
                trns();
            } else {
                auto res = setup_can();
                if (res < 0) {
                    PERIODIC_CALL(
                        getLogger()->info("[{}] setup {} failed({})", getName(), param.can_if.port.read().c_str(), res);
                    , 1s);
                } else {
                    PERIODIC_CALL(
                        getLogger()->info("[{}] setup {} succeed", getName(), param.can_if.port.read().c_str());
                    , 1s);
                    motor_stop();
                }
            }

            dw_wm_state.write(can_so > 0);
        }
    private:
        DataWriter<bool> dw_wm_state{"wm_state", ArchiveOption::Enable};
        DataWriter<std::array<double, 4>> dw_wheel{"wheel_pv", ArchiveOption::Enable};
        DataWriter<double> dw_crane_spd{"crane_spd_pv", ArchiveOption::Enable};
        DataWriter<double> dw_crane_pos{"crane_pos_pv", ArchiveOption::Enable};

        DataReader<std::array<double, 4>> dr_wheel{"wheel_sv", DependencyType::Weak};
        DataReader<double> dr_crane_spd{"crane_spd_sv", DependencyType::Weak};
        DataReader<Signal> dr_crane_stop{"crane_stop", DependencyType::Weak};
        DataReader<Signal> dr_crane_pos_reset{"crane_pos_reset", DependencyType::Weak};

        struct {
            struct {
                Parameter<std::string> port{"param.can_if.port", "can0"};
            } can_if;
            struct {
                Parameter<double> gear{"param.crane.gear", 50};
                Parameter<double> cpr{"param.crane.cpr", 30};
                Parameter<int> rpm_max{"param.crane.rpm_max", 60};
                Parameter<double> mm_per_round{"param.crane.mm_per_round", (60+8)*3.14159265};
                Parameter<bool> inv{"param.crane.inv", true};
            } crane;
            struct {
                Parameter<double> gear{"param.mecanum.gear", 50};
                Parameter<int> rpm_max{"param.mecanum.rpm_max", 60};
                Parameter<double> ppr{"param.mecanum.ppr", 0x4000*4};
            } mecanum;
        } param;
        
    private:
        struct {
            std::array<double, 4> rpm_sv = {0};
            std::array<double, 4> rpm_pv = {0};
            std::array<double, 4> dpos_pv = {0};
            std::array<int32_t, 4> pls_pv_old = {0};
            std::array<int32_t, 4> pls_pv = {0};
        } mecanum;

        struct {
            double vel_sv = 0;
            int16_t rpm_sv = 0;
            double vc_sv = 0;
            int16_t rpm_pv = 0;
            double pos_pv = 0;
        } crane;

    private:
        int setup_can() {
            struct sockaddr_can addr;
            struct ifreq ifr;
            int so;
            if ((so = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
                return -2;

            int flags = fcntl(so, F_GETFL, 0);
            if (flags < 0) {
                close(so);
                return -3;
            }

            // NONBLOCKING 
            if (fcntl(so, F_SETFL, flags | O_NONBLOCK) < 0) {
                close(so);
                return -4;
            }

            // // or TIMEOUT
            // struct timeval timeout;
            // timeout.tv_sec = 0;
            // timeout.tv_usec = 500;
            // if (setsockopt(so, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
            //     perror("setsockopt SO_RCVTIMEO");
            //     return -4;
            // }


            strcpy(ifr.ifr_name, param.can_if.port.read().c_str());
            ioctl(so, SIOCGIFINDEX, &ifr);

            memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            if (bind(so, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                close(so);
                return -5;
            }

            // struct can_filter filters[1];
            // filters[0].can_id = 0xB8B700 | CAN_EFF_FLAG;
            // filters[0].can_mask = 0xFFFF00 | (CAN_EFF_FLAG);

            // int res = setsockopt(so, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));
            // if (res < 0) {
            //   // RCLCPP_ERROR(this->get_logger(), "can filter failed(%d)", errno);
            //   printf("can filter failed(%d)\n", errno);
            //   return -6;
            // }

            can_so = so;
            return 0;
        }

        void trns() {


            uint32_t rmid = 183;
            uint32_t tmid = 184;
            
            uint8_t md_id[3] = {1, 2, 3};

            for (auto i=0; i<2; i++) {
                struct can_frame txb;
                txb.can_id = (rmid << 16) | (tmid << 8) | (md_id[i]) | CAN_EFF_FLAG;
                txb.can_dlc = 8;
                txb.data[0] = PID_PNT_VEL_CMD;
                txb.data[1] = 1;
                (*(int16_t*)&txb.data[2]) = mecanum.rpm_sv[i]*param.mecanum.gear.read();
                txb.data[4] = 1;
                (*(int16_t*)&txb.data[5]) = mecanum.rpm_sv[i+2]*param.mecanum.gear.read();
                // return data type
                txb.data[7] = 7;
                send(&txb);
            }
            {
                int16_t data = crane.rpm_sv*param.crane.gear.read();
                struct can_frame txb;
                txb.can_id = (rmid << 16) | (tmid << 8) | (md_id[2]) | CAN_EFF_FLAG;
                txb.can_dlc = 8;
                txb.data[0] = PID_VEL_CMD;
                txb.data[1] = data & 0x00FF;
                txb.data[2] = (data >> 8);
                send(&txb);

                // txb.data[0] = PID_REQ_PID_DATA;
                // txb.data[1] = PID_MONITOR;
                // send(&txb);
            }
        }

        int recv() {
            if (can_so < 0)
                return -1;
            
            int max_fd = can_so;

            fd_set read_fds;
            FD_ZERO(&read_fds);
            FD_SET(can_so, &read_fds);

            struct timeval timeout;
            timeout.tv_sec = 0;
            timeout.tv_usec = 10;

            int ret = select(max_fd+1, &read_fds, nullptr, nullptr, &timeout);

            if (ret > 0) {
                if (FD_ISSET(can_so, &read_fds)) {
                    struct can_frame rx;
                    int res = read(can_so, &rx, sizeof(rx));
                    if (res > 0)      
                        procedure(rx);
                }
            } else if(ret == 0) {
                

            } else {
                PERIODIC_CALL(
                    getLogger()->warn("[{}] can recv error({})", getName(), errno);
                    // printf("can recv error(%d)\n", errno)
                , 1s);
            }
            
            return ret;
        }

        void send(struct can_frame* txb) {
            if (can_so < 0)
                return;
            if (write(can_so, txb, sizeof(struct can_frame)) > 0) {

            }
        }

        void procedure(struct can_frame& msg) {
            // TODO: check recv
                // std::cout << "--------------------" << std::endl;

            // // can comm test
            // std::cout << std::hex << msg.can_id << "  ";
            // for (int i = 0; i < msg.can_dlc; i++) {
            //   std::cout << std::hex << int(msg.data[i]) << " ";
            // }
            // std::cout << std::dec << std::endl;

            // struct can_frame* txb = new struct can_frame;
            // txb->can_id = 0x1234;
            // txb->can_dlc = 8;
            // txb->data[0] = 0x01;
            // txb->data[1] = 0x23;
            // txb->data[2] = 0x45;
            // txb->data[3] = 0x67;
            // txb->data[4] = 0x89;
            // txb->data[5] = 0xAB;
            // txb->data[6] = 0xCD;
            // txb->data[7] = 0xEF;
            // write(txb);

            // const int PID_REQ_PID_DATA = 4;
            // const int PID_PNT_VEL_DATA = 177;
            // const int PID_PNT_VEL_CMD = 207;


            auto rmid = (msg.can_id & 0x00ff0000) >> 16;
            auto tmid = (msg.can_id & 0x0000ff00) >> 8;
            auto id = (msg.can_id & 0x000000ff);

            typedef struct {
                bool alarm: 1;
                bool ctrl_fail: 1;
                bool over_volt: 1;
                bool over_temp: 1;
                bool over_load: 1;
                bool hall_fail: 1;
                bool inv_vel: 1;
                bool stall: 1;
            } MOT_STATUS;

            switch(msg.data[0]) {
                case PID_PNT_VEL_DATA: {
                int16_t rpm[2];
                rpm[0] = *((int16_t*)&msg.data[1]);
                rpm[1] = *((int16_t*)&msg.data[3]);
                MOT_STATUS status = (*(MOT_STATUS*)&msg.data[7]);
                switch(id) {
                    case 1: {
                        mecanum.rpm_pv[0] = rpm[0]/param.mecanum.gear.read();
                        mecanum.rpm_pv[2] = rpm[1]/param.mecanum.gear.read();
                        break;
                    }
                    case 2: {
                        mecanum.rpm_pv[1] = rpm[0]/param.mecanum.gear.read();
                        mecanum.rpm_pv[3] = rpm[1]/param.mecanum.gear.read();
                        // std::cout << "mecanum updated" << std::endl;
                        dw_wheel.write(mecanum.rpm_pv);
                        break;
                    }
                    default: {
                        getLogger()->warn("[{}] << rmid={}, tmid={}, id={}, data={:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}", getName(),
                            rmid, tmid, id, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                        // printf("wheel_if: << rmid=%d, tmid=%d, id=%d, data=%02X %02X %02X %02X %02X %02X %02X %02X\n", 
                        //     rmid, tmid, id, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                    }
                }
                break;
                }
                case PID_MONITOR: 
                case PID_MONITOR2: {

                struct {
                    uint8_t code;
                    MOT_STATUS status;
                    int16_t rpm;
                    int32_t pos;
                } packet;
                memcpy(&packet, msg.data, 8);

                switch(id) {
                    case 1: {
                    if (msg.data[0] == PID_MONITOR) {
                        mecanum.rpm_pv[0] = packet.rpm/param.mecanum.gear.read();
                        mecanum.pls_pv[0] = packet.pos;

                    } else {
                        mecanum.rpm_pv[2] = packet.rpm/param.mecanum.gear.read();
                        mecanum.pls_pv[2] = packet.pos;

                    }
                    break;
                    }
                    case 2: {
                    if (msg.data[0] == PID_MONITOR) {
                        mecanum.rpm_pv[1] = packet.rpm/param.mecanum.gear.read();
                        mecanum.pls_pv[1] = packet.pos;

                    } else {
                        mecanum.rpm_pv[3] = packet.rpm/param.mecanum.gear.read();
                        mecanum.pls_pv[3] = packet.pos;

                        if (enc_init) {
                        for (auto i=0; i<4; i++)
                            mecanum.dpos_pv[i] = (mecanum.pls_pv[i] - mecanum.pls_pv_old[i])/0.01/param.mecanum.ppr.read()*60/param.mecanum.gear.read();
        
                        // signal.tx.wheel.send(mecanum.rpm_pv);
                        dw_wheel.write(mecanum.dpos_pv);

                        // PERIODIC_CALL(
                        //   printf("rpm(pls)=%lf, %lf, %lf, %lf\n", mecanum.dpos_pv[0], mecanum.dpos_pv[1], mecanum.dpos_pv[2], mecanum.dpos_pv[3]);
                        //   printf("rpm=%lf, %lf, %lf, %lf\n", mecanum.rpm_pv[0], mecanum.rpm_pv[1], mecanum.rpm_pv[2], mecanum.rpm_pv[3]);
                        //   printf("ppr=%lf\n", mecanum.ppr);
                        // , 1s);
                        } else {
                            enc_init = true;
                        }

                        mecanum.pls_pv_old = mecanum.pls_pv;
                    }
                    break;
                    }
                    case 3: {
                        crane.rpm_pv = packet.rpm / param.crane.gear.read();
                        crane.pos_pv = packet.pos / param.crane.cpr.read() / param.crane.gear.read() * param.crane.mm_per_round.read() * 0.001;
                        dw_crane_spd.write(crane.rpm_pv);
                        dw_crane_pos.write(crane.pos_pv);
                        break;
                    }
                    default: {
                        getLogger()->warn("[{}] << rmid={}, tmid={}, id={}, data={:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}", getName(),
                            rmid, tmid, id, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                        // printf("can_if: << rmid=%d, tmid=%d, id=%d, data=%02X %02X %02X %02X %02X %02X %02X %02X\n", 
                        //     rmid, tmid, id, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                    }
                }
                break;
                }
                default: {
                    getLogger()->warn("[{}] << rmid={}, tmid={}, id={}, data={:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}", getName(),
                        rmid, tmid, id, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                    // printf("can_if: << rmid=%d, tmid=%d, id=%d, data=%02X %02X %02X %02X %02X %02X %02X %02X\n", 
                    // rmid, tmid, id, msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
                    break;
                }      
            }
        }

        void motor_stop() {
            mecanum.rpm_sv[0] = 0;
            mecanum.rpm_sv[1] = 0;
            mecanum.rpm_sv[2] = 0;
            mecanum.rpm_sv[3] = 0;
            crane.rpm_sv = 0;
            trns();
        }
        

    private:
        int can_so = -1;
        bool enc_init = false;
    };
};