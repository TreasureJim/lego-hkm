#pragma once

#include "IMotion.hpp"
#include "kinematics.h"
#include <Eigen/Dense>
#include <atomic>

Eigen::Vector3d joint_angle_to_cart_loc(agile_pkm_model& model, const double angles[4]);

class Robot {
protected:
    Eigen::Vector3d cart_pos;
    agile_pkm_model* model;

    Eigen::Vector3d joint_angle_to_cart_loc(const double angles[4]);

    virtual int robot_setup() = 0;
    virtual void robot_shutdown() = 0;
    virtual int go_to(Eigen::Vector3d pos) = 0;

public:
    virtual ~Robot() = default;

    bool error = false;

    Robot(agile_pkm_model* model);

    agile_pkm_model* get_model();

    virtual Eigen::Vector3d get_current_cart_loc() = 0;
    virtual std::array<double, 4> get_current_joint_angles() = 0;

    int execute_motion(IMotion& motion, float interval_size = 0.05);
};

class LegoRobot : public Robot {
    int robot_setup() override;
    void robot_shutdown() override;
    int go_to(Eigen::Vector3d pos) override;

public:
    LegoRobot(agile_pkm_model* model);
    ~LegoRobot();
    
    Eigen::Vector3d get_current_cart_loc() override;
    std::array<double, 4> get_current_joint_angles() override;
};

class FakeVisRobot : public Robot {
    int port = 4445;
    int s_socket;
    int vis_conn;
    struct chan_encoder * encoder;

    // The amount of time to move 1mm in ms
    double fake_delay = 1;

    Eigen::Vector3d current_loc;

    int robot_setup() override;
    void robot_shutdown() override;

public:
    FakeVisRobot(double fake_delay = 1.0, int port = 4445, Eigen::Vector3d starting_pos = Eigen::Vector3d(0.945, 0.906, -0.0305));
    ~FakeVisRobot();

    int go_to(Eigen::Vector3d pos) override;

    Eigen::Vector3d get_current_cart_loc() override;
    std::array<double, 4> get_current_joint_angles() override;
};

extern std::atomic_bool stop_robot_thread;
void robot_thread_func(Robot* robot);
