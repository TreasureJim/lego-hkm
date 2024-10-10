#pragma once

#include "IMotion.hpp"
#include "kinematics.h"
#include "pathfinding.hpp"
#include <Eigen/Dense>
#include <atomic>

Eigen::Vector3d joint_angle_to_cart_loc(agile_pkm_model& model, const double angles[4]);

class Robot {
protected:
    PathFinding pathfinding;
    Eigen::Vector3d cart_pos;
    agile_pkm_model& model;

    Eigen::Vector3d joint_angle_to_cart_loc(const double angles[4]);

    virtual int robot_setup() = 0;
    virtual void robot_shutdown() = 0;
    virtual int go_to(Eigen::Vector3d pos) = 0;

public:
    virtual ~Robot() = default;

    bool error = false;

    Robot(agile_pkm_model& model);

    agile_pkm_model& get_model();

    virtual Eigen::Vector3d get_current_cart_loc() = 0;
    // int move_joint(double joints[4]);

    int execute_motion(IMotion& motion, float interval_size = 0.05);
};

class LegoRobot : public Robot {
    int robot_setup() override;
    void robot_shutdown() override;
    int go_to(Eigen::Vector3d pos) override;

public:
    LegoRobot(agile_pkm_model& model);
    ~LegoRobot();
    
    Eigen::Vector3d get_current_cart_loc() override;
};

class FakeVisRobot : public Robot {
    int port = 4445;
    int s_socket;
    int vis_conn;
    struct chan_encoder * encoder;

    int fake_delay = 0.0;

    Eigen::Vector3d current_loc = Eigen::Vector3d(945, 906, -30.5);

    int robot_setup() override;
    void robot_shutdown() override;

public:
    FakeVisRobot(int fake_delay = 50, int port = 4445, Eigen::Vector3d starting_pos = Eigen::Vector3d(945, 906, -30.5));
    ~FakeVisRobot();

    int go_to(Eigen::Vector3d pos) override;

    Eigen::Vector3d get_current_cart_loc() override;
};

extern std::atomic_bool stop_robot_thread;
void robot_thread_func(Robot* robot);
