#pragma once

#include "IMotion.hpp"
#include "pathfinding.hpp"
#include <Eigen/Dense>

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
    bool error = false;

    Robot(agile_pkm_model& model);

    Eigen::Vector3d get_current_cart_loc();
    // int move_joint(double joints[4]);

    int execute_motion(IMotion& motion, float interval_size = 0.05);
};

class LegoRobot : Robot {
    int robot_setup() override;
    void robot_shutdown() override;
    int go_to(Eigen::Vector3d pos) override;

public:

    LegoRobot(agile_pkm_model& model);
    ~LegoRobot();
};

class FakeVisRobot : Robot {
    int port;
    int s_socket;
    int vis_conn;
    struct chan_encoder * encoder;

    int fake_delay = 0.0;

    int robot_setup();
    void robot_shutdown();
    int go_to(Eigen::Vector3d pos);

public:
    FakeVisRobot(int port, int fake_delay);
    ~FakeVisRobot();
};

void robot_thread_func();
