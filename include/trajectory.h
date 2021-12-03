#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "control_math.h"

using namespace std;
using namespace Eigen;

class Trajectory
{
    void initialize(int dofs, double dt);
public:
    Trajectory();
    ~Trajectory();
    void setDefault(int dofs, double dt);
    void setStart(VectorXd x_start, VectorXd xdot_start, double start_time);
    void setGoal(VectorXd x_goal, VectorXd xdot_goal, double goal_time);
    void checkSize(VectorXd x, int control_mode);
    void updateTime(double time);
    void resetTarget();
    void getCmdCount(int cmd_count);
    bool isTrajFinished();
    int getControlMode();

    Eigen::VectorXd getPositionTrajectory();
    Eigen::VectorXd getOrientationTrajectory();
private:
    int _dofs, _size;
    int _control_mode, _cmd_count;
    double _dt, _time;
    double _start_time, _goal_time;

    Eigen::VectorXd _x_start, _x_goal;
    Eigen::VectorXd _xdot_start, _xdot_goal;
    Eigen::VectorXd _bool_cmd;
};