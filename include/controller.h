#pragma once
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#include "control_math.h"
#include "model.h"
#include "trajectory.h"

class Controller{
public:
    Controller();
    ~Controller();
    void getJointsDatas(double t, double *q, double *qdot);
    void control();
    void setJointsDatas(double *tau);
    void initialize();
private:
    Model _cmodel; // dynamic model of the robot (model.h and model.cpp)
    
    int _control_mode; // control mode of the robot, 1 : gravity compensation, 2 : joint space control, 3 : task space control

    double _time, _time_pre; // current time which is obtained from the mujoco simulator
    double _dt;
    double _dofs; // degrees of the freedom
    Trajectory _jtrajectory;
    Trajectory _ctrajectory; // trajectory generator with cubic spline interpolation
    
    double _kp_t; // p gain (task)
    double _kd_t; // d gain (task)
    double _kp_j; // p gain (joint)
    double _kd_j; // d gain (joint)

    bool _bool;

    VectorXd _q, _qdot; // angle and angular velocities of joints of the panda
    VectorXd _tau; // torque of joints of the panda
    VectorXd _x, _xdot; // poses and velocities of end-effector of the panda

    VectorXd _q_des, _qdot_des; // desired joint space angle and joint angular velocity of panda
    VectorXd _q_goal, _qdot_goal; // goal joint space angle and joint angular velocity of panda
    VectorXd _x_des, _xdot_des; // desired task space position and orientation of panda
    VectorXd _x_goal, _xdot_goal; // goal position and orientation which is obtained from the planner

    Vector3d _pos_err, _posdot_err;
    Vector3d _ori_err, _oridot_err;
    VectorXd _xddot_ref;
    MatrixXd _lambda, _null_space_projection;

    MatrixXd _J, _J_T, _J_weighted_inv; // jacobian, jacobian transpose, jacobian weighted inverse matrix of the panda
    Matrix3d _R; // rotation matrix of end-effector of the panda
    MatrixXd _I; // identity matrix DoFs x DoFs

    void modelUpdate(); // Update the model of the robot using RBDL and Eigen
    void trajectoryPlan(); // planning the trajectory of end-effector of the panda

    void jointControl(); // joint space control
    void taskControl(); // task space control
    void gravityCompensation(); // gravity compensation of the all joints
};
#endif