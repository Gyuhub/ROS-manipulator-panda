#pragma once
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__
#include "control_math.h"
#include "model.h"
#include "trajectory.h"
#include "roswrapper.h"

class Controller{
public:
    Controller();
    ~Controller();

    ros::NodeHandle* _nh_ptr;

    void getNodeHandler(ros::NodeHandle* nh_ptr);

    void getJointsDatas(double t, double *q, double *qdot);
    void control();
    void setJointsDatas(double *tau, double *qpos);
    void initialize();
private:
    Model _cmodel; // dynamic model of the robot (model.h and model.cpp)
    ROSWrapper _ROSWrapper; // ROS object to handle the events for ros
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

    VectorXd _q, _qdot; // angle and angular velocities of joints of the panda
    VectorXd _tau, _qpos; // torque of joints of the panda
public:
    VectorXd _x, _xdot; // poses and velocities of end-effector of the panda
private:
    VectorXd _q_des, _qdot_des; // desired joint space angle and joint angular velocity of panda
    VectorXd _q_goal, _qdot_goal; // goal joint space angle and joint angular velocity of panda
    VectorXd _x_des, _xdot_des; // desired task space position and orientation of panda
    VectorXd _x_goal, _xdot_goal; // goal position and orientation which is obtained from the planner
    VectorXd _x_err, _xdot_err; // error of the position and orientation

    Vector3d _pos_err, _posdot_err;
    Vector3d _ori_err, _oridot_err;
    VectorXd _xddot_ref, _qdot_ref;
    MatrixXd _lambda, _null_space_projection;

    MatrixXd _J, _J_T, _J_des, _J_T_des; // jacobian, jacobian transpose matrices of the panda
    MatrixXd _T, _J_A, _J_T_A; // transformation matrix from geometric to analytic jacobian, analytic jacobian and jacobian transpose matices
    Matrix3d _R; // rotation matrix of end-effector of the panda
    MatrixXd _I_dofs; // identity matrix DoFs x DoFs
    Matrix3d _I_3, _O_3; // identity matrix 3x3 and zero matrix 3x3

    void modelUpdate(); // Update the model of the robot using RBDL and Eigen
    void rosHandle(); // handle the entire ROS routine which we want to control the panda
    void trajectoryPlan(); // planning the trajectory of end-effector of the panda

    void jointControl(); // joint space control
    void taskControl(); // task space control
    void gravityCompensation(); // gravity compensation of the all joints
};
#endif