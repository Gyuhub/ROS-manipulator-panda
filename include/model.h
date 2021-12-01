#pragma once
#ifndef __MODEL_H__
#define __MODEL_H__
#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/Body.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class Model
{
public:
    RigidBodyDynamics::Model _model;
    Eigen::VectorXd _q, _qdot; // joint angle and joint angular velocity of the model
    Eigen::Vector3d _pos, _posdot, _ori, _oridot; // position and orientation of the model
    Eigen::VectorXd _bg, _b, _g; // b = Coriolis and centrifugal force, g = gravitational force, bg = b + g 
    Eigen::MatrixXd _A, _J, _J_des; // inertial matrix and jacobian matrix of the model
    Eigen::Matrix3d _R; // rotation matrix of end-effector of the model

    Model();
    virtual ~Model();
    void getModel();
    void updateKinematics(VectorXd &q, VectorXd &qdot);
    void updateDynamics();
    void getJacobian();
    void getState();
    double getDOFs();
    Eigen::Vector3d getDesiredPositionFromJointAngle(VectorXd q);
    Eigen::Vector3d getDesiredOrientationFromJointAngle(VectorXd q);
    Eigen::MatrixXd getDesiredJacobianFromJointAngle(VectorXd q);

private:
    void configurateBody();
    void initialize();
    
    Eigen::VectorXd _body_point_local_ee; // translation of the end-effector calculated from the final joint coordinate
    Eigen::VectorXd _zero; // 3x1 zero column vector

    int _dofs;
    int _ee_id;
    bool _bool_update_kinemtaics, _bool_update_dynamics, _bool_get_state, _bool_get_jacobian;
};


#endif