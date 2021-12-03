#include "model.h"

Model::Model()
{
    initialize();
}

Model::~Model()
{
}

void Model::getModel()
{
    bool bool_get_model = RigidBodyDynamics::Addons::URDFReadFromFile("/home/kist/KIST-Dual-Arm-ROS/src/ROS-manipulator-panda/model/franka_panda.urdf", &_model, false, true);
    if (bool_get_model)
    {
        _dofs = _model.dof_count;
        cout << "Successfully get model! the DoFs of the model is " << _dofs << '\n';
    }
    else
    {
        cout << "Fail to get model!! Please get model first...\n";
    }
}

void Model::updateKinematics(VectorXd &q, VectorXd &qdot)
{
    _q = q;
    _qdot = qdot;
    
    if (_dofs != 0)
    {
        RigidBodyDynamics::UpdateKinematicsCustom(_model, &_q, &_qdot, NULL);
        _bool_update_kinemtaics = true;
    }
    else
    {
        _bool_update_kinemtaics = false;
        cout << "Fault!! the DoFs of the robot is zero!!Please get correct model first...\n";
    }
}

void Model::updateDynamics()
{
    if (_bool_update_kinemtaics)
    {
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(_model, _q, _A, false);
        RigidBodyDynamics::InverseDynamics(_model, _q, _zero, _zero, _g, NULL);
        RigidBodyDynamics::InverseDynamics(_model, _q, _qdot, _zero, _bg, NULL);
        _b = _bg - _g;
        _bool_update_dynamics = true;
    }
    else
    {
        _bool_update_dynamics = false;
        cout << "Fault!! Kinematics is not ready yet! Please update the Kinematics first...\n";
    }
}

void Model::getJacobian()
{
    if (_bool_update_kinemtaics)
    {
        MatrixXd J_;
        J_.setZero(6, _dofs);
        RigidBodyDynamics::CalcPointJacobian6D(_model, _q, _ee_id, _body_point_local_ee, J_, false);
        _J.block<3, 9>(0, 0) = J_.block<3, 9>(3, 0);
        _J.block<3, 9>(3, 0) = J_.block<3, 9>(0, 0);
        _bool_get_jacobian = true;
    }
    else
    {
        _bool_get_jacobian = false;
        cout << "Fault!! Can't get the states of the robot! Please update the Kinematics first...\n";
    }
}

void Model::getState()
{
    if (_bool_update_dynamics)
    {
        _pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, _q, _ee_id, _body_point_local_ee, false);
        _R = RigidBodyDynamics::CalcBodyWorldOrientation(_model, _q, _ee_id, false).transpose();
        _ori = _R.eulerAngles(0, 1, 2); // XYZ euler angle rotation

        Eigen::VectorXd xdot_(6);
        xdot_ = _J * _qdot;
        _posdot = xdot_.head(3);
        _oridot = xdot_.tail(3);
        _bool_get_state = true;
    }
    else
    {
        _bool_get_state = false;
        cout << "Fault!! Dynamics is not ready yet! Please update the Dynamics first...\n";
    }
}

double Model::getDOFs()
{
    return _dofs;
}

Eigen::Vector3d Model::getDesiredPositionFromJointAngle(VectorXd q)
{
    Eigen::Vector3d pos_des_;
    pos_des_.setZero();
    pos_des_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(_model, q, _ee_id, _body_point_local_ee, true); // change false to true
    return pos_des_;
}

Eigen::Vector3d Model::getDesiredOrientationFromJointAngle(VectorXd q)
{
    Eigen::Matrix3d R_;
    Eigen::Vector3d ori_des_;
    R_.setZero();
    ori_des_.setZero();
    R_ = RigidBodyDynamics::CalcBodyWorldOrientation(_model, q, _ee_id, true).transpose(); // change false to true
    ori_des_ = R_.eulerAngles(0, 1, 2);
    return ori_des_;
}

Eigen::MatrixXd Model::getDesiredJacobianFromJointAngle(VectorXd q)
{
    MatrixXd J_des_;
    J_des_.setZero(6, _dofs);
    RigidBodyDynamics::CalcPointJacobian6D(_model, q, _ee_id, _body_point_local_ee, _J_des, true); // change false to true
    J_des_.block<3, 9>(0, 0) = _J_des.block<3, 9>(3, 0);
    J_des_.block<3, 9>(3, 0) = _J_des.block<3, 9>(0, 0);
    return J_des_;
}

void Model::configurateBody()
{
    _body_point_local_ee(0) = 0.0;
    _body_point_local_ee(1) = 0.0;
    _body_point_local_ee(2) = 0.107;
}

void Model::initialize()
{
    _dofs = 0;
    _ee_id = 7;
    _bool_update_kinemtaics = false;
    _bool_update_dynamics = false;
    _bool_get_state = false;
    _bool_get_jacobian = false;

    getModel();
    _q.setZero(_dofs);
    _qdot.setZero(_dofs);

    _pos.setZero();
    _posdot.setZero();
    _ori.setZero();
    _oridot.setZero();

    _bg.setZero(_dofs);
    _b.setZero(_dofs);
    _g.setZero(_dofs);

    _body_point_local_ee.setZero(3);

    _zero.setZero(_dofs);

    _A.setZero(_dofs, _dofs);
    _J.setZero(6, _dofs);
    _J_des.setZero(6, _dofs);

    _R.setZero();
    configurateBody();
}