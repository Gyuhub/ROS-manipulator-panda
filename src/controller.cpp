#include "controller.h"

Controller::Controller()
{
    initialize();
    _jtrajectory.setDefault(_dofs, _dt);
    _ctrajectory.setDefault(_dofs, _dt);
}

Controller::~Controller()
{
}

void Controller::getNodeHandler(ros::NodeHandle* nh_ptr)
{
    _nh_ptr = nh_ptr;
    _ROSWrapper.getNodeHandler(_nh_ptr);
    _ROSWrapper.initSettings();
}

void Controller::getJointsDatas(double t, double *q, double *qdot)
{
    _time_pre = _time;
    _time = t;
    _dt = _time - _time_pre;
    for (int i = 0; i < _dofs; i++)
    {
        _q(i) = q[i];
        _qdot(i) = qdot[i];
    }
}

void Controller::control()
{
    modelUpdate();
    rosHandle();
    trajectoryPlan();
    switch (_control_mode)
    {
    case 1: // gravity compensation
        gravityCompensation();
        break;
    case 2: // joint space control
        jointControl();
        break;
    case 3: // task space control
        taskControl();
        break;
    default:
        cout << "ERROR! wrong control mode! exit program...\n";
        exit(0);
        break;
    }
    // cout << _x.head(3).transpose() << _x.tail(3).transpose() * RAD2DEG << '\n';
    ros::spinOnce();
}

void Controller::setJointsDatas(double *tau, double *qpos)
{
    if (_control_mode == 3) // task control CLIK
    {
        for (int i = 0; i < _dofs; i++) qpos[i] = _qpos(i);
        qpos[7] = 0.0;
        qpos[8] = 0.0;
    }
    else
    {
        for (int i = 0; i < _dofs; i++) tau[i] = _tau(i);
        tau[7] = 0.0;
        tau[8] = 0.0;
    }
    // cout << _x.head(3).transpose() << ' ' << _x.tail(3).transpose() * RAD2DEG << '\t' << _time << '\n';
}

void Controller::modelUpdate()
{
    _cmodel.updateKinematics(_q, _qdot);
    _cmodel.updateDynamics();
    _cmodel.getJacobian();
    _cmodel.getState();

    _q = _cmodel._q;
    _qdot = _cmodel._qdot;

    _J = _cmodel._J;
    _J_T = _J.transpose();

    _x.head(3) = _cmodel._pos;
    _x.tail(3) = _cmodel._ori;
    _R = _cmodel._R;

    _xdot.head(3) = _cmodel._posdot;
    _xdot.tail(3) = _cmodel._oridot;
}

void Controller::rosHandle()
{
    _control_mode = _ROSWrapper.getCmdMod();
    _jtrajectory.getCmdCount(_ROSWrapper.getCmdCount());
    if (_ROSWrapper.isCmdReceived() == true)
    {
        switch (_control_mode)
        {
        case 1: // gravity compensation
            break;
        case 2: // joint control
            _q_goal = _ROSWrapper.getTargetPose();
            _jtrajectory.resetTarget();
            break;
        case 3: // task control
            _qpos = _q;
            _x_goal = _ROSWrapper.getTargetPose();
            _ctrajectory.resetTarget();
            break;
        default:
            break;
        }
    }
}

void Controller::trajectoryPlan()
{
    switch (_control_mode)
    {
    case 1: // gravity compensation
        break;
    case 2: // joint space control
        if (_jtrajectory.isTrajFinished())
        {
            _jtrajectory.checkSize(_q, _control_mode);
            _jtrajectory.setStart(_q, _qdot, _time);
            _jtrajectory.setGoal(_q_goal, _qdot_goal, _time + 5.0);
        }
        _control_mode = _jtrajectory.getControlMode();
        _jtrajectory.updateTime(_time);
        _q_des = _jtrajectory.getPositionTrajectory();
        _qdot_des = _jtrajectory.getOrientationTrajectory();
        break;
    case 3: // task space control
        if (_ctrajectory.isTrajFinished())
        {
            _ctrajectory.checkSize(_x, _control_mode);
            _ctrajectory.setStart(_x, _xdot, _time);
            _ctrajectory.setGoal(_x_goal, _xdot_goal, _time + 5.0);
        }
        _control_mode = _ctrajectory.getControlMode();
        _ctrajectory.updateTime(_time);
        _x_des = _ctrajectory.getPositionTrajectory();
        _xdot_des = _ctrajectory.getOrientationTrajectory();
        break;
    default:
        cout << "ERROR! wrong control mode! exit program...\n";
        exit(0);
        break;
    }
}

void Controller::jointControl()
{
    _tau.setZero();
    _tau = _cmodel._A * (_kp_j * (_q_des - _q) + _kd_j * (_qdot_des - _qdot)) + _cmodel._bg;
}

void Controller::taskControl()
{
    // _qpos.setZero();
    if (_bool_task_control_init == false)
    {
        _qpos = _q; // tau is not torque in this control mode. It is joint angle 'qpos'
        _bool_task_control_init = true;
    }
    _pos_err = _x_des.head(3) - _x.head(3);
    _ori_err = CMath::calcRotationError(_R, CMath::calcRotationMatrixFromEulerAngleXYZ(_x_des.tail(3)));

    _posdot_err = _xdot_des.head(3) - _xdot.head(3);
    _oridot_err = -_xdot.tail(3); // only damping

    _J_des = _cmodel.getDesiredJacobianFromJointAngle(_qpos);
    _J_T_des = _J_des.transpose();

    _x_err.head(3) = _x_des.head(3) - _cmodel.getDesiredPositionFromJointAngle(_qpos);
    _x_err.tail(3) = CMath::calcRotationError(_R, CMath::calcRotationMatrixFromEulerAngleXYZ(_cmodel.getDesiredOrientationFromJointAngle(_qpos)));

    _qdot_ref = (_J_T_des * (_J_des * _J_T_des).inverse()) * (_xdot_des + 10 * _x_err);
    _qpos = _qpos + _qdot_ref * _dt;

    // _xddot_ref.head(3) = _kp_t * _pos_err + _kd_t * _posdot_err;
    // _xddot_ref.tail(3) = _kp_t * _ori_err + _kd_t * _oridot_err;
    // _lambda = CMath::pseudoInverseQR(_J_T) * _cmodel._A * CMath::pseudoInverseQR(_J);
    // _null_space_projection = _I - _J_T * _lambda * _J * _cmodel._A.inverse();
    // _tau = _J_T * _lambda * _xddot_ref + _null_space_projection * (-_cmodel._A * _kd_j * _qdot) + _cmodel._bg;
}

void Controller::gravityCompensation()
{
    _tau = _cmodel._bg;
}

void Controller::initialize()
{
    //////////////////////////////////////////////////////////////////
    ////////////////////       control mode       ////////////////////
    ////////////////////  1. gravity compensation ////////////////////
    ////////////////////  2. joint space control  ////////////////////
    ////////////////////  3. task space control   ////////////////////
    //////////////////////////////////////////////////////////////////
    _control_mode = 1; // initial control mode of the robot is gravity compensation

    _time = 0.0;
    _time_pre = 0.0;
    _dt = 0.002;
    _dofs = _cmodel.getDOFs();

    _kp_t = 100.0; // task p gain
    _kd_t = 10.0;  // task d gain
    _kp_j = 100.0; // joint p gain
    _kd_j = 20.0;  // joint d gain

    _bool_task_control_init = false;

    _q.setZero(_dofs);
    _qdot.setZero(_dofs);
    _tau.setZero(_dofs);
    _qpos.setZero(_dofs);

    _x.setZero(6);
    _xdot.setZero(6);

    _q_des.setZero(_dofs);
    _qdot_des.setZero(_dofs);
    _q_goal.setZero(_dofs);
    _qdot_goal.setZero(_dofs);
    _x_des.setZero(6);
    _xdot_des.setZero(6);
    _x_goal.setZero(6);
    _xdot_goal.setZero(6);
    _x_err.setZero(6);
    _xdot_err.setZero(6);

    _q_goal(0) = 0.0 * DEG2RAD;
    _q_goal(1) = 0.0 * DEG2RAD;
    _q_goal(2) = 0.0 * DEG2RAD;
    _q_goal(3) = -90.0 * DEG2RAD;
    _q_goal(4) = 0.0 * DEG2RAD;
    _q_goal(5) = 90.0 * DEG2RAD;
    _q_goal(6) = 45.0 * DEG2RAD;
    //_q_goal(7) = 0.0;
    //_q_goal(8) = 0.0;

    _x_goal(0) = 0.15;
    _x_goal(1) = 0.45;
    _x_goal(2) = 0.8;
    _x_goal(3) = 0.0 * DEG2RAD;
    _x_goal(4) = 0.0 * DEG2RAD;
    _x_goal(5) = 0.0 * DEG2RAD;

    _pos_err.setZero();
    _posdot_err.setZero();
    _ori_err.setZero();
    _oridot_err.setZero();
    _xddot_ref.setZero(6);
    _qdot_ref.setZero(_dofs);
    _lambda.setZero(6, 6);
    _null_space_projection.setZero(6, 6);

    _J_weighted_inv.setZero(_dofs, 6);
    _J.setZero(6, _dofs);
    _J_T.setZero(_dofs, 6);

    _J_des.setZero(6, _dofs);
    _J_T_des.setZero(_dofs, 6);

    _R.setZero();
    _I.setZero(_dofs, _dofs);
    _bool = false;
}