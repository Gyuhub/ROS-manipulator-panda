#include "trajectory.h"

Trajectory::Trajectory()
{
    initialize(7, 0.002);
}

Trajectory::~Trajectory()
{
}

void Trajectory::setDefault(int dofs, double dt)
{
    _dofs = dofs;
    _dt = dt;
}

void Trajectory::setStart(VectorXd x_start, VectorXd xdot_start, double start_time)
{
    _x_start = x_start;
    _xdot_start = xdot_start;
    _start_time = start_time;
}

void Trajectory::setGoal(VectorXd x_goal, VectorXd xdot_goal, double goal_time)
{
    _x_goal = x_goal;
    _xdot_goal = xdot_goal;
    _goal_time = goal_time;
}

void Trajectory::checkSize(VectorXd x, int control_mode)
{
    _size = x.size();
    _control_mode = control_mode;
    _x_start.resize(_size);
    _x_goal.resize(_size);
    _xdot_start.resize(_size);
    _xdot_goal.resize(_size);
}

void Trajectory::updateTime(double time)
{
    _time = time;
}

void Trajectory::resetTarget()
{
    if (_bool_cmd(_cmd_count) == 0)
    {
        _goal_time = 0.0;
        _start_time = 0.0; 
    }
}

void Trajectory::getCmdCount(int cmd_count)
{
    _cmd_count = cmd_count;
    return;
}

bool Trajectory::isTrajFinished()
{
    if (_time == 0)
    {
        return true;
    }
    else if (_time >= _goal_time && _bool_cmd(_cmd_count) == 0)
    {
        _bool_cmd(_cmd_count) = 1;
        _control_mode = 1;
        return true;
    }
    else
    {
        return false;
    }
}

int Trajectory::getControlMode()
{
    return _control_mode;
}

VectorXd Trajectory::getPositionTrajectory()
{
    Eigen::VectorXd x_(_size);
    if (_time <= _start_time)
    {
        x_ = _x_start;
    }
    else if (_time >= _goal_time)
    {
        x_ = _x_goal;
    }
    else
    {
        x_ = _x_start + _xdot_start * (_time - _start_time)
			+ (3.0 * (_x_goal - _x_start) / (std::pow((_goal_time - _start_time), 2)) - 2.0 * _xdot_start / (_goal_time - _start_time) - _xdot_goal / (_goal_time - _start_time)) * std::pow((_time - _start_time), 2)
			+ (-2.0 * (_x_goal - _x_start) / (std::pow((_goal_time - _start_time), 3)) + (_xdot_start + _xdot_goal) / (std::pow((_goal_time - _start_time), 2))) * std::pow((_time - _start_time), 3);
    }
    return x_;
}

VectorXd Trajectory::getOrientationTrajectory()
{
    Eigen::VectorXd xdot_(_size);
    if (_time <= _start_time)
    {
        xdot_ = _xdot_start;
    }
    else if (_time >= _goal_time)
    {
        xdot_ = _xdot_goal;
    }
    else
    {
        xdot_ = _xdot_start
            + 2.0 * (3.0 * (_x_goal - _x_start) / (std::pow((_goal_time - _start_time), 2)) - 2.0 * _xdot_start / (_goal_time - _start_time) - _xdot_goal / (_goal_time - _start_time)) * (_time - _start_time)
			+ 3.0 * (-2.0 * (_x_goal - _x_start) / (std::pow((_goal_time - _start_time), 3)) + (_xdot_start + _xdot_goal) / (std::pow((_goal_time - _start_time), 2))) * std::pow((_time - _start_time), 2);
    }
    return xdot_;
}

void Trajectory::initialize(int dofs, double dt)
{
    _dofs = dofs;
    _size = 6;
    _control_mode = 1;
    _cmd_count = 0;
    _dt = dt;
    _time = 0.0;
    _start_time = 0.0;
    _goal_time = 0.0;

    _x_start.setZero(_size);
    _x_goal.setZero(_size);
    _xdot_start.setZero(_size);
    _xdot_goal.setZero(_size);
    _bool_cmd.setZero(100); // max size of command buffer
}