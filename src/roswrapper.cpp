#include "roswrapper.h"

ROSWrapper::ROSWrapper()
{
    initialize();
}

ROSWrapper::~ROSWrapper()
{
}

void ROSWrapper::getNodeHandler(ros::NodeHandle* nh_ptr)
{
    _nh_ptr = nh_ptr;
}

void ROSWrapper::initSettings()
{
    _pub = _nh_ptr->advertise<std_msgs::Float32MultiArray>("/cmd_TODO",1);
    _sub = _nh_ptr->subscribe("/cmd_mod", 1, &ROSWrapper::CmdModCallBack, this);
    _srv = _nh_ptr->advertiseService("/cmd_mod_srv", &ROSWrapper::CmdModServiceCallBack, this);
    ROS_INFO("* Initialize ROS settings...");
}

void ROSWrapper::CmdModCallBack(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    ROS_INFO("Received command of control mode! the control mode is [%f]", msg->data[0]);
    _ros_control_mode = msg->data[0];
    switch (_ros_control_mode)
    {
    case 1: // gravity compensation
        ROS_WARN("<<<<< WARNING! Modify the control mode to Gravity Compensation!! >>>>>");
        break;
    case 2: // joint control
        ROS_WARN("<<<<< WARNING! Modify the control mode to Joint Control!! >>>>>");
        _target_pose.resize(_jdofs); // NOTE: 9 is the number of joint DoFs
        for (int i = 0; i < _jdofs; i++) _target_pose(i) = msg->data[i + 1];
        ROS_INFO("* Received target position!");
        for (int i = 0; i < _jdofs; i++) cout << "[ " << _target_pose(i) << " ]";
        cout << '\n'; 
        break;
    case 3: // task control
        ROS_WARN("<<<<< WARNING! Modify the control mode to Task Control!! >>>>>");
        _target_pose.resize(6); // NOTE: 6 is the number of task space DoFs
        for (int i = 0; i < 6; i++) _target_pose(i) = msg->data[i + 1];
        ROS_INFO("* Received target position!");
        for (int i = 0; i < 6; i++) cout << "[ " << _target_pose(i) << " ]";
        cout << '\n';
        break;
    default:
        ROS_ERROR("Something is wrong! Wait until the correct control mode is received...");
        _ros_control_mode = 1;
        break;
    }
}

bool ROSWrapper::CmdModServiceCallBack(manipulator_test::pandaSrv::Request& req, manipulator_test::pandaSrv::Response& res)
{
    _ros_control_mode = (int)req.cmd.front();
    _target_pose.setZero();
    int idx = 0;
    switch (_ros_control_mode)
    {
    case 1: // gravity compensation
        ROS_WARN("<<<<< WARNING! Modify the control mode to Gravity Compensation!! >>>>>");
        break;
    case 2: // joint control
        ROS_WARN("<<<<< WARNING! Modify the control mode to Joint Control!! >>>>>");
        if (req.cmd.size() != 10) {ROS_ERROR("ERROR! The size of command is not appropriate! Please check the size of command and it should be 9..."); return false;}
        _target_pose.resize(_jdofs); // NOTE: 9 is the number of joint DoFs
        _target_pose.setZero();
        for (auto it = (req.cmd.begin() + 1); it != req.cmd.end(); it++) _target_pose(idx++) = *it;
        ROS_INFO("* Received target pose!");
        for (int i = 0; i < _jdofs; i++) cout << "[ " << _target_pose(i) << " ]";
        cout << '\n'; 
        break;
    case 3: // task control
        ROS_WARN("<<<<< WARNING! Modify the control mode to Task Control!! >>>>>");
        if (req.cmd.size() != 7) {ROS_ERROR("ERROR! The size of command is not appropriate! Please check the size of command and it should be 9..."); return false;}
        _target_pose.resize(6); // NOTE: 6 is the number of task space DoFs
        _target_pose.setZero();
        for (auto it = (req.cmd.begin() + 1); it != req.cmd.end(); it++) _target_pose(idx++) = *it;
        ROS_INFO("* Received target pose!");
        for (int i = 0; i < 6; i++) cout << "[ " << _target_pose(i) << " ]";
        cout << '\n';
        break;
    default:
        ROS_ERROR("Something is wrong! Wait until the correct control mode is received...");
        _ros_control_mode = 1;
        break;
    }
    res.ret = true;
    return true;
}

int ROSWrapper::getCmdMod()
{
    return _ros_control_mode;
}

Eigen::VectorXd ROSWrapper::getTargetPose()
{
    if (_ros_control_mode == 1) {}
    else if (_ros_control_mode == 2)
    {
        Eigen::VectorXd q_(_jdofs);
        q_ = _target_pose;
        return q_;
    }
    else if (_ros_control_mode == 3)
    {
        Eigen::VectorXd x_(6);
        x_ = _target_pose;
        return x_;
    }
    else {}
}

void ROSWrapper::publishTopic()
{
    _pub.publish(_float32_multi_array);
}

void ROSWrapper::initialize()
{
    _float32_multi_array.data.resize(7);
    _float32_multi_array.data[0] = 0;
    _float32_multi_array.data[1] = 1;
    _float32_multi_array.data[2] = 2;

    _ros_control_mode = 1;
    _jdofs = 9;
    _target_pose.setZero(6);
}