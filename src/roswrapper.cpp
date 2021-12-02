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
        _target_pose.resize(9); // NOTE: 9 is the number of joint DoFs
        for (int i = 0; i < 9; i++) _target_pose(i) = msg->data[i + 1];
        ROS_INFO("* Received target position!");
        for (int i = 0; i < 9; i++) cout << "[ " << _target_pose(i) << " ]";
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

int ROSWrapper::getCmdMod()
{
    return _ros_control_mode;
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
    _target_pose.setZero(6);
}