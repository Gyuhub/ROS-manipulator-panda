#pragma once
#include <ros/ros.h>
#include <manipulator_test/pandaSrv.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace ros;
using namespace Eigen;

class ROSWrapper
{
public:
    ROSWrapper();
    ~ROSWrapper();
    void getNodeHandler(ros::NodeHandle* nh_ptr);
    void initSettings();
    void publishTopic();
    void CmdModCallBack(const std_msgs::Float32MultiArrayConstPtr& msg);
    bool CmdModServiceCallBack(manipulator_test::pandaSrv::Request& req, manipulator_test::pandaSrv::Response& res);
    
    bool isCmdReceived();
    int getCmdMod();
    int getCmdCount();
    Eigen::VectorXd getTargetPose();
private:
    void initialize();

    ros::NodeHandle* _nh_ptr;
    ros::Publisher _pub;
    ros::Subscriber _sub;
    ros::ServiceServer _srv;

    ////////////////////  ROS standard message type declaration ////////////////////
    
    std_msgs::Float32MultiArray _float32_multi_array;

    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////  USER defined variables to handle with ROS ////////////////////

    bool _is_cmd_received;
    int _ros_control_mode, _ros_cmd_count;
    int _jdofs;
    Eigen::VectorXd _target_pose;

    ////////////////////////////////////////////////////////////////////////////////////

};