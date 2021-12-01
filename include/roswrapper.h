#pragma once
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>

using namespace std;
using namespace ros;

class ROSWrapper
{
public:
    ROSWrapper();
    ~ROSWrapper();
    void getNodeHandler(ros::NodeHandle* nh_ptr);
    void publishTopic();
private:
    void initialize();

    ros::NodeHandle* _nh_ptr;
    ros::Publisher _pub;
    ros::Subscriber _sub;

    std_msgs::Float32MultiArray _float32_multi_array;
};