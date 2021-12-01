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
    _pub = _nh_ptr->advertise<std_msgs::Float32MultiArray>("/cmd_mod",1);
}

void ROSWrapper::publishTopic()
{
    _pub.publish(_float32_multi_array);
}

void ROSWrapper::initialize()
{
    _float32_multi_array.data.resize(7);
    _float32_multi_array.data.clear();
    _float32_multi_array.data[0] = 0;
    _float32_multi_array.data[1] = 1;
    _float32_multi_array.data[2] = 2;
}