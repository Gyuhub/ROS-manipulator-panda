#include <ros/ros.h>
#include <iostream>
#include <manipulator_test/pandaSrv.h>
#include <cstdlib>

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manipulator_cli");

    ros::NodeHandle nh_;
    ros::ServiceClient client = nh_.serviceClient<manipulator_test::pandaSrv>("/cmd_mod_srv");
    manipulator_test::pandaSrv srv;
    srv.request.cmd.push_back(atoi(argv[1]));
    int id = srv.request.cmd.front();
    switch (id)
    {
    case 1:
        ROS_INFO("* Gravity Compensation command");
        break;
    case 2:
        ROS_INFO("* Joint Control");
        for (int i = 0; i < 9; i++) srv.request.cmd.push_back(atof(argv[i + 2]));
        ROS_INFO("* Send message!");
        for (auto it = srv.request.cmd.begin(); it != srv.request.cmd.end(); it++) cout << "[ " << *it << " ]";
        cout << '\n';
        break;
    case 3:
        ROS_INFO("* Task Control");
        for (int i = 0; i < 6; i++) srv.request.cmd.push_back(atof(argv[i + 2]));
        ROS_INFO("* Send message!");
        for (auto it = srv.request.cmd.begin(); it != srv.request.cmd.end(); it++) cout << "[ " << *it << " ]";
        cout << '\n';
        break;
    default:
        ROS_ERROR("usage : input id from 1 ~ 3, then input goal pose");
        return 1;
        break;
    }
    if (client.call(srv))
    {
        ROS_INFO("Is Ok? : %ld", (long int)srv.response.ret);
    }
    else
    {
        ROS_ERROR("Failed to call service...");
        return 1;
    }
    return 0;
}