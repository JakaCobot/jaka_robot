#include <cstdlib>
#include <cstdio>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"

#include "jaka_msgs/RobotMsg.h"
#include "jaka_msgs/Move.h"
#include "jaka_msgs/ServoMoveEnable.h"
#include "jaka_msgs/ServoMove.h"
#include "jaka_msgs/SetUserFrame.h"
#include "jaka_msgs/SetTcpFrame.h"
#include "jaka_msgs/SetPayload.h"
#include "jaka_msgs/SetCollision.h"
#include "jaka_msgs/SetIO.h"
#include "jaka_msgs/GetIO.h"
#include "jaka_msgs/ClearError.h"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <string>
#include <map>
#include <chrono>
#include <thread>
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "linear_move_client");
    // Get two addends from terminal
    if (argc != 7)
    {
        ROS_INFO("bbq0");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<jaka_msgs::Move>("/jaka_driver/linear_move");
    jaka_msgs::Move srv;
    ROS_INFO("bbq3");
    for (int i = 0; i <6; i++)
    {
        srv.request.pose.push_back(atof(argv[i+1]));
    }
    // srv.request.pose[0] = -376.0;
    // ROS_INFO("bbq4");
    // srv.request.pose[1] = atof(argv[2]);
    // ROS_INFO("bbq5");
    // srv.request.pose[2] = atof(argv[3]);
    // srv.request.pose[3] = atof(argv[4]);
    // srv.request.pose[4] = atof(argv[5]);
    // srv.request.pose[5] = atof(argv[6]);
    srv.request.mvvelo = 100;
    srv.request.mvacc = 100;
	// srv.request.coord_mode=0;
	// srv.request.index=0;
    if (client.call(srv))
    {
        ROS_INFO("ret: %d", (int)srv.response.ret);
    }
    else
    {
    	ROS_INFO("bbq6");
        ROS_ERROR("Failed to call service");
        return 1;
    }

    return 0;
}