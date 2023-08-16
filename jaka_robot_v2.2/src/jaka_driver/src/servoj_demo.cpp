#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include "jaka_msgs/RobotMsg.h"
#include "jaka_msgs/Move.h"
#include "jaka_msgs/ServoMoveEnable.h"
#include "jaka_msgs/ServoMove.h"
#include "jaka_msgs/SetUserFrame.h"
#include "jaka_msgs/SetTcpFrame.h"
#include "jaka_msgs/SetPayload.h"
#include "jaka_msgs/SetCollision.h"
#include "jaka_msgs/ClearError.h"
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"
#include <string>
using namespace std;

BOOL in_pos;
JAKAZuRobot robot;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "client_test");
    ros::NodeHandle nh;
    ros::ServiceClient servo_move_enable_client = nh.serviceClient<jaka_msgs::ServoMoveEnable>("/jaka_driver/servo_move_enable");
    ros::ServiceClient servo_j_client = nh.serviceClient<jaka_msgs::ServoMove>("/jaka_driver/servo_j");
    // ros::service::waitForService("/jaka_driver/servo_j");
    servo_j_client.waitForExistence();
    jaka_msgs::ServoMoveEnable enable_state;
    enable_state.request.enable = TRUE;
    servo_move_enable_client.call(enable_state);
    ros::Duration(1).sleep();
    jaka_msgs::ServoMove servo_pose;
    float pose[6] = {0.001, 0, 0, 0, 0, 0.001};
    for (int i =0; i < 6; i++)
    {
        servo_pose.request.pose.push_back(pose[i]);
    } 

    for (int i = 0; i < 200; i++)
    {
        servo_j_client.call(servo_pose);
        cout << "The return value of calling servo_j:" << servo_pose.response.ret << "  ";
        cout << servo_pose.response.message << endl;
    }
    
    ros::Duration(1).sleep();

    return 0;
}
