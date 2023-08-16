#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/StdVector"
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"
#include <string>
#include <map>
#include <chrono>
#include <thread>
using namespace std;
const double PI = 3.1415926;
map<int, string>mapErr = {
    {2,"ERR_FUCTION_CALL_ERROR"},
    {-1,"ERR_INVALID_HANDLER"},
    {-2,"ERR_INVALID_PARAMETER"},
    {-3,"ERR_COMMUNICATION_ERR"},
    {-4,"ERR_KINE_INVERSE_ERR"},
    {-5,"ERR_EMERGENCY_PRESSED"},
    {-6,"ERR_NOT_POWERED"},
    {-7,"ERR_NOT_ENABLED"},
    {-8,"ERR_DISABLE_SERVOMODE"},
    {-9,"ERR_NOT_OFF_ENABLE"},
    {-10,"ERR_PROGRAM_IS_RUNNING"},
    {-11,"ERR_CANNOT_OPEN_FILE"},
    {-12,"ERR_MOTION_ABNORMAL"}
};
JAKAZuRobot robot;

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "moveit_server");
    ros::NodeHandle nh;
    string default_ip = "10.5.5.100";
    string robot_ip = nh.param("ip", default_ip);
    robot.login_in(robot_ip.c_str());
    robot.power_on();
    robot.enable_robot();

    RobotStatus robot_status;
    robot.get_robot_status(&robot_status);
    for (int i = 0; i < 6; i++)
    {
        cout << robot_status.joint_position[i] << endl;
    }

   return 0;
                                                                               
}