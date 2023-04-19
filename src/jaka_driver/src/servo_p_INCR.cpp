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

#include "unistd.h"
#include <iostream> 
#include <fstream>
#include <sstream>
#include <vector>
#include <cstring>
#include <algorithm>
#include <iomanip>

using namespace std;


struct Position{
	
	double pos1;
	double pos2;
	double pos3;
	double pos4;
	double pos5;
	double pos6;

};


int main(int argc, char *argv[])
{
    //设置编码
    setlocale(LC_ALL, "");
    //初始化节点
    ros::init(argc, argv, "client_test");
    //创建句柄
    ros::NodeHandle nh;

    ros::ServiceClient servo_move_enable_client = nh.serviceClient<jaka_msgs::ServoMoveEnable>("/jaka_driver/servo_move_enable");
    ros::ServiceClient servo_p_client = nh.serviceClient<jaka_msgs::ServoMove>("/jaka_driver/servo_p");
    // ros::service::waitForService("/jaka_driver/servo_j");
    ros::ServiceClient movj = nh.serviceClient<jaka_msgs::Move>("/jaka_driver/joint_move");
    // ros::service::waitForService("/jaka_driver/servo_j");
    movj.waitForExistence();

    jaka_msgs::Move joint_move;
    double pos666[]={0.524264,1.717393,0.591115,1.753385,-1.570553,0.000055};
    for (int i = 0; i < 6; i++)
    {
        joint_move.request.pose.push_back((pos666[i]));
        
    }
    joint_move.request.mvvelo = 1;
    joint_move.request.mvacc = 1;
    movj.call(joint_move);
    ROS_INFO("设置机器人初始点");
    ros::Duration(5).sleep();

    ifstream infile("/home/whm/jaka_robot_V2.1/jaka_robot/src/jaka_driver/src/sdp.csv", ios::in);
	string line;
	vector<struct  Position> posVector;
	getline(infile, line);
	while (getline(infile, line)) {
		stringstream ss(line);
		string str;
		Position position;

		getline(ss, str, ',');
		position.pos1 = stod(str);
		getline(ss, str, ',');
		position.pos2 = stod(str);
		getline(ss, str, ',');
		position.pos3 = stod(str);
		getline(ss, str, ',');
		position.pos4  = stod(str);
		getline(ss, str, ',');
		position.pos5  = stod(str);
		getline(ss, str, ',');
		position.pos6 = stod(str);
		posVector.push_back(position);
	}
 
    cout << "Read file" << endl;
    jaka_msgs::ServoMoveEnable enable_state;
    enable_state.request.enable = TRUE;
    servo_move_enable_client.call(enable_state);
    ros::Duration(1).sleep();
    jaka_msgs::ServoMove servo_pose;
    cout << "ServoMove enable" << endl;
    // float pose[6] = {0.524264,1.717393,0.591115,1.753385,-1.570553,0.000055};
    cout<<posVector[0].pos1<<endl;
    
for (int i = 0; i < posVector.size(); i++) {
    double pose[6] ={posVector[i].pos1,posVector[i].pos2,posVector[i].pos3,posVector[i].pos4,posVector[i].pos5,posVector[i].pos6 };
                for (int j =0; j < 6; j++)
                {
                        servo_pose.request.pose.push_back(pose[j]*0.008);
                } 
        servo_p_client.call(servo_pose);

        cout << "调用servo_p的返回值:" << servo_pose.response.ret << "  ";
        cout << servo_pose.response.message << endl;
        servo_pose.request.pose.clear();

    }
    // ros::Duration(1).sleep();
    enable_state.request.enable = FALSE;
    servo_move_enable_client.call(enable_state);

    return 0;
}