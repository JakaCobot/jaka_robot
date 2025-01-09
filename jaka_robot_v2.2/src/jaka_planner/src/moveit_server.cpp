#include "ros/ros.h"
#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jkerr.h"
#include "jaka_planner/jktypes.h"
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <map>
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Empty.h"
#include <thread>

using namespace std;
JAKAZuRobot robot;
const  double PI = 3.1415926;
BOOL in_pos;
int ret_preempt;
int ret_inPos;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

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

// Determine if the robot has reached the target position.
bool jointStates(JointValue joint_pose)
{
    RobotStatus robotstatus;
    robot.get_robot_status(&robotstatus);
    bool joint_state = true;
   
    for (int i = 0; i < 6; i++)
    {
        bool ret = joint_pose.jVal[i] * 180 / PI - 0.2 < robotstatus.joint_position[i] * 180 / PI
        && robotstatus.joint_position[i] * 180 / PI < joint_pose.jVal[i] * 180 / PI + 0.2;
        joint_state = joint_state && ret; 
    }
    cout << "Whether the robot has reached the target position: " << joint_state << endl;       //1到达；0未到达
    return joint_state;
}

//Moveit server
void goalCb(const control_msgs::FollowJointTrajectoryGoalConstPtr& torso_goal, Server* as)
{
    BOOL in_pos;
    robot.servo_move_enable(true);
    int point_num=torso_goal->trajectory.points.size();
    ROS_INFO("number of points: %d",point_num);
    JointValue joint_pose;
    float lastDuration=0.0;
    OptionalCond* p = nullptr;
    for (int i=1; i<point_num; i++) {        
        joint_pose.jVal[0] = torso_goal->trajectory.points[i].positions[0];
        joint_pose.jVal[1] = torso_goal->trajectory.points[i].positions[1];
        joint_pose.jVal[2] = torso_goal->trajectory.points[i].positions[2];
        joint_pose.jVal[3] = torso_goal->trajectory.points[i].positions[3];
        joint_pose.jVal[4] = torso_goal->trajectory.points[i].positions[4];
        joint_pose.jVal[5] = torso_goal->trajectory.points[i].positions[5];      
        float Duration=torso_goal->trajectory.points[i].time_from_start.toSec();

        float dt=Duration-lastDuration;
        lastDuration=Duration;

        int step_num=int (dt/0.008);
        int sdk_res=robot.servo_j(&joint_pose, MoveMode::ABS, step_num);

        if (sdk_res !=0)
        {
            ROS_INFO("Servo_j Motion Failed");
        } 
        ROS_INFO("The return status of servo_j:%d",sdk_res);
        ROS_INFO("Accepted joint angle: %f %f %f %f %f %f %f %d", joint_pose.jVal[0],joint_pose.jVal[1],joint_pose.jVal[2],joint_pose.jVal[3],joint_pose.jVal[4],joint_pose.jVal[5],dt,step_num);
        }

    while(true)
    {
        if(jointStates(joint_pose))
        {
            robot.servo_move_enable(false);
            ROS_INFO("Servo Mode Disable");
            cout<<"==============Motion stops or reaches the target position=============="<<endl;
            break;
        }

        if ( ret_preempt = as->isPreemptRequested())      
        {
            robot.motion_abort();
            robot.servo_move_enable(false);
            ROS_INFO("Servo Mode Disable");
            cout<<"==============Motion stops or reaches the target position=============="<<endl;
            break;
        }
        ros::Duration(0.5).sleep();
    }
as->setSucceeded();    
ros::Duration(0.5).sleep();
}

//Send the joint value of the physical robot to move_group
void joint_states_callback(ros::Publisher joint_states_pub)
{
    sensor_msgs::JointState joint_position;
    RobotStatus robotstatus;
    robot.get_robot_status(&robotstatus);
    for (int i = 0; i < 6; i++)
    {
        joint_position.position.push_back(robotstatus.joint_position[i]);
        int j = i + 1;
        joint_position.name.push_back("joint_" + to_string(j));
    }
    joint_position.header.stamp = ros::Time::now();
    joint_states_pub.publish(joint_position);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "moveit_server");
    ros::NodeHandle nh;
    string default_ip = "10.5.5.100";
    string default_model = "zu3";
    string robot_ip = nh.param("ip", default_ip);
    string robot_model = nh.param("model", default_model);
    robot.login_in(robot_ip.c_str());
    // robot.set_status_data_update_time_interval(100);
    ros::Rate rate(125);
    robot.servo_move_enable(false);
    ros::Duration(0.5).sleep();
    //Set filter parameter
    robot.servo_move_use_joint_LPF(0.5);
    RobotStatus robotstatus;
    robot.get_robot_status(&robotstatus);
     cout << "robotstatus.powered_on:" << robotstatus.powered_on << ", " << "robotstatus.enabled:" << robotstatus.enabled << endl;
    if (!robotstatus.powered_on)
    {
        robot.power_on();
        sleep(8);
    }
    if (!robotstatus.enabled)
    {
        robot.enable_robot();
        sleep(4);
    }
    
    //Create topic "/joint_states"
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    //Create action server object
    Server moveit_server(nh, "/jaka_"+robot_model+"_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
	moveit_server.start();
    cout << "==================Moveit Start==================" << endl;

    while(ros::ok())
    {
        //Report robot joint information to RVIZ
        joint_states_callback(joint_states_pub);
        rate.sleep();
        ros::spinOnce();
    }
    //ros::spin();
}