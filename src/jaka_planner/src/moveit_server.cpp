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

using namespace std;
JAKAZuRobot robot;
const  double PI = 3.1415926;
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

//判断robot 是否运行到目标位置
bool jointStates(JointValue joint_pose)
{
    RobotStatus robotstatus;
    robot.get_robot_status(&robotstatus);
    // for (int i = 0; i < 6; i++)
    // {
    //     cout << robotstatus.joint_position[i] << endl;
    // }
    bool joint_state = true;
   
    for (int i = 0; i < 6; i++)
    {
        bool ret = joint_pose.jVal[i] * 180 / PI - 0.2 < robotstatus.joint_position[i] * 180 / PI
        && robotstatus.joint_position[i] * 180 / PI < joint_pose.jVal[i] * 180 / PI + 0.2;
        // cout << "ret = " << ret << endl;
        joint_state = joint_state && ret;  
    }
    cout << "Whether the robot has reached the target position: " << joint_state << endl;
    return joint_state;
}

//Moveit的服务端
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
        // int step_num=int (dt/0.008+0.5);
        int step_num=int (dt/0.008);
        int sdk_res=robot.servo_j(&joint_pose, MoveMode::ABS, step_num);
        // int sdk_res=robot.joint_move(&joint_pose,MoveMode::ABS,1,1,1,0.5,p);
        if (sdk_res !=0)
        {
            ROS_INFO("servo_j 运动失败");
        } 
        ROS_INFO("The return status of servo_j:%d",sdk_res);
        ROS_INFO("Accepted joint angle: %f %f %f %f %f %f %f %d", joint_pose.jVal[0],joint_pose.jVal[1],joint_pose.jVal[2],joint_pose.jVal[3],joint_pose.jVal[4],joint_pose.jVal[5],dt,step_num);
        }
    as->setSucceeded();
    while(true)
    {
        if(jointStates(joint_pose))
        {
            robot.servo_move_enable(false);
            ROS_INFO("伺服模式使能关闭");
            break;
        }
        ros::Duration(1).sleep();
    }
    

}

//向move_group发送实体机器人的关节信息
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
    string robot_ip = nh.param("ip", default_ip);
    //robot 登入
    robot.login_in(robot_ip.c_str());
    // robot.login_in("192.168.126.128");
    ros::Rate rate(75);
    //robot 关闭伺服模式使能
    robot.servo_move_enable(false);
    ros::Duration(0.5).sleep();
    //robot 设置滤波参数
    robot.servo_move_use_joint_LPF(0.5);
    //robot 上电
    robot.power_on();
    //robot 上使能
    robot.enable_robot();

    //创建"/joint_states"的话题
    ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    //创建action服务端对象
    //Server moveit_server(nh, "/jaka_zu3_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    // Server moveit_server(nh, "/jaka_zu7_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    //Server moveit_server(nh, "/jaka_zu5_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    //Server moveit_server(nh, "/jaka_zu12_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    //Server moveit_server(nh, "/jaka_zu18_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    Server moveit_server(nh, "/jaka_minicobo_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    // Server moveit_server(nh, "/jaka_Pro7_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);
    // Server moveit_server(nh, "/jaka_pro16_controller/follow_joint_trajectory", boost::bind(&goalCb, _1, &moveit_server), false);

    // 服务器开始运行
	moveit_server.start();
    cout << "moveit_start" << endl;

    while(ros::ok())
    {
        //向RVIZ上报robot 关节信息
        joint_states_callback(joint_states_pub);
        rate.sleep();
        ros::spinOnce();
    }
    //ros::spin();
}