#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "std_srvs/Empty.h"
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaka_planner");
    ros::NodeHandle nh;
    // Start a thread on the CPU
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //  MoveIt uses a JointModelGroup to store the joints of the robot arm, called PLANNING_GROUP. 
    // The "planning group" and the "joint model group" can be used interchangeably throughout the movement.
    // The official configuration file defines it as two move_groups: {manipulator, endeffector}
    static const std::string PLANNING_GROUP = "jaka_zu3";
    // By creating an instance of the planning_interface:`MoveGroupInterface` class, you can easily connect, control, or plan a planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // Add or remove obstacles in the virtual world by using the planning_scene_interface: `PlanningSceneInterface` class
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // Get the state of the robotic arm
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ros::Duration du_1(5);

    //random pose
    // move_group.setRandomTarget();

    //Cartesian spatial goal point planning
    // geometry_msgs::Pose target_pose1;
    // target_pose1.position.x = 0.196444;
    // target_pose1.position.y = -0.391179;
    // target_pose1.position.z = 0.382422;
    // target_pose1.orientation.w = 0.029870;
    // target_pose1.orientation.x = 0.458695;
    // target_pose1.orientation.y = 0.887736;
    // target_pose1.orientation.z = -0.025148;
    // move_group.setPoseTarget(target_pose1);

    //Definie a Plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //Use a Boolean variable to mark whether the motion planning is successful
    bool success =(move_group.plan(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO("Visualizing plan 1 (cartesian space goal) %s",success?"":"FAILED");
    // ROS_INFO("OK");

    // move_group.move();   //The move() function is used to perform motion planning and execution operations, and parameters cannot be passed
    // move_group.execute(my_plan); //The execute() function will block the program
    // move_group.asyncExecute(my_plan);  //asynchronous execution
    // ros::Duration(2).sleep();
    // move_group.stop();


    // Motion planning in joint space. This will replace the pose target we set above.
    //First, we'll create a pointer that references the current robot state. RobotState is an object containing all current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // move_group.setStartStateToCurrentState();

    for (int i = 0; i < 2; i++)
    {
        // Now, let's modify the joint, plan to the new joint space target and visualize the plan.
        move_group.setStartStateToCurrentState();
        joint_group_positions[0] = 0;  // radians
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = -1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
        ROS_INFO("OK");
        // move_group.execute(my_plan);
        move_group.move();
        ros::Duration(0.5).sleep();

        move_group.setStartStateToCurrentState();
        joint_group_positions[0] = 1.57;  // radians
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = -1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
        ROS_INFO("OK");
        // move_group.execute(my_plan);
        move_group.move();
        ros::Duration(0.5).sleep();

        joint_group_positions[0] = 1.57;  // radians
        joint_group_positions[1] = 1.57;  
        joint_group_positions[2] = 1.57;  
        joint_group_positions[3] = 1.57;  
        joint_group_positions[4] = 1.57;  
        joint_group_positions[5] = 0;  
        move_group.setJointValueTarget(joint_group_positions);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (joint space goal) %s", success ? "" : "FAILED");
        ROS_INFO("OK");
        // move_group.execute(my_plan);
        move_group.move();
        ros::Duration(0.5).sleep();

    }

    move_group.setStartStateToCurrentState();
    joint_group_positions[0] = 0;  // radians
    joint_group_positions[1] = 1.57;  
    joint_group_positions[2] = -1.57;  
    joint_group_positions[3] = 1.57;  
    joint_group_positions[4] = 1.57;  
    joint_group_positions[5] = 0;  
    move_group.setJointValueTarget(joint_group_positions);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (joint space goal) %s", success ? "" : "FAILED");
    ROS_INFO("OK");
    // move_group.execute(my_plan);
    move_group.move();
    ros::Duration(1).sleep();
    

    ros::shutdown(); 
    return 0;


}
