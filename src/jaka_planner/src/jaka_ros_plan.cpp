#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jaka_planner");
    ros::NodeHandle nh;
    // 在CPU中开启一个线程
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // MoveIt 使用JointModelGroup储存机械臂的joints，被称为PLANNING_GROUP.在整个运动中"planning group"和"joint model group"可以互换使用
    // 在官方提供的UR5e配置文件将其定义为两个move_group:{manipulator,endeffector}
    static const std::string PLANNING_GROUP = "jaka_zu7";

    // 通过 创建planning_interface:`MoveGroupInterface` 类的实例可以轻松连接、控制或者计划planning group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // 通过 使用planning_scene_interface:`PlanningSceneInterface`类在virtual world添加或者移除障碍物
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 获取机械臂状态
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ros::Duration du_1(5);

    
    //move_group.setRandomTarget();

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.38;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);

    //定义⼀个plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //⽤布尔型变量标记运动规划是否成功
    bool success =(move_group.plan(my_plan)== moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // 打印结果
    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    ros::Duration du_2(2);
    ROS_INFO("OK");
    move_group.execute(my_plan);
    ros::Duration du_3(5);
    // 关节空间的运动规划。这将取代我们上面设置的姿态目标。
    // 首先，我们将创建一个引用当前机器人状态的指针。RobotState是包含所有当前位置/速度/加速度数据的对象。
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // 接下来获取组的当前关节值集合。
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // 现在，让我们修改其中一个关节，计划到新的关节空间目标并将计划可视化。
    joint_group_positions[0] = -1.0;  // radians
    move_group.setJointValueTarget(joint_group_positions);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    ROS_INFO("OK");
    move_group.execute(my_plan);

    return 0;
}