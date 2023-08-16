#include <ros/ros.h>
#include <QtGui>
#include <QApplication>
#include  <sensor_msgs/JointState.h>
#include "../include/jaka_jog_panel/main_window.hpp"

void joint_states_callback(const sensor_msgs::JointState& joint_states)
{
    ROS_INFO("I heard: joint1:[%f], joint2:[%f], joint3:[%f], joint4:[%f], joint5:[%f], joint6:[%f]",
    joint_states.position[0], joint_states.position[1], joint_states.position[2],
    joint_states.position[3], joint_states.position[4], joint_states.position[5]);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    /*********************
    ** Qt
    **********************/
    //ROS_INFO("1");
        //app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    ros::NodeHandle n;
    ROS_INFO("1");
    ros::Subscriber jog_command_sub = n.subscribe("robot_jog_command", 10, joint_states_callback);
    ros::spin();

  return 0;
}
