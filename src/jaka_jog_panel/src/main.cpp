#include <QtGui>
#include <QApplication>
#include  <sensor_msgs/JointState.h>
#include "../include/jaka_jog_panel/main_window.hpp"


int main(int argc, char **argv) {
    //ros::init(argc, argv, "main");
    /*********************
    ** Qt
    **********************/
    //ROS_INFO("1");
    QApplication app(argc, argv);
    Jaka_jog_panel::MainWindow w(argc,argv);
    w.setWindowTitle("Jaka Jog Panel v1.0");
    w.show();
    //app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    //ros::NodeHandle n;
    //ros::Subscriber jog_command_sub = n.subscribe("robot_jog_command", 10, joint_states_callback);
    //ros::spin();


    int result = app.exec();

  return result;
}
