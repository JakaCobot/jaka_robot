#ifndef _JAKAJOGPANEL_H
#define _JAKAJOGPANEL_H

#include  <ros/ros.h>
#include  <rviz/panel.h>
#include  <sensor_msgs/JointState.h>
#include  <std_msgs/Float32MultiArray.h>
#include  <QWidget>
#include  <QTimer>
#include <QMainWindow>
//#include <QtGui/QMainWindow>
#include "ui_main_window.h"

#include "sensor_msgs/JointState.h"

#include "../../src/libs/robot.h"
//#include "libs/conversion.h"

#define PI 3.1415926

namespace Ui {
  class Jaka_Jog_Panel;
}


namespace Jaka_jog_panel
{
  class MainWindow : public QMainWindow
  {
      Q_OBJECT

    public:
      explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
      ~MainWindow();

    private Q_SLOTS:
      //Joint button  Left-Decrease Right-Increase
      void on_joint1Left_pressed();
      void on_joint1Right_pressed();

      void on_joint2Left_pressed();
      void on_joint2Right_pressed();

      void on_joint3Left_pressed();
      void on_joint3Right_pressed();

      void on_joint4Left_pressed();
      void on_joint4Right_pressed();

      void on_joint5Left_pressed();
      void on_joint5Right_pressed();

      void on_joint6Left_pressed();
      void on_joint6Right_pressed();

      //button release
      void on_joint1Left_released();
      void on_joint1Right_released();

      void on_joint2Left_released();
      void on_joint2Right_released();

      void on_joint3Left_released();
      void on_joint3Right_released();

      void on_joint4Left_released();
      void on_joint4Right_released();

      void on_joint5Left_released();
      void on_joint5Right_released();

      void on_joint6Left_released();
      void on_joint6Right_released();

      void on_joint1Slider_valueChanged(int value);
      void on_joint2Slider_valueChanged(int value);
      void on_joint3Slider_valueChanged(int value);
      void on_joint4Slider_valueChanged(int value);
      void on_joint5Slider_valueChanged(int value);
      void on_joint6Slider_valueChanged(int value);

      void timer_timeout();//Timer callback

      void Jog_Command_Callback(ros::Publisher& Publish);//ROS Timer

      void on_connect_button_pressed();

      void on_disconnect_button_pressed();

      void on_lineEdit_textChanged(const QString &arg1);

  protected:
      QTimer* m_timer;

      void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

      std_msgs::Float32MultiArray joints;

      float nowJointpose[6];

      //ros::NodeHandle nh;
      //ros::Publisher jog_command_pub = nh.advertise<sensor_msgs::JointState>("robot_jog_command", 10);
      ros::Publisher jog_command_pub;

      //sensor_msgs::JointState joint_states;



    private:
    Ui::MainWindow *ui;

    JAKAZuRobot robot;//Jaka robot
    std::string RobotIP = "192.168.3.100";
    //const char* RobotIP = "192.168.3.100";
//std::string robotIP;

    JointValue joint_pose;
    bool Real_Robot_Flag = false;

    QTimer *jTimer;
    QTimer *rosTimer;
    QTime *jTimeCounter;
    QTime *rosTimeCounter;
    int Update_Timer = 10;
    int ros_Timer = 10;

    void Jog_Ros_Init(int argc, char **argv);

    void Robot_Connect();
    //QTextEdit *joint1_text;

  };
}


#endif // JAKAJOGPANEL_H
