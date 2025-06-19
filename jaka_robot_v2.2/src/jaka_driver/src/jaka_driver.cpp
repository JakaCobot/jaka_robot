#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/TwistStamped.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
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
#include "jaka_msgs/SetIO.h"
#include "jaka_msgs/GetIO.h"
#include "jaka_msgs/GetFK.h"
#include "jaka_msgs/GetIK.h"
#include "jaka_msgs/ClearError.h"

#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jkerr.h"
#include "jaka_driver/jktypes.h"
#include "jaka_driver/conversion.h"

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <string>
#include <map>
#include <chrono>
#include <thread>
using namespace std;

const double PI = 3.1415926;
//Define variable: the direction that was sent down the last time the jog was called
int jog_index_last = -1; 
//Define variable: number of calls to jog
int jog_count = 0;
//Define variable: save the number of jog calls
int jog_count_temp = 0;
JAKAZuRobot robot;
//SDK interface return status
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

ros::Publisher tool_position_pub ;
ros::Publisher joint_position_pub ;
ros::Publisher robot_state_pub;

bool linear_move_callback(jaka_msgs::Move::Request &request,
                         jaka_msgs::Move::Response &response)
{
    CartesianPose end_pose;
    double speed = (double)request.mvvelo;
    double accel = (double)request.mvacc;
    double tol = 0.5;
    Rpy rpy;
    OptionalCond *option_cond = nullptr;
    end_pose.tran.x = request.pose[0];
    end_pose.tran.y = request.pose[1];
    end_pose.tran.z = request.pose[2];
    end_pose.rpy.rx = request.pose[3];
    end_pose.rpy.ry = request.pose[4];
    end_pose.rpy.rz = request.pose[5];
    int ret = robot.linear_move(&end_pose, MoveMode::ABS, TRUE, speed, accel, tol, option_cond);
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "linear_move has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;

}

bool joint_move_callback(jaka_msgs::Move::Request &request,
                        jaka_msgs::Move::Response &response)
{
    JointValue joint_pose;
    joint_pose.jVal[0] = request.pose[0];
    joint_pose.jVal[1] = request.pose[1];
    joint_pose.jVal[2] = request.pose[2];
    joint_pose.jVal[3] = request.pose[3];
    joint_pose.jVal[4] = request.pose[4]; 
    joint_pose.jVal[5] = request.pose[5];
    double speed = (double)request.mvvelo;
    double accel = (double)request.mvacc;
    double tol = 0.5;
    OptionalCond *option_cond = nullptr;

    int ret = robot.joint_move(&joint_pose, MoveMode::ABS, true, speed, accel, tol, option_cond);
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "joint_move has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool jog_callback(jaka_msgs::Move::Request &request,
        jaka_msgs::Move::Response &response)
{
    // 1. Initialization parameters
    double move_velocity = 0;
    CoordType coord_type = COORD_JOINT;
    
    // 2. Select index   mapping the index and velocity
    //request.index 如果是关节空间   就是  0+,0-,1+,1-,2+,2-,3+,3-,4+,4-,5+,5-
    //request.index 如果是笛卡尔空间 就是x+,x-,y+,y-,z+,z-,rx+,rx-,ry+,ry-,rz+,rz-
    float index_temp = static_cast<float>(request.index) / 2 + 0.1;
    int index = static_cast<int>(index_temp);
    
    // 3. Select coordinates
    switch (request.coord_mode)
    {
        case 0:
            //coordinate system of joints
            coord_type = COORD_JOINT; 
            //Joint Movement Velocity (rad/s)
            move_velocity = request.mvacc;       
            break;
        case 1:
            //Base coordinate system (Cartesian space)
            coord_type = COORD_BASE;
            //movement speed (mm/s)
            move_velocity = request.mvacc;  
            break;
        case 2:
            //Tool coordinate system (Cartesian space)
		    coord_type = COORD_TOOL;
            //movement speed (mm/s)
            move_velocity = request.mvacc;
            break; 
        default:
            ROS_INFO("Coordinate system input error, please re-enter");
            return true;
    }
    // 4. Determine the direction of velocity 
    //Determine whether robot motion (articulated or Cartesian) is in a positive or negative direction
    if(request.index&1)
    {
        move_velocity = -move_velocity;
    }
    //5. Conducting jogging
    if (jog_index_last != request.index)
    {   
        int ret = robot.motion_abort();
        if (ret == 0)
        {
            int jog_state = robot.jog(index, CONTINUE, coord_type, move_velocity, 0);
            switch(jog_state)
            {
                case 0:
                    response.ret = 1;
                    response.message = "Position is reached";
                    break;
                default:
                    response.ret = jog_state;
                    response.message = "error occurred:" + mapErr[jog_state];
                    break;
            }
        }
        else
        {
            response.ret = ret;
            response.message = "error occurred:" + mapErr[ret];
        }
        jog_index_last = request.index;
    }
    else
    {
        response.ret = 1;
        response.message = "Robot is jogging";
        ROS_INFO("Robot is jogging\n");
    }
    jog_count = jog_count + 1;
    return true;
}

bool servo_move_enable_callback(jaka_msgs::ServoMoveEnable::Request &request,
                               jaka_msgs::ServoMoveEnable::Response &response)
{
    BOOL enable = request.enable;
    int ret = robot.servo_move_enable(enable);
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "servo_move_enable has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool servo_p_callback(jaka_msgs::ServoMove::Request &request,
                     jaka_msgs::ServoMove::Response &response)
{
    //speed * 0.008
    CartesianPose cartesian_pose;
    cartesian_pose.tran.x = request.pose[0];
    cartesian_pose.tran.y = request.pose[1];
    cartesian_pose.tran.z = request.pose[2];
    cartesian_pose.rpy.rx = request.pose[3];
    cartesian_pose.rpy.ry = request.pose[4];
    cartesian_pose.rpy.rz = request.pose[5];
    int ret = robot.servo_p(&cartesian_pose, MoveMode::INCR);
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "Servo_p has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool servo_j_callback(jaka_msgs::ServoMove::Request &request,
                     jaka_msgs::ServoMove::Response &response)
{
    JointValue joint_pose;
    joint_pose.jVal[0] = request.pose[0];
    joint_pose.jVal[1] = request.pose[1];
    joint_pose.jVal[2] = request.pose[2];
    joint_pose.jVal[3] = request.pose[3];
    joint_pose.jVal[4] = request.pose[4];
    joint_pose.jVal[5] = request.pose[5];
    int ret = robot.servo_j(&joint_pose, MoveMode::INCR);
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "Servo_j has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool stop_move_callback(std_srvs::Empty::Request &request,
                        std_srvs::Empty::Response &response)
{
    //Initialize jog related parameters
    jog_count = 0;
    jog_count_temp = 0;
    jog_index_last = -1;
    int ret = robot.motion_abort();
    switch(ret)
    {
        case 0:
            ROS_INFO("stop_move has been executed");

            break;
        default:
            ROS_INFO("error occurred: %s", mapErr[ret].c_str());
            return false;;
    }
    return true;
}


bool set_toolFrame_callback(jaka_msgs::SetTcpFrame::Request &request,
                          jaka_msgs::SetTcpFrame::Response &response)
{
    CartesianPose tool_frame;
    int tool_frame_id = request.tool_num;
    tool_frame.tran.x = request.pose[0];
    tool_frame.tran.y = request.pose[1];
    tool_frame.tran.z = request.pose[2];
    Eigen::Vector3d Angaxis = {request.pose[3],request.pose[4],request.pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(tool_frame.rpy));
    // Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
    // auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
    // tool_frame.rpy.rx = rpy.x();
    // tool_frame.rpy.ry = rpy.y();
    // tool_frame.rpy.rz = rpy.z();

    int ret = robot.set_tool_data(tool_frame_id, &tool_frame, "ToolCoord");
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "set_toolFrame has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}


bool set_userFrame_callback(jaka_msgs::SetUserFrame::Request &request,
                           jaka_msgs::SetUserFrame::Response &response)
{
    CartesianPose user_frame;
    int user_frame_id = request.user_num; 
    user_frame.tran.x = request.pose[0];
    user_frame.tran.y = request.pose[1];
    user_frame.tran.z = request.pose[2];
    Eigen::Vector3d Angaxis = {request.pose[3],request.pose[4],request.pose[5]};
    RotMatrix Rot = Angaxis2Rot(Angaxis);
    robot.rot_matrix_to_rpy(&Rot, &(user_frame.rpy));
    // Eigen::AngleAxisd rotation_vector(Angaxis.norm(), Angaxis.normalized());
    // auto rpy = rotation_vector.matrix().eulerAngles(0, 1, 2);
    // user_frame.rpy.rx = rpy.x();
    // user_frame.rpy.ry = rpy.y();
    // user_frame.rpy.rz = rpy.z();
    int ret = robot.set_user_frame_data(user_frame_id, &user_frame, "BaseCoord");
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "set_userFrame has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}

bool set_payload_callback(jaka_msgs::SetPayload::Request &request,
                      jaka_msgs::SetPayload::Response &response)
{
    PayLoad payload;
    int tool_id = request.tool_num;

    payload.centroid.x = request.xc;
    payload.centroid.y = request.yc;
    payload.centroid.z = request.zc;
    payload.mass = request.mass;

    robot.set_tool_id(tool_id);
    int ret = robot.set_payload(&payload);

    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "set_payload has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;
}

bool drag_mode_callback(std_srvs::SetBool::Request &request,
                        std_srvs::SetBool::Response &response)
{
    int ret = robot.drag_mode_enable(request.data);
    switch(ret)
    {
        case 0:
            response.success = 1;
            response.message = "drag_mode has been executed";
            break;
        default:
            response.success = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }

    return true;

}

bool set_collisionLevel_callback(jaka_msgs::SetCollision::Request &request,
                           jaka_msgs::SetCollision::Response &response)
{
    int collision_level;
    if(request.is_enable = 0)
    {
        collision_level = 0;
    }
    else
    {
        if(request.value <= 25)
        {
            collision_level = 1;
        }
        else if(request.value <= 50)
        {
            collision_level = 2;
        }
        else if(request.value <= 75)
        {
            collision_level = 3;
        }
        else if(request.value <=100)
        {
            collision_level = 4;
        }
        else
        {
            collision_level = 5;
        }
    }
    int ret = robot.set_collision_level(collision_level);
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "Collision level" + to_string(collision_level) + " has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;

    }
    return true;
}


bool set_io_callback(jaka_msgs::SetIO::Request &request,
                     jaka_msgs::SetIO::Response &response)
{   
    IOType type;
    int ret;
    switch(request.type)
    {
        case 0:
            type = IO_CABINET;
            break;
        case 1:
            type = IO_TOOL;
            break;
        case 2:
            type = IO_EXTEND;
            break;
    }
    float value = request.value;
    string signal = request.signal;
    int index = request.index;
    if(signal == "digital")
    {      
        BOOL digital_value;
        if(value)
        {
            digital_value = TRUE;
        }
        else
        {
            digital_value = FALSE;
        }
        ret = robot.set_digital_output(type, index, digital_value);
    }
    else if(signal == "analog")
    {
        ret = robot.set_analog_output(type, index, value);
    }
    switch(ret)
    {
        case 0:
            response.ret = 1;
            response.message = "set IO has been executed";
            break;
        default:
            response.ret = 0;
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;
}



bool get_io_callback(jaka_msgs::GetIO::Request &request,
                     jaka_msgs::GetIO::Response &response)
{   
    IOType type;
    int ret;
    BOOL digital_result;
    float analog_result;
    switch(request.type)
    {
        case 0:
            type = IO_CABINET;
            break;
        case 1:
            type = IO_TOOL;
            break;
        case 2:
            type = IO_EXTEND;
            break;
    }
    string signal = request.signal;
    int index = request.index;
    int path = request.path;
    if(signal == "digital")
    {       
        if(path == 0)
        {
            ret = robot.get_digital_input(type, index, &digital_result);
        }
        else if(path == 1)
        {
            ret = robot.get_digital_output(type, index, &digital_result);
        }
        switch(ret)
        {
            case 0:
                response.value = float(digital_result);
                response.message = "get IO has been executed";
                break;
            default:
                response.value = -999999;
                response.message = "error occurred:" + mapErr[ret];
        }
        return true;
    }
    else if(signal == "analog")
    {
        if(path == 0)
        {
            ret = robot.get_analog_input(type, index, &analog_result);
        }
        else if(path == 1)
        {
            ret = robot.get_analog_output(type, index, &analog_result);
        }
        switch(ret)
        {
            case 0:
                response.value = analog_result;
                response.message = "get IO has been executed";
                break;
            default:
                response.value = -999999;
                response.message = "error occurred:" + mapErr[ret];

        }
    return true;
    }
    
}

bool get_fk_callback(jaka_msgs::GetFK::Request &request,
                     jaka_msgs::GetFK::Response &response)
{
    JointValue joint_pose;
    CartesianPose cartesian_pose;
    for(int i = 0; i < 6; i++)
    {
        joint_pose.jVal[i] = request.joint[i];
    }
    int ret = robot.kine_forward(&joint_pose, &cartesian_pose);
    switch(ret)
    {
        case 0:
            response.cartesian_pose.push_back(cartesian_pose.tran.x);
            response.cartesian_pose.push_back(cartesian_pose.tran.y);
            response.cartesian_pose.push_back(cartesian_pose.tran.z);
            response.cartesian_pose.push_back(cartesian_pose.rpy.rx);
            response.cartesian_pose.push_back(cartesian_pose.rpy.ry);
            response.cartesian_pose.push_back(cartesian_pose.rpy.rz);
            response.message = "get FK has been executed";
            break;
        default:
            float pose_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};
            for(int i = 0; i < 6; i++)
            {
                response.cartesian_pose.push_back(pose_init[i]);
            }
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;

}

bool get_ik_callback(jaka_msgs::GetIK::Request &request,
                     jaka_msgs::GetIK::Response &response)
{
    JointValue joint_pose;
    JointValue ref_joint;
    CartesianPose cartesian_pose;
    for(int i = 0; i < 6; i++)
    {
        ref_joint.jVal[i] = request.ref_joint[i];

    }
    cartesian_pose.tran.x = request.cartesian_pose[0];
    cartesian_pose.tran.y = request.cartesian_pose[1];
    cartesian_pose.tran.z = request.cartesian_pose[2];
    cartesian_pose.rpy.rx = request.cartesian_pose[3];
    cartesian_pose.rpy.ry = request.cartesian_pose[4];
    cartesian_pose.rpy.rz = request.cartesian_pose[5];
    int ret = robot.kine_inverse(&ref_joint, &cartesian_pose, &joint_pose);
    switch(ret)
    {
        case 0:
            for(int i = 0; i < 6; i++)
            {
                response.joint.push_back(joint_pose.jVal[i]);
            }
            response.message = "get IK has been executed";
            break;
        default:
            float joint_init[6] = {9999.0, 9999.0, 9999.0, 9999.0, 9999.0, 9999.0};
            for(int i = 0; i < 6; i++)
            {
                response.joint.push_back(joint_init[i]);
            }
            response.message = "error occurred:" + mapErr[ret];
            return false;
    }
    return true;

}


/*
bool clear_error_callback(jaka_msgs::ClearError::Request &request,
                         jaka_msgs::ClearError::Response &response)
{

    return true;
}
*/
/*
void tool_position_callback(ros::Publisher tool_position_pub)
{
    geometry_msgs::TwistStamped  tool_position;
    RobotStatus robotstatus;
    RotMatrix rot;
    Rpy rpy;
    robot.get_robot_status(&robotstatus);
    tool_position.twist.linear.x = robotstatus.cartesiantran_position[0];
    tool_position.twist.linear.y = robotstatus.cartesiantran_position[1];
    tool_position.twist.linear.z = robotstatus.cartesiantran_position[2];
    rpy.rx = robotstatus.cartesiantran_position[3];
    rpy.ry = robotstatus.cartesiantran_position[4];
    rpy.rz = robotstatus.cartesiantran_position[5];
    robot.rpy_to_rot_matrix(&rpy, &rot);
    // Eigen::Vector3d angaxis = Rot2Angaxis(rot);
    // tool_position.twist.angular.x = angaxis[0];
    // tool_position.twist.angular.y = angaxis[1];
    // tool_position.twist.angular.z = angaxis[2];
    tool_position.twist.angular.x = (rpy.rx )/PI*180;
    tool_position.twist.angular.y = (rpy.ry )/PI*180;
    tool_position.twist.angular.z = (rpy.rz )/PI*180;

    // Eigen::Vector3d eulerAngle(rpy.rx,rpy.ry,rpy.rz);
    // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
    // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));
    // Eigen::AngleAxisd rotation_vector;
    // rotation_vector=yawAngle*pitchAngle*rollAngle;
    // cout << "angle is: " << rotation_vector.angle() << endl;
    // cout << "axis is: " << rotation_vector.axis() << endl;
    // double angle = rotation_vector.angle();
    // Eigen::Vector3d axis = rotation_vector.axis();
    // Eigen::Vector3d v = angle * axis;
    // tool_position.twist.angular.x = v[0];
    // tool_position.twist.angular.y = v[1];
    // tool_position.twist.angular.z = v[2];
    
    tool_position.header.stamp = ros::Time::now();
    tool_position_pub.publish(tool_position);
}
*/
void tool_position_callback(ros::Publisher tool_position_pub)
{
    geometry_msgs::PoseStamped tool_position;
    RobotStatus robotstatus;
    RotMatrix rot;
    Rpy rpy;
    
    robot.get_robot_status(&robotstatus);
    tool_position.pose.position.x = robotstatus.cartesiantran_position[0];
    tool_position.pose.position.y = robotstatus.cartesiantran_position[1];
    tool_position.pose.position.z = robotstatus.cartesiantran_position[2];
    rpy.rx = robotstatus.cartesiantran_position[3];
    rpy.ry = robotstatus.cartesiantran_position[4];
    rpy.rz = robotstatus.cartesiantran_position[5];
    tf2::Quaternion q;
    q.setRPY(rpy.rx, rpy.ry, rpy.rz);
    tool_position.pose.orientation.x = q.x();
    tool_position.pose.orientation.y = q.y();
    tool_position.pose.orientation.z = q.z();
    tool_position.pose.orientation.w = q.w();
    tool_position.header.stamp = ros::Time::now();
    tool_position.header.frame_id = "base_link"; 
    tool_position_pub.publish(tool_position);
}
void joint_position_callback(ros::Publisher joint_position_pub)
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
    joint_position_pub.publish(joint_position);
}
void robot_states_callback(ros::Publisher robot_states_pub)
{
    jaka_msgs::RobotMsg robot_states;
    RobotStatus robotstatus;
    ProgramState programstate;
    BOOL in_pos = true;
    BOOL in_col = false;
    robot.is_in_pos(&in_pos);
    robot.is_in_collision(&in_col);
    robot.get_robot_status(&robotstatus);
    robot.get_program_state(&programstate);

    if(robotstatus.emergency_stop)
    {
        robot_states.motion_state = 2;
    }
    else if(robotstatus.errcode)
    {
        robot_states.motion_state = 4;
    }
    else if(in_pos && programstate == PROGRAM_IDLE && (!robotstatus.drag_status))
    {
        robot_states.motion_state = 0;
    }
    else if(programstate == PROGRAM_PAUSED)
    {
        robot_states.motion_state = 1;
    }
    else if((!in_pos) || programstate == PROGRAM_RUNNING || robotstatus.drag_status)
    {
        robot_states.motion_state = 3;
    }

    if(robotstatus.powered_on)
    {
        robot_states.power_state = 1;
 
    }
    else
    {
        robot_states.power_state = 0;
    }

    if(robotstatus.enabled)
    {
        robot_states.servo_state = 1;
    }
    else
    {
        robot_states.servo_state = 0;
    }

    if(in_col)
    {
        robot_states.collision_state = 1;
    }
    else
    {
        robot_states.collision_state = 0;
    }
    robot_states_pub.publish(robot_states);
}

void stop_jog_callback(const ros::TimerEvent &event)
{
     if (jog_count >= 1 && jog_count_temp == jog_count )
    {
        robot.jog_stop(-1);
        jog_count = 0;
        jog_count_temp = 0;
        jog_index_last = -1;
        ROS_INFO("jog stop");
        
    }
    jog_count_temp = jog_count;
}

void* get_conn_scoket_state(void* args){
    RobotStatus robot_status;
	
    while (ros::ok())
    {
        int ret = robot.get_robot_status(&robot_status);
		if (ret)
        {
            ROS_ERROR("get_robot_status error!!!");
        }
        else if(!robot_status.is_socket_connect)
		{
            ROS_ERROR("connect error!!!");
        }
        if(ret==0)
        {
            tool_position_callback(tool_position_pub);
            joint_position_callback(joint_position_pub);
            robot_states_callback(robot_state_pub);
        
        }
        ros::Duration(0.1).sleep(); 
    }    
}

int main(int argc, char *argv[])
{

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "jaka_driver");
    ros::NodeHandle nh;
    ros::Rate rate(125);
    // robot.login_in(argv[1]);
    string default_ip = "10.5.5.100";
    string robot_ip = nh.param("ip", default_ip);
    robot.login_in(robot_ip.c_str());
    robot.set_status_data_update_time_interval(100);
    robot.set_block_wait_timeout(120);
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
    //Joint-space first-order low-pass filtering in robot servo mode
    //robot.servo_move_use_joint_LPF(2);
    robot.servo_speed_foresight(15,0.03);

    //1.1 Linear motion (in customized user coordinate system)
    ros::ServiceServer linear_move_service = nh.advertiseService("/jaka_driver/linear_move", linear_move_callback);
    //1.2 Joint motion
    ros::ServiceServer joint_move_service = nh.advertiseService("/jaka_driver/joint_move", joint_move_callback);
    //1.3 Jog motion
    ros::ServiceServer jog_service = nh.advertiseService("/jaka_driver/jog", jog_callback);
    //1.4 Servo Position Control Mode Enable
    ros::ServiceServer servo_move_enable_service = nh.advertiseService("/jaka_driver/servo_move_enable", servo_move_enable_callback);
    //1.5 Servo-mode motion in Cartesian space
    ros::ServiceServer servo_p_service = nh.advertiseService("/jaka_driver/servo_p", servo_p_callback);
    //1.6 Joint space servo mode motion
    ros::ServiceServer servo_j_service = nh.advertiseService("/jaka_driver/servo_j", servo_j_callback);
    //1.7 stop motion
    ros::ServiceServer stop_move_service = nh.advertiseService("/jaka_driver/stop_move",stop_move_callback);
    //2.1 Setting tcp parameters
    ros::ServiceServer set_toolframe_service = nh.advertiseService("/jaka_driver/set_toolframe", set_toolFrame_callback);
    //2.2 Setting user coordinate system parameters
    ros::ServiceServer set_userframe_service = nh.advertiseService("/jaka_driver/set_userframe", set_userFrame_callback);
    //2.3 Set the center of gravity parameters of the robot arm load
    ros::ServiceServer set_payload_service = nh.advertiseService("/jaka_driver/set_payload", set_payload_callback);
    //2.4 Set free drive mode
    ros::ServiceServer drag_move_service = nh.advertiseService("/jaka_driver/drag_move", drag_mode_callback);
    //2.5 Set collision sensitivity
    ros::ServiceServer set_collisionlevel_service = nh.advertiseService("/jaka_driver/set_collisionlevel", set_collisionLevel_callback);
    //2.6 Set IO
    ros::ServiceServer set_io_service = nh.advertiseService("jaka_driver/set_io",set_io_callback);
    //2.7 Get IO
    ros::ServiceServer get_io_service = nh.advertiseService("jaka_driver/get_io",get_io_callback);
    //2.8 Find the positive solution
    ros::ServiceServer get_fk_service = nh.advertiseService("jaka_driver/get_fk", get_fk_callback);
    //2.9 Find the inverse solution
    ros::ServiceServer get_ik_service = nh.advertiseService("jaka_driver/get_ik", get_ik_callback);
    //3.1 End position pose status information reporting
    //ros::Publisher tool_position_pub = nh.advertise<geometry_msgs::TwistStamped>("/jaka_driver/tool_position", 10);
    //3.2 Joint status information reporting
    //ros::Publisher joint_position_pub = nh.advertise<sensor_msgs::JointState>("/jaka_driver/joint_position", 10);
    //3.3 Report robot event status information
   // ros::Publisher robot_state_pub = nh.advertise<jaka_msgs::RobotMsg>("/jaka_driver/robot_states", 10);

    //3.1 End position pose status information reporting
//    tool_position_pub = nh.advertise<geometry_msgs::TwistStamped>("/jaka_driver/tool_position", 10);
   tool_position_pub = nh.advertise<geometry_msgs::PoseStamped>("/jaka_driver/tool_position", 10);
    //3.2 Joint status information reporting
    joint_position_pub = nh.advertise<sensor_msgs::JointState>("/jaka_driver/joint_position", 10);
    //3.3 Report robot event status information
    robot_state_pub = nh.advertise<jaka_msgs::RobotMsg>("/jaka_driver/robot_states", 10);
    
    // Automatically stop robot jog and motion
    ros::Timer stop_jog = nh.createTimer(ros::Duration(3), stop_jog_callback, false);
    // Monitor network connection status
    pthread_t conn_state_thread;
    int ret = pthread_create(&conn_state_thread,NULL,get_conn_scoket_state,NULL);

    ROS_INFO("start");
    
    while(ros::ok())
    {   
        tool_position_callback(tool_position_pub);
        joint_position_callback(joint_position_pub);
        robot_states_callback(robot_state_pub);
        rate.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}