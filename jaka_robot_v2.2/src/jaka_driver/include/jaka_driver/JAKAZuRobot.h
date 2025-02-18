/**************************************************************************
 * 
 * Copyright (c) 2024 JAKA Robotics, Ltd. All Rights Reserved.
 * 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * For support or inquiries, please contact support@jaka.com.
 * 
 * File: JAKAZuRobot.h
 * @author star@jaka
 * @date Nov-30-2021 
 *  
**************************************************************************/


#ifndef _JAKAAPI_H_
#define _JAKAAPI_H_

#include <stdio.h>
#include <string>
#include <stdint.h>
#include "jkerr.h"
#include "jktypes.h"

#if defined(_WIN32) || defined(WIN32)
/**
    * @brief Constructor for the robotic arm control class
 */
#if __cpluscplus

#ifdef DLLEXPORT_API
#undef DLLEXPORT_API
#endif // DLLEXPORT_API

#ifdef DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllexport)
#else // DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllimport)
#endif // DLLEXPORT_EXPORTS

#else // __cpluscplus

#define DLLEXPORT_API

#endif // __cpluscplus

#elif defined(__linux__)

#define DLLEXPORT_API __attribute__((visibility("default")))

#else

#define DLLEXPORT_API

#endif // defined(_WIN32) || defined(WIN32)

class DLLEXPORT_API JAKAZuRobot
{
public:
	/**
	* @brief Robotic arm control class constructor
	*/
	JAKAZuRobot();

///@name general part
///@{
	/**
    * @brief Create a control handle for the robot
	*
    * @param ip IP address of the controller
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t login_in(const char *ip);

	/**
    * @brief Disconnect the controller. The connection with the cobot will be then terminated.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t login_out();

	/**
    * @brief Power on the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t power_on();

	/**
	* @brief Power off the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t power_off();

	/**
    * @brief  Shut down the control cabinet. The cobot must be disabled and powered off before executing this command.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t shut_down();

	/**
    * @brief Enable the cobot. The cobot must be powered on before executing this command.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t enable_robot();

	/**
    * @brief Disable the robot
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t disable_robot();

	/**
    * @brief Get current status of the cobot(A large struct contains as more data as possible).
	* @deprecated please check each inner data using corresponding interfaces instead of this one. 
	* @warning please note that config of OptionalInfoConfig.ini may affect data from port 10004 and data in RobotStatus without error
	* if you're not familiar with this, please keep OptionalInfoConfig.ini no changed as defalut setting. 
	*
    * @param status Robot status
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_robot_status(RobotStatus *status);

	/**
	* @brief Get current status of the cobot(A small struct only contains error/poweron/enable info).
	*
	* @param status Pointer for the returned cobot status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_robot_status_simple(RobotStatus_simple *status);

	/**
	* @brief Get current status of the cobot
	* @deprecated please use "get_robot_status_simple" instead
	*
	* @param state Pointer for the returned cobot status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_robot_state(RobotState* state);

	/**
    * @brief Get the Denavit–Hartenberg parameters of the cobot.
	*
	* @param dhParam Pointer of a varible to save the returned Denavit–Hartenberg parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_dh_param(DHParam* dh_param);

	/**
	* @brief Set installation (or mounting) angle of the cobot.
	*
    * @param angleX Rotation angle around the X-axis
    * @param angleZ Rotation angle around the Z-axis
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_installation_angle(double angleX, double angleZ);

	/**
	* @brief Get installation (or mounting) angle of the cobot.
	*
	* @param quat Pointer for the returned result in quaternion.
	* @param appang Pointer for the returned result in RPY.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_installation_angle(Quaternion* quat, Rpy* appang);

	/**
	* @brief Registor Callback funtion used when controller raise error	
	* @deprecated Only useful before SDK 2.1.12, check return code of each interface instead of using this
	*
	* @param func Callback function
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_error_handler(CallBackFuncType func);
///@}
	
///@name motion part
///@{

	/**
	* @brief Set timeout of motion block
	* @deprecated  Used only in SDK version before v2.1.12
	* 
	* @param seconds Timeout, unit: second
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_block_wait_timeout(float seconds);

	/**
	* @brief Set time interval of getting data via port 10004
	* @deprecated  Used only in SDK version before v2.1.12
	*
	* @param millisecond Time interval, unit: millisecond
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_status_data_update_time_interval(float millisecond);

	/**
	* @brief Jog a single joint or axis of the cobot to move in specified mode
	*
    * @param aj_num 0-based axis or joint number, indicating joint number 0-5 in joint space, and x, y, z, rx, ry, rz in Cartesian space
    * @param move_mode Robot motion mode, incremental motion or continuous motion
    * @param coord_type Robot motion coordinate system, tool coordinate system, base coordinate system (current world/user coordinate system) or joint space
    * @param vel_cmd Commanded velocity, unit rad/s for rotary axis or joint motion, mm/s for moving axis
    * @param pos_cmd Commanded position, unit rad for rotary axis or joint motion, mm for moving axis
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t jog(int aj_num, MoveMode move_mode, CoordType coord_type, double vel_cmd, double pos_cmd);

	/**
	* @brief Stop the ongoing jog movement.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t jog_stop(int num);

	/**
	* @brief Move the cobot in joint space, without consideration of the path of TCP in Cartesian space.
	*
    * @param joint_pos Target position for robot joint motion
    * @param move_mode Specifies the motion mode: incremental motion (relative motion) or absolute motion
    * @param is_block Set whether the interface is a blocking interface, TRUE for blocking interface, FALSE for non-blocking interface
    * @param speed Robot joint motion speed, unit: rad/s
    * @param acc Robot joint motion angular acceleration, unit: rad/s^2
    * @param tol Robot joint motion end point error, unit: mm
    * @param option_cond Optional parameters for robot joints, if not needed, the value can be left unassigned, just fill in a null pointer
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed, double acc = 90, double tol = 0, const OptionalCond *option_cond= nullptr);

	/**
	 * @brief joint move with single param
	 *
	 * @param param contains all necessary param
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t joint_move(MoveJParam param);

	/**
	* @brief Move the cobot in Cartesian space, the TCP will move linearly.
	*
	* @param end_pos The target position of the robot's end motion.
	* @param move_mode Specify the motion mode: incremental (relative) or absolute.
	* @param is_block Set if the interface is a blocking interface, TRUE for blocking interface FALSE for non-blocking interface.
	* @param speed Robot linear motion speed, unit: mm/s
	* @param acc Acceleration of the robot in mm/s^2.
	* @param tol The robot's endpoint error in mm.
	* @param option_cond robot joint optional parameters, if not needed, the value can not be assigned, fill in the empty pointer can be
	* @param ori_vel Attitude velocity, unit rad/s
	* @param ori_acc Attitude acceleration, unit rad/s^2.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t linear_move(const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel = 500, double tol = 0, const OptionalCond *option_cond = nullptr, double ori_vel=3.14, double ori_acc=12.56);

	/**
	 *@brief Move the cobot linearly in Cartesian space, with a packed parameter.
	 *
	 * @param param Packed parameter with all required inside.
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t linear_move(MoveLParam param);

	/**
	* @brief Move the cobot in Cartesan space and the TCP will move along the definded arc.
	*
	* @param end_pos The target position of the robot's end motion.
	* @param mid_pos The midpoint of the robot's end motion.
	* @param move_mode Specify the motion mode: incremental (relative) or absolute.
	* @param is_block Set if the interface is a blocking interface, TRUE for blocking interface FALSE for non-blocking interface.
	* @param speed the robot arc speed, unit: rad/s
	* @param acc The acceleration of the robot's arc motion, in rads/s^2.
	* @param tol endpoint error of the robot's circular motion, in millimeters.
	* @param option_cond Optional parameter for robot joint, if not needed, this value can not be assigned, just fill in the empty pointer.
	* @param circle_cnt Specifies the number of circles of the robot. A value of 0 is equivalent to circle_move.
	* @param circle_mode Specifies the mode of the robot's circular motion, the parameter explanation is as follows:
	- 0: Fixed to use the axis angle of rotation angle less than 180° from the start attitude to the end attitude for attitude change; (current program)
	- 1: Fixedly adopts the axis angle of the rotation angle from the start attitude to the termination attitude which is greater than 180° for attitude change;
	- 2: Selection of whether the angle is less than 180° or more than 180° is automatically chosen according to the midpoint attitude;
	- 3: The attitude pinch angle is always consistent with the arc axis. (Current whole circle motion)
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t circular_move(const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond = nullptr, int circle_cnt = 0, int circle_mode = 0);

	/**
	 * @brief Move the cobot in Cartesan space along the definded arc, with a packed parameter.
	 * 
	 * @param param Packed parameter with all required inside.
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t circular_move(MoveCParam param);

	/**
	* @brief Setting the robot run rate
	*
	* @param rapid_rate Value of the velociry rate, range from [0,1].
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_rapidrate(double rapid_rate);

	/**
	* @brief Get the robot runtime rate
	*
	* @param rapid_rate Pointer for the returned current velocity rate.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_rapidrate(double *rapid_rate);

	/**
	* @brief Define the data of tool with specified ID.
	*
	* @param id ID of the tool to be defined.
	* @param tcp Data of the tool to be set.
	* @param name Alias name of the tool to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tool_data(int id, const CartesianPose *tcp, const char *name);

	/**
	* @brief Switch to the tool with specified ID.
	*
	* @param id ID of the tool.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tool_id(const int id);

	/**
	* @brief Get ID of the tool currently used.
	*
	* @param id Pointer for the returned current tool ID.		
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_id(int *id);

	/**
	* @brief Get definition data of the tool with specified ID.
	*
	* @param id ID of the tool.
	* @param tcp Pointer for the returned tool data	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_data(int id, CartesianPose *tcp);

	/**
	* @brief Define the data of user frame with specified ID.
	*
	* @param id ID of the user frame.
	* @param user_frame Data of the user frame to be set.
	* @param name Alias	name of the user frame.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_user_frame_data(int id, const CartesianPose *user_frame, const char *name);

	/**
	* @brief Switch to the user frame with specified ID.
	*
	* @param id ID of the user frame.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_user_frame_id(const int id);

	/**
	* @brief Get ID of the user frame currently used.
	*
	* @param id Pointer for the returned current user frame ID.		
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_user_frame_id(int *id);

	/**
	* @brief Get definition data of the user frame with specified ID.
	*
	* @param id ID of the user frame.
	* @param tcp Pointer for the returned user frame data	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_user_frame_data(int id, CartesianPose *user_frame);

	/**
	* @brief Set payload for the cobot.
	*
	* @param payload payload data to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_payload(const PayLoad *payload);

	/**
	* @brief Get current payload of the cobot.
	*
	* @param payload Pointer for the returned payload data.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_payload(PayLoad *payload);

	/**
	* @brief Get the position of the end of the tool in the current setting.
	*
	* @param tcp_position Pointer for the returned TCP position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tcp_position(CartesianPose *tcp_position);

	/**
	* @brief Get current joint position.
	*
	* @param joint_position Pointer for the return joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_joint_position(JointValue *joint_position);

	/**
	 * @brief Check if the cobot is now in E-Stop state.
	 *
	 * @param estop Pointer for the returned result.	
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */	
	errno_t is_in_estop(BOOL *estop);

	/**
	* @brief Check if the cobot is now on soft limit.
	*
	* @param on_limit Pointer for the returned result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_on_limit(BOOL *on_limit);

	/**
	* @brief Check if the cobot has completed the motion (does not mean reached target).
	*
	* @param in_pos Pointer for the returned result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_in_pos(BOOL *in_pos);

	/**
	* @brief Stop all the ongoing movements of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t motion_abort();
	
	/**
	* @brief Get motion status of the cobot.
	*
	* @param status Pointer for the returned motion status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t get_motion_status(MotionStatus *status);

///@}

///@name TIO part
///@{

	/**
	* @brief Set voltage parameter for TIO of the cobot. It only takes effect for TIO with hardware version 3.
	*
	* @param vout_enable Option to enable voltage output. 0:turn off， 1:turn on.
	* @param vout_vol Option to set output voltage. 0: 24V, 1:12V.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tio_vout_param(int vout_enable, int vout_vol);

	/**
	* @brief Get voltage parameter of TIO of the cobot. It only takes effect for TIO with hardware version 3.
	* 
	* @param vout_enable Pointer for the returned voltage enabling option.
	* @param vout_vol Pointer for the returned output voltage option.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tio_vout_param(int* vout_enable, int* vout_vol);

	/**
	* @brief Add or modify the signal for TIO RS485 channels.
	*
	* @param sign_info Definition data of the signal.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t add_tio_rs_signal(SignInfo sign_info);

	/**
	* @brief Delete the specified signal for TIO RS485 channel.
	*
	* @param sig_name Signal name.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t del_tio_rs_signal(const char* sig_name);

	/**
	* @brief Send a command to the specified RS485 channel.
	*
	* @param chn_id ID of the RS485 channel in TIO.
	* @param data Command data.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t send_tio_rs_command(int chn_id, uint8_t* data,int buffsize);

	/**
	* @brief Get all the defined signals in TIO module.
	* 
	* @param sign_info_array Pointer for the returned signal list.
	* @param array_len Pointer for the size of the returned signal list.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_rs485_signal_info(SignInfo* sign_info_array, int* array_len);

	/**
	* @brief Set tio mode.
	*
	* @param pin_type tio type 0 for DI Pins, 1 for DO Pins, 2 for AI Pins
	* @param pin_type tio mode DI Pins: 0:0x00 DI2 is NPN,DI1 is NPN,1:0x01 DI2 is NPN,DI1 is PNP, 2:0x10 DI2 is PNP,DI1 is NPN,3:0x11 DI2 is PNP,DI1 is PNP
							 DO Pins: Low 8-bit data high 4-bit for DO2 configuration, low 4-bit for DO1 configuration, 0x0 DO for NPN output, 0x1 DO for PNP output, 0x2 DO for push-pull output, 0xF RS485H interface
							 AI Pins: 0: analog input function enable, RS485L disable, 1: RS485L interface enable, analog input function disable
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tio_pin_mode(int pin_type, int pin_mode);

	/**
	* @brief Get mode of the TIO pin of specified type.
	*
	* @param pin_type tio type 0 for DI Pins, 1 for DO Pins, 2 for AI Pins
	* @param pin_mode tio mode DI Pins: 0:0x00 DI2 is NPN,DI1 is NPN,1:0x01 DI2 is NPN,DI1 is PNP, 2:0x10 DI2 is PNP,DI1 is NPN,3:0x11 DI2 is PNP,DI1 is PNP
							 DO Pins: Low 8-bit data high 4-bit for DO2 configuration, low 4-bit for DO1 configuration, 0x0 DO for NPN output, 0x1 DO for PNP output, 0x2 DO for push-pull output, 0xF RS485H interface
							 AI Pins: 0: analog input function enable, RS485L disable, 1: RS485L interface enable, analog input function disable
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tio_pin_mode(int pin_type, int* pin_mode);

	/**
	* @brief Setup communication for specified RS485 channel.
	*
	* @param ModRtuComm When the channel mode is set to Modbus RTU, you need to specify the Modbus slave node ID additionally.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_rs485_chn_comm(ModRtuComm mod_rtu_com);

	/**
	* @brief Get RS485 commnunication setting.
	* 
	* @param mod_rtu_com Pointer for the returned communication settings.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_rs485_chn_comm(ModRtuComm* mod_rtu_com);

	/**
	* @brief Set the mode for specified RS485 channel.
	*
	* @param chn_id Channel id. 0 for RS485H, channel 1; 1 for RS485L, channel 2.
	* @param chn_mode Mode to indicate the usage of RS485 channel. 0 for Modbus RTU, 1 for Raw RS485, 2 for torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_rs485_chn_mode(int chn_id, int chn_mode);

	/**
	* @brief Get the mode of specified RS485 channel.
	* 
	* @param chn_id Channel id. 0: RS485H, channel 1; 1: RS485L, channel 2
	* @param chn_mode Pointer for the returned mode. 0: Modbus RTU, 1: Raw RS485, 2, torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_rs485_chn_mode(int chn_id, int* chn_mode);

///@}

///@name Trajectory recording & replay
///@{

	/**
	* @brief Set trajectory recording parameters for the cobot.
	*
	* @param para Trajectory recording parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_traj_config(const TrajTrackPara *para);

	/**
	* @brief Get trajectory recording parameters of the cobot.
	*
	* @param para Pointer for the returned trajectory recording parameters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_traj_config(TrajTrackPara *para);

	/**
	* @brief Acquisition track reproduction data control switch
	*
	* @param mode Trajectory recording (sampling) mode. TRUE to start trajectory recording (sampling) and FALSE to disable.
	* @param filename File name	to save the trajectory recording results.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_traj_sample_mode(const BOOL mode, char *filename);

	/**
	* @brief Get current trajectory recording status of the cobot.
	*
	* @param mode Pointer for the returned status.TRUE if it's now recording, FALSE if the data recording is over or not recording. 
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_traj_sample_status(BOOL *sample_status);

	/**
	* @brief Get all the existing trajectory recordings of the cobot.
	*
	* @param filename  Pointer for the returned trajectory recording files.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_exist_traj_file_name(MultStrStorType *filename);

	/**
	* @brief Rename the specified trajectory recording file.
	*
	* @param src Source file name of the trajectory recording.
	* @param dest Dest file name of the trajectory recording, the length must be no more than 100 characters.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rename_traj_file_name(const char *src, const char *dest);

	/**
	* @brief Delete the specified trajectory file.
	*
	* @param filename File name of the trajectory recording.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t remove_traj_file(const char *filename);

	/**
	* @brief  Generate program scripts from specified trajectory recording file.
	*
	* @param filename Trajectory recording file.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t generate_traj_exe_file(const char *filename);

///@}

///@name servo part
///@{
	/**
	* @brief Enable or disable servo mode for the cobot.
	*
	* @param enable Option to enable or disable the mode. TRUE to enable servo mode，FALSE to disable servo mode.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_enable(BOOL enable);

	/**
	 * @brief Check if the cobot is now in servo move mode.
	 *
	 * @param is_servo Pointer for the returned result.
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t is_in_servomove(BOOL *in_servo);

	/**
	* @brief Move the cobot to the specifed joint position in servo mode. It will only work when the cobot is already in servo 
	* move mode. The servo_j command will be processed within one interpolation cycle. To ensure the cobot moves smoothly, 
	* client should send next command immediately to avoid any time delay.
	*
	* @param joint_pos The target position of the robot joint motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	* @param step_num times the period, servo_j movement period for step_num * 8ms, where step_num> = 1
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num = 1);

	/**
	* @brief Move the cobot to the specifed Cartesian position in servo mode. Simalar with servo_j command, it will only 
	* work when the cobot is already in servo move mode and will be processed within one interpolation cycle. To ensure 
	* the cobot moves smoothly, client should send next command immediately to avoid any time delay.
	*
	* @param cartesian_pose The target position of the robot's Cartesian motion.
	* @param move_mode Specify the motion mode: incremental or absolute.
	* @param step_num times the period, servo_p movement period is step_num * 8ms, where step_num>=1
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num = 1);
	
	/**
	* @brief Disable the filter for servo move commands.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_none_filter();

	/**
	* @brief Set 1st-order low-pass filter for servo move. It will take effect for both servo_j and servo_p commands.
	*
	* @param cutoffFreq Cut-off frequency for low-pass filter.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_joint_LPF(double cutoffFreq);

	/**
	* @brief Set 3rd-order non-linear filter in joint space for servo move. It will take effect for both servo_j and 
	* servo_p commands.
	*
	* @param max_vr Joint speed limit, in unit deg/s
	* @param max_ar Joint acceleration limit, in unit deg/s^2
	* @param max_jr Joint jerk limit, in unit deg/s^3	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_joint_NLF(double max_vr, double max_ar, double max_jr);

	/**
	* @brief Set 3rd-order non-linear filter in Cartesian space for servo move. It will only take effect for servo_p
	* since the filter will be applied to Cartesian position in the commands.
	*
	* @param max_vp Speed limit, in unit mm/s.s
	* @param max_ap Acceleration limit, in unit mm/s^2.
	* @param max_jp Jerk limit, in unit mm/s^3.
	* @param max_vr Orientation speed limit, in unit deg/s.
	* @param max_ar Orientation acceleration limit, in unit deg/s^2.
	* @param max_jr Orientation jerk limit, in unit deg/s^3.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_carte_NLF(double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr);

	/**
	* @brief Set multi-order mean filter in joint space for servo move. It will take effect for both servo_j and servo_p
	* commands.
	*
	* @param max_buf Indicates the size of the mean filter buffer. If the filter buffer is set too small (less than 3), it
	* is likely to cause planning failure. The buffer value should not be too large (>100), which will bring computational
	* burden to the controller and cause planning delay; as the buffer value increases, the planning delay time increases.
	* @param kp Position filter coefficient.
	* @param kv Velocity filter coefficient.
	* @param ka Acceleration filter coefficient.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_move_use_joint_MMF(int max_buf, double kp, double kv, double ka);

	/**
	* @brief Set velocity look-ahead filter. It’s an extended version based on the multi-order filtering algorithm with 
	* look-ahead algorithm, which can be used for joints data and Cartesian data.
	* 
	* @param max_buf Buffer size of the mean filter. A larger buffer results in smoother results, but with higher precision
	* loss and longer planning lag time.
	* @param kp Position filter coefficient. Reducing this coefficient will result in a smoother filtering effect, but a 
	* greater loss in position accuracy. Increasing this coefficient will result in a faster response and higher accuracy,
	* but there may be problems with unstable operation/jitter, especially when the original data has a lot of noise.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t servo_speed_foresight(int max_buf, double kp);

///@}

///@name IO part
///@{
	/**
	* @brief Set the value of a digital output (DO).
	*
	* @param type DO type
	* @param index DO index
	* @param value DO set value
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_digital_output(IOType type, int index, BOOL value);

	/**
	* @brief Set the value of the analog output (AO).
	*
	* @param type AO type.
	* @param index AO index.
	* @param value AO set value.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_analog_output(IOType type, int index, float value);

	/**
	* @brief Query Digital Input (DI) Status.
	*
	* @param type DI type.
	* @param index DI index.
	* @param result DI status query result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_digital_input(IOType type, int index, BOOL *result);

	/**
	* @brief Query digital output (DO) status.
	*
	* @param type DO type.
	* @param index DO index.
	* @param result DO status query result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_digital_output(IOType type, int index, BOOL *result);

	/**
	* @brief Get the value of the analog input (AI).
	*
	* @param type The type of AI.
	* @param index AI index.
	* @param result Specify the result of AI status query.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_analog_input(IOType type, int index, float *result);

	/**
	* @brief Get the value of analog output (AO).
	*
	* @param type The type of AO.
	* @param index AO index.
	* @param result Specify the result of AO status query.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_analog_output(IOType type, int index, float *result);

	/**
	* @brief Set multiple digital outputs (DO), specified number of DOs starting from certain index will be set.
	*
	* @param type Type of the digital output.
	* @param index Starting index of the digital outputs to be set.
	* @param value Array of the target value.
	* @param len Number of DO to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_digital_output(IOType type, int index, BOOL* value, int len);

	/**
	* @brief Set multiple analog outputs (AO), specified number of AOs starting from certain index will be set.
	*
	* @param type Type of the analog output.
	* @param index Starting index of the analog outputs to be set.
	* @param value Array of the target value.
	* @param len Number of AO to be set.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_analog_output(IOType type, int index, float* value, int len);

	/**
	* @brief Get current status of multiple digital inputs (DI), specified number of DIs starting from certain index will be retrieved.
	*
	* @param type Type of the digital input.
	* @param index Starting index of the digital input.
	* @param result Pointer for the returned DI status.
	* @param len Number of DI to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_digital_input(IOType type, int index, BOOL *result, int len);

	/**
	* @brief Get current status of multiple digital inputs (DO), specified number of DOs starting from certain index will be retrieved.
	*
	* @param type Type of the digital output.
	* @param index Starting index of the digital outputs.
	* @param result Pointer for the returned DO status.
	* @param len Number of DO to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_digital_output(IOType type, int index, BOOL *result, int len);

	/**
	* @brief Get current value of multiple analog inputs (AI), specified number of AIs starting from certain index will be retrieved.
	*
	* @param type The type of AI
	* @param index Starting index of the analog inputs.
	* @param result Pointer for the returned AI status.
	* @param len Number of AI to be queried.

	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_analog_input(IOType type, int index, float *result, int len);

	/**
	* @brief Get the value of analog output variable (AO)
	*
	* @param type The type of AO.
	* @param index Starting index of the analog outputs.
	* @param result Pointer for the returned AO status.
	* @param len Number of AO to be queried.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_analog_output(IOType type, int index, float *result, int len);

	/**
	* @brief Check if the extended IO modules are running.
	*
	* @param is_running Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_extio_running(BOOL *is_running);


///@}

///@name program part
///@{
	/**
	* @brief Start the program for the cobot. It will work only when one program has been loaded.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_run();

	/**
	* @brief Pause the ongoing program for the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_pause();

	/**
	* @brief Resume the program for the cobot. It will work only when one program is now in paused status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_resume();

	/**
	* @brief Abort the ongoing tasks of the cobot, the program or any movement will be terminated.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_abort();

	/**
	* @brief Load a program for the cobot.
	*
	* @param file The path of the program. For example: A/A.jks, the <file> is "A"
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t program_load(const char *file);

	/**
	* @brief Get the name of the loaded job program.
	*
	* @param file Pointer for the returned loaded program path. For example: A/A.jks, the <file> is "A"
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_loaded_program(char *file);

	/**
	* @brief Get current execting line. It's the line number of the motion command for the program scripts. For movement 
	* commands sent via SDK, it's the motion ID defined in the movement.
	*
	* @param curr_line The current line number query result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_current_line(int *curr_line);

	/**
	* @brief Get the status of the robot's program execution.
	*
	* @param status Pointer for the returned program status.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_program_state(ProgramState *status);

	/**
	* @brief Get the user defined variables.
	 *
	 * @param vlist Pointer for the returned list of user defined variables.	
	 
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t get_user_var(UserVariableList* vlist);

	/**
	 * @brief Set the specified user defined variable.
	 *
	 * @param v Data of the user defined variable to be set, including the ID, value and alias name.	
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t set_user_var(UserVariable v);

///@}

///@name Hand-guiding (drag)
///@{
	/**
	* @brief Enable/disable the hand-guiding (drag) mode.
	*
	* @param enable TRUE to enter drag and drop mode, FALSE to exit drag and drop mode.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t drag_mode_enable(BOOL enable);

	/**
	* @brief Check if the cobot is in hand-guiding (drag) mode.
	*
	* @param in_drag Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_in_drag_mode(BOOL *in_drag);
///@}

///@name collision part
///@{

	/**
	* @brief Check if the cobot is in collision state.
	*
	* @param in_collision Pointer for the returned result.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t is_in_collision(BOOL *in_collision);

	/**
	* @brief Recover the cobot from collision state. It will only take effect when a cobot is in collision state.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t collision_recover();

	/**
	* @brief Clear error status.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t clear_error();

	/**
	* @brief Set the collision sensitivity level for the cobot.
	*
	* @param level  Collision sensitivity level, which is an integer value that ranges from [0-5]:
					0: disable collision detect
					1: collision detection threshold 25N，
					2: collision detection threshold 50N，
					3: collision detection threshold 75N，
					4: collision detection threshold 100N，
					5: collision detection threshold 125N	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_collision_level(const int level);

	/**
	* @brief Get current collision sensitivity level of the cobot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_collision_level(int *level);


///@}

///@name math
///@{
	/**
	* @brief Calculate the inverse kinematics for a Cartesian position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param ref_pos Reference joint position for solution selection, the result will be in the same solution space with
	* the reference joint position. 
	* @param cartesian_pose Cartesian position to do inverse kinematics calculation.
	* @param joint_pos Pointer for the returned joint position.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);

	/**
	* @brief Calculate the forward kinematics for a joint position. It will be calclulated with the current tool, current 
	* mounting angle, and current user coordinate.
	*
	* @param joint_pos The position of the joint in joint space.
	* @param cartesian_pose Cartesian space pose calculation result.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t kine_forward(const JointValue *joint_pos, CartesianPose *cartesian_pose);

	/**
	* @brief Convert an Euler angle in RPY to rotation matrix.
	*
	* @param rpy The Euler angle data to be converted.
	* @param rot_matrix The converted rotation matrix.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rpy_to_rot_matrix(const Rpy *rpy, RotMatrix *rot_matrix);

	/**
	* @brief Convert a rotation matrix to Euler angle in RPY
	*
	* @param rot_matrix The rotation matrix data to be converted.
	* @param rpy The result of the converted RPY Euler angles.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rot_matrix_to_rpy(const RotMatrix *rot_matrix, Rpy *rpy);

	/**
	* @brief Convert a quaternion to rotation matrix.
	*
	* @param quaternion The quaternion data to be converted.
	* @param rot_matrix Pointer for the returned rotation matrix.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t quaternion_to_rot_matrix(const Quaternion *quaternion, RotMatrix *rot_matrix);

	/**
	* @brief Conversion of a rotation matrix to quaternions
	*
	* @param rot_matrix The rotation matrix to be converted.
	* @param quaternion Pointer for the returned quaternion.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rot_matrix_to_quaternion(const RotMatrix *rot_matrix, Quaternion *quaternion);


///@}

///@name SDK support
///@{
	/**
	* @brief Set the reaction behavior of the cobot when connection to SDK is lost.
	*
	* @param millisecond Timeout of connection loss, unit: ms.
	* @param mnt Reaction behavior type.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_network_exception_handle(float millisecond, ProcessType mnt);

	/**
	* @brief Get the version number of SDK.
	*
	* @param version Pointer for the returned SDK verion.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_sdk_version(char *version);

	/**
	* @brief Get the IP address of the control cabinet.
	*
	* @param controller_name The controller name.
	* @param ip_list Controller ip list, controller name for the specific value to return the name of the corresponding controller IP address, controller name is empty, return to the segment class of all the controller IP address
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_controller_ip(char *controller_name, char *ip_list);

	/**
	* @brief Set the path to the error code file, if you need to use the get_last_error interface you need to set the path to the error code file, if you don't use the get_last_error interface, you don't need to set the interface.
	*
	* @param path  File path for the error code.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_errorcode_file_path(char *path);

	/**
	* @brief Get the last error code of the robot, the last error code will be cleared when clear_error is called.
	*
	* @param code Pointer for the returned error code.
	* 
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_last_error(ErrorCode *code);

	/**
	* @brief Enable or disable the debug mode for SDK control. If enabled, SDK log may contains more detailed information. 
	* @deprecated Only useful before SDK 2.1.12
	*
	* @param mode Option to enable or disable the debug mode. 1: enable, 0: disable.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_debug_mode(BOOL mode);

	/**
	* @brief Get the SDK log path.
	*
	* @param filepath Pointer for the returned path.
	* @param size Size of char* buffer
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_SDK_filepath(char* path, int size);

	/**
	* @brief Set file path for the SDK log.
	*
	* @param filepath File path of the log.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_SDK_filepath(const char *filepath);

	/**
	* @brief get SDK log path.
	*
	* @param path Path of SDK log.
	* @param size Size of char* buffer.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	static errno_t static_Get_SDK_filepath(char* path, int size);

	/**
	* @brief Set file path for the SDK log.
	*
	* @param filepath File path of the log.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	static errno_t static_Set_SDK_filepath(const char *filepath);



///@}

///@name Torque sensor and force control
///@{

	/**
	* @brief Set the type/brand of torque sensor.
	* 
	* @param sensor_brand Type/brand of the torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torsenosr_brand(int sensor_brand);

	/**
	* @brief Get the type/brand of torque sensor.
	* 
	* @param sensor_brand  Pointer to the returned torque sensor type/brand.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torsenosr_brand(int *sensor_brand);

	/**
	* @brief Set mode to turn on or turn off the torque sensor.
	* 
	* @param sensor_mode Mode of the torque sensor, 1 to turn on and 0 to turn off the torque sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_sensor_mode(int sensor_mode);

	/**
	* @brief Set parameters for admittance control of the cobot.
	* 
	* @param axis ID of the axis to be controlled, axis with ID 0 to 5 corresponds to x, y, z, Rx, Ry, Rz.
	* @param opt  Enable flag. 0: disable, 1: enable.
	* @param ftUser  Force to move the cobot in maximum speed.
	* @param ftReboundFK  Ability to go back to initial position.
	* @param ftConstant Set to 0 when operate manually.
	* @param ftNnormalTrack Set to 0 when operate manually.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_admit_ctrl_config(int axis, int opt, double ftUser, double ftConstant, int ftNnormalTrack, double ftReboundFK);

	/**
	* @brief Start to identify payload of the torque sensor.
	* 
	* @param joint_pos End joint position of the trajectory.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t start_torq_sensor_payload_identify(const JointValue *joint_pos);

	/**
	* @brief Get the status of torque sensor payload identification
	*
	* @param identify_status Pointer of the returned result. 0: done，1: in progress，2: error.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torq_sensor_identify_staus(int *identify_status);

	/**
	* @brief Get identified payload of the torque sensor.
	*
	* @param payload Pointer to the returned identified payload.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torq_sensor_payload_identify_result(PayLoad *payload);

	/**
	* @brief Set the payload for the torque sensor.
	*
	* @param payload Payload of torque sensor.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torq_sensor_tool_payload(const PayLoad *payload);

	/**
	* @brief Get current payload of the torque sensor.
	*
	* @param payload Pointer to the returned payload.	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torq_sensor_tool_payload(PayLoad *payload);

	/**
	* @brief Enable or disable the admittance control of the cobot. It will only work when a torque sensor is equiped.
	*
	* @param enable_flag Option to indicate enable or disable the admittance control. 1 to enable and 0 to disable
	* the admittance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t enable_admittance_ctrl(const int enable_flag);

	/**
	* @brief Enable or disable the tool drive. customer may drag robot using torque-sensor
	* 
	* @param handle Control handler of the cobot. 
	* @param enable_flag Option to indicate enable or disable: 1 to enable and 0 to disable
	* the admittance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t enable_tool_drive(const int enable_flag);

	/**
	* @brief Set tool drive configuration.
	* 
	* @param cfg Configuration for the tool drive.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tool_drive_config(ToolDriveConfig cfg);

	/**
	* @brief Get current configuration for tool drive.
	* 
	* @param cfg Pointer for the returned tool drive configuration.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_drive_config(RobotToolDriveCtrl* cfg);

	/**
	* @brief Set compliance control type and sensor initialization status.
	*
	* @param sensor_compensation Whether to enable sensor compensation, 1 means enable is initialized, 0 means not initialized.
	* @param compliance_type 0 for not using any kind of compliance control method 1 for constant force compliance control, 2 for speed compliance control
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_compliant_type(int sensor_compensation, int compliance_type);

	/**
	* @brief Get compliance control type and sensor initialization status.
	*
	* @param sensor_compensation Whether to enable sensor compensation, 1 means enable is initialized, 0 means not initialized.
	* @param compliance_type 0 for not using any kind of compliance control method 1 for constant force compliance control, 2 for speed compliance control
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_compliant_type(int *sensor_compensation, int *compliance_type);

	/**
	* @brief Get admitrance control configurations.
	*
	* @param admit_ctrl_cfg Pointer for the returned admittance control configurations.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_admit_ctrl_config(RobotAdmitCtrl *admit_ctrl_cfg);

	/**
	* @brief Setup the communication for the torque sensor.
	*
	* @param type Commnunication type of the torque sensor, 0: TCP/IP, 1: RS485.
	* @param ip_addr IP address of the torque sensor. Only for TCP/IP
	* @param port Port for the torque sensor. only for tcp/ip
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_sensor_comm(const int type, const char *ip_addr, const int port);

	/**
	* @brief Get the communication settings of the torque sensor.
	*
	* @param type Pointer for the returned commnunication type.
	* @param ip_addr Pointer for the returned IP address.
	* @param port Pointer for the returned port.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_sensor_comm(int *type, char *ip_addr, int *port);

	/**
	* @brief Set the torque sensor low-pass filter parameter for force control.
	*
	* @param torque_sensor_filter Filter parameter, unit：Hz
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_sensor_filter(const float torque_sensor_filter);

	/**
	* @brief Get the filter parameter of force control.
	*
	* @param torque_sensor_filter Pointer for the returned filter parameter, unit：Hz
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_sensor_filter(float *torque_sensor_filter);

	/**
	* @brief Set soft force or torque limit for the torque sensor.
	*
	* @param torque_sensor_soft_limit Soft limit, fx/fy/fz is force limit in unit N and tx/ty/tz is torque limit unit：N*m.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_sensor_soft_limit(const FTxyz torque_sensor_soft_limit);

	/**
	* @brief Get current soft force or torque limit of the torque sensor.
	*
	* @param torque_sensor_soft_limit Pointer for the returned soft limits.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_sensor_soft_limit(FTxyz *torque_sensor_soft_limit);

	/**
	* @brief Disabled force control of the coot.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t disable_force_control();

	/**
	* @brief Set the parameters for velocity complianance control.
	*
	* @param vel_cfg Prameters for velocity compliance control.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_vel_compliant_ctrl(const VelCom *vel_cfg);

	/**
	* @brief Set condition for compliance control.
	*
	* @param ft the max force, if over limit, the robot will stop movement
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_compliance_condition(const FTxyz *ft);


	/**
	* @brief Set the coordinate or frame for the force control.
	*
	* @param ftFrame Coordinate or frame option. 0: tool frame, 1:world frame.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_ft_ctrl_frame(const int ftFrame);

	/**
	* @brief Get the coordinate or frame of the force control.
	*
	* @param ftFrame Pointer for the returned frame option.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_ft_ctrl_frame(int* ftFrame);

	/**
	* @brief Get the torque sensor data of specified type.
	*
	* @param type Type of the data to be retrieved: 1 for actual feedback,  2 for general data,  3 for real data without gravity and bias.
	* @param data Pointer for the returned feedback data
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_sensor_data(int type, TorqSensorData* data);

	/**
	 * @brief Trigger sensor zeroing and blocking for 0.5 seconds
	 *
	 * @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	 */
	errno_t zero_end_sensor();

	/**
	* @brief Get the tool drive mode and state.
	*
	* @param enable Pointer for the returned value that indicating if the tool drive mode is enabled or not.
	* @param state Pointer for the returned value that indicating if current state of tool drive triggers singularity point, speed, joint limit warning.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_drive_state(int* enable, int *state);

	/**
	* @brief Get coordinate system for tool drive.
	*
	* @param ftFrame Pointer for the return coordinate system, 0 for tool coordinate and 1 for world coordinate.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_tool_drive_frame(FTFrameType *ftFrame);

	/**
	* @brief Set the the coordinate system for tool drive.
	*
	* @param ftFrame Coordinate system option for tool drive, 0 for tool coordinate and 1 for world coordinate.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_tool_drive_frame(FTFrameType ftFrame);

	/**
	* @brief Get the sensitivity setting for fusion drive.
	*
	* @param level Pointer for the returned sensitivity level,which ranges within [0,5] and 0 means off.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_fusion_drive_sensitivity_level(int *level);

	/**
	* @brief Set the sensitivity level for the fusion drive.
	*
	* @param level Sensitivity level, which ranges within [0,5] and 0 means off.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_fusion_drive_sensitivity_level(int level);

	/**
	* @brief Get the warning range of motion limit (singularity point and joint limit)
	*
	* @param range_level Range level, 1-5
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_motion_limit_warning_range(int *warningRange);

	/**
	* @brief Set the warning range for motion limit, like singularity point and joint limit.
	*
	* @param range_level Range level, 1-5.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_motion_limit_warning_range(int warningRange);

	/**
	* @brief Get force control speed limit.
	*
	* @param vel Line speed limit, mm/s.
	* @param angularVel Angular velocity limit, rad/s	
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_compliant_speed_limit(double* vel, double* angularVel);

	/**
	* @brief Set force control speed limit.
	*
	* @param speed_limit Line speed limit, mm/s.
	* @param angular_speed_limit Angular velocity limit, rad/s.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_compliant_speed_limit(double vel, double angularVel);

	/**
	* @brief Get the torque reference center.
	*
	* @param ref_point 0 represents the sensor center, 1 represents TCP
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_torque_ref_point(int *refPoint);

	/**
	* @brief Set the torque reference center.
	*
	* @param ref_point 0 represents the sensor center, 1 represents TCP
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_torque_ref_point(int refPoint);

	/**
	* @brief Get sensor sensitivity.
	*
	* @param threshold Torque or force threshold for each axis, ranging within [0, 1]. The larger the value, the less sensitive the sensor.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_end_sensor_sensitivity_threshold(FTxyz *threshold);

	/**
	* @brief Set the sensor sensitivity.
	*
	* @param threshold for each axis, 0~1, the larger the value, the less sensitive the sensor
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t set_end_sensor_sensitivity_threshold(FTxyz threshold);

///@}

///@name FTP part
///@{
	/**
	* @brief Establish ftp connection with controller.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t init_ftp_client();

	/**
	* @brief Establish an encrypted ftp connection with the controller (requires app login and controller version support)
	*
	* @param password Robot login password
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t init_ftp_client_with_ssl(char* password);

	/**
	* @brief Disconnect ftp from controller
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t close_ftp_client();
	/**
	* @brief Download a file of the specified type and name from the controller to the local
	*
	* @param remote The absolute path to the file name inside the controller.
	* @param local The absolute path to the file name to be downloaded locally.
	* @param opt 1 single file 2 folder
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t download_file(char* local, char* remote, int opt);

	/**
	* @brief Upload a file of a specified type and name from the controller to the local
	*
	* @param remote Absolute path of the file name to be uploaded inside the controller.
	* @param local Absolute path to local file name.
	* @param opt 1 single file 2 folder
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t upload_file(char* local, char* remote, int opt);


	/**
	* @brief Delete a file of the specified type and name from the controller.
	*
	* @param remote Controller internal file name
	* @param opt 1 single file 2 folder
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t del_ftp_file(char* remote, int opt);

	/**
	* @brief Rename a file of the type and name specified by the controller.
	*
	* @param remote Original name of the controller's internal file name
	* @param des The target name to rename.
	* @param opt 1 single file 2 folder
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t rename_ftp_file(char* remote, char* des, int opt);

	/**
	* @brief Get the directory of the FTP service. 
	*
	* @param remotedir Pointer for the returned FIP directory. like "/track/" or "/program/"
	* @param type Type of the file. 0: file and folder, 1: single file, 2: folder
	* @param ret Returned structure of the directory in string format.
	*
	* @return Indicate the status of operation. ERR_SUCC for success and other for failure.
	*/
	errno_t get_ftp_dir(const char* remotedir, int type, char* ret);
///@}

	~JAKAZuRobot();

private:
	class BIFClass;
	BIFClass *ptr;
};


#undef DLLEXPORT_API
#endif
