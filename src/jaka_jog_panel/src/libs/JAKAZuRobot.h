#ifndef _JAKAAPI_H_
#define _JAKAAPI_H_

#include "jkerr.h"
#include "jktypes.h"
#include <stdio.h>
#include <string>

class JAKAZuRobot
{
public:
	/**
	* @brief 机械臂控制类构造函数
	*/
	JAKAZuRobot();

	/**
	* @brief 创建机器人控制句柄
	* @param ip  控制器ip地址
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t login_in(const char *ip);

	/**
	* @brief 断开控制器连接
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t login_out();

	/**
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t power_on();

	/**
	* @brief 关闭机器人电源
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t power_off();

	/**
	* @brief 机器人控制柜关机
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t shut_down();

	/**
	* @brief 控制机器人上使能
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t enable_robot();

	/**
	* @brief 控制机器人下使能
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t disable_robot();

	/**
	* @brief 控制机器人手动模式下运动
	* @param aj_num 1_based标识值，在关节空间下代表关节号0-5，笛卡尔下依次为x，y，z，rx，ry，rz
	* @param move_mode 机器人运动模式，增量运动或者连续运动
	* @param coord_type 机器人运动坐标系，工具坐标系，基坐标系（当前的世界/用户坐标系）或关节空间
	* @param vel_cmd 指令速度，旋转轴或关节运动单位为rad/s，移动轴单位为mm/s
	* @param pos_cmd 指令位置，旋转轴或关节运动单位为rad，移动轴单位为mm
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t jog(int aj_num, MoveMode move_mode, CoordType coord_type, double vel_cmd, double pos_cmd);

	/**
	* @brief 控制机器人手动模式下运动停止
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t jog_stop(int num);

	/**
	* @brief 机器人关节运动
	* @param joint_pos 机器人关节运动目标位置
	* @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE为阻塞接口 FALSE为非阻塞接口
	* @param speed 机器人关节运动速度，单位：rad/s
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed);

	/**
	* @brief 机器人关节运动
	* @param joint_pos 机器人关节运动目标位置
	* @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE为阻塞接口 FALSE为非阻塞接口
	* @param speed 机器人关节运动速度，单位：rad/s
	* @param acc 机器人关节运动角加速度，单位：rad/s^2
	* @param tol 机器人关节运动终点误差,单位：mm
	* @param option_cond 机器人关节可选参数，如果不需要，该值可不赋值,填入空指针就可
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed, double acc, double tol, const OptionalCond *option_cond);

	/**
	* @brief 机器人末端直线运动
	* @param end_pos 机器人末端运动目标位置
	* @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE 为阻塞接口 FALSE 为非阻塞接口
	* @param speed 机器人直线运动速度，单位：mm/s
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t linear_move(const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed);

	/**
	* @brief 机器人末端直线运动
	* @param end_pos 机器人末端运动目标位置
	* @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE 为阻塞接口 FALSE 为非阻塞接口
	* @param speed 机器人直线运动速度，单位：mm/s
	* @param acc 机器人直线运动加速度,单位mm/s^2
	* @param tol 机器人关节运动终点误差，单位mm
	* @param option_cond 机器人关节可选参数，如果不需要，该值可不赋值,填入空指针就可
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t linear_move(const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond);

	/**
	* @brief 机器人末端圆弧运动
	* @param end_pos 机器人末端运动目标位置
	* @param mid_pos 机器人末端运中间点
	* @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE 为阻塞接口 FALSE 为非阻塞接口
	* @param speed 机器人圆弧运动速度，单位：rad/s
	* @param acc 机器人圆弧运动角加速度，单位：rad/s^2
	* @param tol 机器人圆弧运动终点误差, 单位mm
	* @param option_cond 机器人关节可选参数，如果不需要，该值可不赋值,填入空指针就可
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t circular_move(const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond);

	/**
	* @brief 机器人SERVO MOVE模式使能
	* @param enable  TRUE为进入SERVO MOVE模式，FALSE表示退出该模式
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_move_enable(BOOL enable);

	/**
	* @brief 机器人关节空间位置控制模式
	* @param joint_pos 机器人关节运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @return ERR_SUCC成功 其他失败
	*/
	errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode);

	/**
	* @brief 机器人关节空间位置控制模式
	* @param joint_pos 机器人关节运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @step_num  倍分周期，servo_j运动周期为step_num*8ms，其中step_num>=1
	* @return ERR_SUCC成功 其他失败
	*/
	errno_t servo_j(const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num);

	/**
	* @brief 机器人笛卡尔空间位置控制模式
	* @param cartesian_pose 机器人笛卡尔空间运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode);

	/**
	* @brief 机器人笛卡尔空间位置控制模式
	* @param cartesian_pose 机器人笛卡尔空间运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @step_num  倍分周期，servo_p运动周期为step_num*8ms，其中step_num>=1
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_p(const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num);

	/**
	* @brief 设置数字输出变量(DO)的值
	* @param type DO类型
	* @param index DO索引
	* @param value DO设置值
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_digital_output(IOType type, int index, BOOL value);

	/**
	* @brief 设置模拟输出变量的值(AO)的值
	* @param type AO类型
	* @param index AO索引
	* @param value AO设置值
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_analog_output(IOType type, int index, float value);

	/**
	* @brief 查询数字输入(DI)状态
	* @param type DI类型
	* @param index DI索引
	* @param result DI状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_digital_input(IOType type, int index, BOOL *result);

	/**
	* @brief 查询数字输出(DO)状态
	* @param type DO类型
	* @param index DO索引
	* @param result DO状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_digital_output(IOType type, int index, BOOL *result);

	/**
	* @brief 获取模拟量输入变量(AI)的值
	* @param type AI的类型
	* @param index AI索引
	* @param result 指定AI状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_analog_input(IOType type, int index, float *result);

	/**
	* @brief 获取模拟量输出变量(AO)的值
	* @param type AO的类型
	* @param index AO索引
	* @param result 指定AO状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_analog_output(IOType type, int index, float *result);

	/**
	* @brief 查询扩展IO模块是否运行
	* @param is_running 扩展IO模块运行状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t is_extio_running(BOOL *is_running);

	/**
	* @brief 运行当前加载的作业程序
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t program_run();

	/**
	* @brief 暂停当前运行的作业程序
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t program_pause();

	/**
	* @brief 继续运行当前暂停的作业程序
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t program_resume();

	/**
	* @brief 终止当前执行的作业程序
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t program_abort();

	/**
	* @brief 加载指定的作业程序
	* @param file 程序文件路径
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t program_load(const char *file);

	/**
	* @brief 获取已加载的作业程序名字
	* @param file 程序文件路径
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_loaded_program(char *file);

	/**
	* @brief 获取当前机器人作业程序的执行行号
	* @param curr_line 当前行号查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_current_line(int *curr_line);

	/**
	* @brief 获取机器人作业程序执行状态
	* @param status 作业程序执行状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_program_state(ProgramState *status);

	/**
	* @brief 设置机器人运行倍率
	* @param rapid_rate 是程序运行倍率，设置范围为[0,1]
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_rapidrate(double rapid_rate);

	/**
	* @brief 获取机器人运行倍率
	* @param rapid_rate 当前控制系统倍率
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_rapidrate(double *rapid_rate);

	/**
	* @brief 设置指定编号的工具信息
	* @param id 工具编号
	* @param tcp 工具坐标系相对法兰坐标系偏置
	* @param name 指定工具的别名
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_tool_data(int id, const CartesianPose *tcp, const char *name);

	/**
	* @brief 设置当前使用的工具ID
	* @param id 工具坐标系ID
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_tool_id(const int id);

	/**
	* @brief 查询当前使用的工具ID
	* @param id 工具ID查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_tool_id(int *id);

	/**
	* @brief 查询当前使用的工具信息
	* @param id 工具ID查询结果
	* @param tcp 工具坐标系相对法兰坐标系偏置
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_tool_data(int id, CartesianPose *tcp);

	/**
	* @brief 设置指定编号的用户坐标系信息
	* @param id 用户坐标系编号
	* @param user_frame 用户坐标系偏置值
	* @param name 用户坐标系别名
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_user_frame_data(int id, const CartesianPose *user_frame, const char *name);

	/**
	* @brief 设置当前使用的用户坐标系ID
	* @param id 用户坐标系ID
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_user_frame_id(const int id);

	/**
	* @brief 查询当前使用的用户坐标系ID
	* @param id 获取的结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_user_frame_id(int *id);

	/**
	* @brief 查询当前使用的用户坐标系信息
	* @param id 用户坐标系ID查询结果
	* @param tcp 用户坐标系偏置值
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_user_frame_data(int id, CartesianPose *tcp);

	/**
	* @brief 控制机器人进入或退出拖拽模式
	* @param enable  TRUE为进入拖拽模式，FALSE为退出拖拽模式
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t drag_mode_enable(BOOL enable);

	/**
	* @brief 查询机器人是否处于拖拽模式
	* @param in_drag 查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t is_in_drag_mode(BOOL *in_drag);

	/**
	* @brief 获取机器人状态
	* @param state 机器人状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_robot_state(RobotState *state);

	/**
	* @brief 获取当前设置下工具末端的位姿
	* @param tcp_position 工具末端位置查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_tcp_position(CartesianPose *tcp_position);

	/**
	* @brief 获取当前机器人关节角度
	* @param joint_position 关节角度查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_joint_position(JointValue *joint_position);

	/**
	* @brief 查询机器人是否处于碰撞保护模式
	* @param in_collision 查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t is_in_collision(BOOL *in_collision);

	/**
	* @brief 查询机器人是否超出限位
	* @param on_limit 查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t is_on_limit(BOOL *on_limit);

	/**
	* @brief 查询机器人运动是否停止
	* @param in_pos 查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t is_in_pos(BOOL *in_pos);

	/**
	* @brief 碰撞之后从碰撞保护模式恢复
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t collision_recover();

	/**
	* @brief  错误状态清除
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t clear_error();

	/**
	* @brief 设置机器人碰撞等级
	* @param level  碰撞等级，等级0-5 ，0为关闭碰撞，1为碰撞阈值25N，2为碰撞阈值50N，3为碰撞阈值75N，4为碰撞阈值100N，5为碰撞阈值125N
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_collision_level(const int level);

	/**
	* @brief 获取机器人设置的碰撞等级
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_collision_level(int *level);

	/**
	* @brief 计算指定位姿在当前工具、当前安装角度以及当前用户坐标系设置下的逆解
	* @param ref_pos 逆解计算用的参考关节空间位置
	* @param cartesian_pose 笛卡尔空间位姿值
	* @param joint_pos 计算成功时关节空间位置计算结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t kine_inverse(const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);

	/**
	* @brief 计算指定关节位置在当前工具、当前安装角度以及当前用户坐标系设置下的位姿值
	* @param joint_pos 关节空间位置
	* @param cartesian_pose 笛卡尔空间位姿计算结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t kine_forward(const JointValue *joint_pos, CartesianPose *cartesian_pose);

	/**
	* @brief 欧拉角到旋转矩阵的转换
	* @param rpy 待转换的欧拉角数据
	* @param rot_matrix 转换后的旋转矩阵
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t rpy_to_rot_matrix(const Rpy *rpy, RotMatrix *rot_matrix);

	/**
	* @brief 旋转矩阵到欧拉角的转换
	* @param rot_matrix 待转换的旋转矩阵数据
	* @param rpy 转换后的RPY欧拉角结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t rot_matrix_to_rpy(const RotMatrix *rot_matrix, Rpy *rpy);

	/**
	* @brief 四元数到旋转矩阵的转换
	* @param quaternion 待转换的四元数数据
	* @param rot_matrix 转换后的旋转矩阵结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t quaternion_to_rot_matrix(const Quaternion *quaternion, RotMatrix *rot_matrix);

	/**
	* @brief 旋转矩阵到四元数的转换
	* @param rot_matrix 待转换的旋转矩阵
	* @param quaternion 转换后的四元数结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t rot_matrix_to_quaternion(const RotMatrix *rot_matrix, Quaternion *quaternion);

	/**
	* @brief 注册机器人出现错误时的回调函数
	* @param func 指向用户定义的函数的函数指针
	* @param error_code 机器人的错误码
	*/
	errno_t set_error_handler(CallBackFuncType func);

	/**
	* @brief 机器人力矩控制模式使能
	* @param enable  TRUE为进入SERVO MOVE模式，FALSE表示退出该模式
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t torque_control_enable(BOOL enable);

	/**
	* @brief 机器人力矩前馈功能
	* @param tor_val  用于力矩前馈的各个关节的力矩值
	* @param grv_flag  0代表选用控制器自带的力矩前馈算法，1代表设置关节力矩控制的偏置，2代表关节控制的力矩完全由用户控制
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t torque_feedforward(TorqueValue tor_val, int grv_flag);

	/**
	* @brief 机器人负载设置
	* @param payload 负载质心、质量数据
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_payload(const PayLoad *payload);

	/**
	* @brief 获取机器人负载数据
	* @param payload 负载查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_payload(PayLoad *payload);

	/**
	* @brief 获取SDK版本号
	* @param version SDK版本号
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_sdk_version(char *version);

	/**
	* @brief 获取控制器IP
	* @param controller_name 控制器名字
	* @param ip_list 控制器ip列表，控制器名字为具体值时返回该名字所对应的控制器IP地址，控制器名字为空时，返回网段类内的所有控制器IP地址
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_controller_ip(char *controller_name, char *ip_list);

	/**
	* @brief 获取机器人状态数据
	* @param status 机器人状态
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_robot_status(RobotStatus *status);

	/**
	* @brief 终止当前机械臂运动
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t motion_abort();

	/**
	* @brief 设置错误码文件路径，需要使用get_last_error接口时需要设置错误码文件路径，如果不使用get_last_error接口，则不需要设置该接口
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_errorcode_file_path(char *path);

	/**
	* @brief 获取机器人运行过程中最后一个错误码,当调用clear_error时，最后一个错误码会清零
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_last_error(ErrorCode *code);

	/**
	* @brief 设置是否开启调试模式，选择TRUE时，开始调试模式，此时会在标准输出流中输出调试信息，选择FALSE时，不输出调试信息
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_debug_mode(BOOL mode);

	/**
	* @brief 设置轨迹复现配置参数
	* @param para 轨迹复现配置参数
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_traj_config(const TrajTrackPara *para);

	/**
	* @brief 获取轨迹复现配置参数
	* @param para 轨迹复现配置参数
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_traj_config(TrajTrackPara *para);

	/**
	* @brief 采集轨迹复现数据控制开关
	* @param mode 选择TRUE时，开始数据采集，选择FALSE时，结束数据采集
	* @param filename 采集数据的存储文件名，当filename为空指针时，存储文件以当前日期命名
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_traj_sample_mode(const BOOL mode, char *filename);

	/**
	* @brief 采集轨迹复现数据状态查询
	* @param mode 为TRUE时，数据正在采集，为FALSE时，数据采集结束，在数据采集状态时不允许再次开启数据采集开关
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_traj_sample_status(BOOL *sample_status);

	/**
	* @brief 查询控制器中已经存在的轨迹复现数据的文件名
	* @param filename 控制器中已经存在的轨迹复现数据的文件名
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_exist_traj_file_name(MultStrStorType *filename);

	/**
	* @brief 重命名轨迹复现数据的文件名
	* @param src 原文件名
	* @param dest 目标文件名，文件名长度不能超过100个字符，文件名不能为空，目标文件名不支持中文
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t rename_traj_file_name(const char *src, const char *dest);

	/**
	* @brief 删除控制器中轨迹复现数据文件
	* @param filename 要删除的文件的文件名，文件名为数据文件名字
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t remove_traj_file(const char *filename);

	/**
	* @brief 控制器中轨迹复现数据文件生成控制器执行脚本
	* @param filename 数据文件的文件名，文件名为数据文件名字，不带后缀
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t generate_traj_exe_file(const char *filename);

	/**
	* @brief SERVO模式下不使用滤波器,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_move_use_none_filter();

	/**
	* @brief SERVO模式下关节空间一阶低通滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param cutoffFreq 一阶低通滤波器截止频率
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_move_use_joint_LPF(double cutoffFreq);

	/**
	* @brief SERVO模式下关节空间非线性滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param max_vr 笛卡尔空间姿态变化速度的速度上限值（绝对值）°/s
	* @param max_ar 笛卡尔空间姿态变化速度的加速度上限值（绝对值）°/s^2
	* @param max_jr 笛卡尔空间姿态变化速度的加加速度上限值（绝对值）°/s^3
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_move_use_joint_NLF(double max_vr, double max_ar, double max_jr);

	/**
	* @brief SERVO模式下笛卡尔空间非线性滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param max_vp 笛卡尔空间下移动指令速度的上限值（绝对值）。单位：mm/s
	* @param max_ap 笛卡尔空间下移动指令加速度的上限值（绝对值）。单位：mm/s^2
	* @param max_jp 笛卡尔空间下移动指令加加速度的上限值（绝对值）单位：mm/s^3
	* @param max_vr 笛卡尔空间姿态变化速度的速度上限值（绝对值）°/s
	* @param max_ar 笛卡尔空间姿态变化速度的加速度上限值（绝对值）°/s^2
	* @param max_jr 笛卡尔空间姿态变化速度的加加速度上限值（绝对值）°/s^3
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_move_use_carte_NLF(double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr);

	/**
	* @brief SERVO模式下关节空间多阶均值滤波器,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param max_buf 均值滤波器缓冲区的大小
	* @param kp 加速度滤波系数
	* @param kv 速度滤波系数
	* @param ka 位置滤波系数
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_move_use_joint_MMF(int max_buf, double kp, double kv, double ka);

	/**
	* @brief SERVO模式下速度前瞻参数设置
	* @param max_buf 缓冲区的大小
	* @param kp 加速度滤波系数
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t servo_speed_foresight(int max_buf, double kp);

	/**
	* @brief 设置SDK日志路径
	* @param filepath SDK日志路径
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_SDK_filepath(char *filepath);

	/**
	* @brief 设置传感器品牌
	* @param sensor_brand 传感器品牌，可选值为1,2,3 分别代表不同品牌力矩传感器
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_torsenosr_brand(int sensor_brand);

	/**
	* @brief 获取传感器品牌
	* @param sensor_brand 传感器品牌，可选值为1,2,3 分别代表不同品牌力矩传感器
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_torsenosr_brand(int *sensor_brand);

	/**
	* @brief 开启或关闭力矩传感器
	* @param sensor_mode 0代表关闭传感器，1代表开启力矩传感器
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_torque_sensor_mode(int sensor_mode);

	/**
	* @brief 设置柔顺控制参数
	* @param axis 代表配置哪一轴，可选值为0~5
	* @param opt 柔顺方向，可选值为 1 2 3 4 5 6分别对应 fx fy fz mx my mz 0代表没有勾选
	* @param ftUser 阻尼力，表示用户用多大的力才能让机器人的沿着某个方向以最大速度进行运动
	* @param ftReboundFK 回弹力，表示机器人回到初始状态的能力
	* @param ftConstant 代表恒力，手动操作时全部设置为0
	* @param ftNnormalTrack 法向跟踪，手动操作时全部设置为0,
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_admit_ctrl_config(int axis, int opt, double ftUser, double ftConstant, int ftNnormalTrack, double ftReboundFK);

	/**
	* @brief 开始辨识工具末端负载
	* @param joint_pos 使用力矩传感器进行自动负载辨识时的结束位置
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t start_torq_sensor_payload_identify(const JointValue *joint_pos);

	/**
	* @brief 获取末端负载辨识状态
	* @param identify_status 0代表辨识完成，1代表未完成，2代表辨识失败
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_torq_sensor_identify_staus(int *identify_status);

	/**
	* @brief 获取末端负载辨识结果
	* @param payload 末端负载
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_torq_sensor_payload_identify_result(PayLoad *payload);

	/**
	* @brief 设置传感器末端负载
	* @param payload 末端负载
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_torq_sensor_tool_payload(const PayLoad *payload);

	/**
	* @brief 获取传感器末端负载
	* @param payload 末端负载
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_torq_sensor_tool_payload(PayLoad *payload);

	/**
	* @brief 力控拖拽使能
	* @param enable_flag 0为关闭力控拖拽使能，1为开启
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t enable_admittance_ctrl(const int enable_flag);

	/**
	* @brief 设置力控类型和传感器初始化状态
	* @param sensor_compensation 是否开启传感器补偿,1代表开启即初始化,0代表不初始化
	* @param compliance_type 0 代表不使用任何一种柔顺控制方法 1 代表恒力柔顺控制,2 代表速度柔顺控制
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_compliant_type(int sensor_compensation, int compliance_type);

	/**
	* @brief 获取力控类型和传感器初始化状态
	* @param sensor_compensation 是否开启传感器补偿,1代表开启即初始化,0代表不初始化
	* @param compliance_type 0 代表不使用任何一种柔顺控制方法 1 代表恒力柔顺控制,2 代表速度柔顺控制
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_compliant_type(int *sensor_compensation, int *compliance_type);

	/**
	* @brief 获取力控柔顺控制参数
	* @param admit_ctrl_cfg 机器人力控柔顺控制参数存储地址
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_admit_ctrl_config(RobotAdmitCtrl *admit_ctrl_cfg);

	/**
	* @brief 设置力控传感器ip地址
	* @param type 0为使用tcp/ip协议，1为使用RS485协议
	* @param ip_addr为力控传感器地址
	* @param port为使用tcp/ip协议时力控传感器端口号
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_torque_sensor_comm(const int type, const char* ip_addr, const int port);

	/**
	* @brief 获取力控传感器ip地址
	* @param type 0为使用tcp/ip协议，1为使用RS485协议
	* @param ip_addr为力控传感器地址
	* @param port为使用tcp/ip协议时力控传感器端口号
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t get_torque_sensor_comm(int* type, char* ip_addr,int* port);

	/**
	* @brief 关闭力控
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t disable_force_control();

	/**
	* @brief 设置速度柔顺控制参数
	* @param vel_cfg为速度柔顺控制参数
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_vel_compliant_ctrl(const VelCom* vel_cfg);

	/**
	* @brief 设置柔顺控制力矩条件
	* @param ft为柔顺控制力矩条件
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_compliance_condition(const FTxyz* ft);

	/**
	* @brief 设置网络异常，SDK与机器人控制器失去连接后多长时间机器人控制器终止机械臂当前运动
	* @param millisecond 时间参数，单位毫秒
	* @param mnt 网络异常时机器人需要进行的动作类型
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_network_exception_handle(float millisecond, ProcessType mnt);

	/**
	* @brief 设置机器人状态数据自动更新时间间隔
	* @param millisecond 时间参数，单位毫秒
	* @return ERR_SUCC 成功 其他失败
	*/
	errno_t set_status_data_update_time_interval(float millisecond);

	~JAKAZuRobot();

private:
	class BIFClass;
	BIFClass *ptr;
};

#endif
