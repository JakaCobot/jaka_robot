	/**
	* @last update Nov 30 2021 
	* @Maintenance star@jaka
	*/

#ifndef _JAKA_API_
#define _JAKA_API_

#include "jkerr.h"
#include "jktypes.h"

#if _WIN32 || WIN32

//#ifdef __cpluscplus

#ifdef DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllexport)
#else // DLLEXPORT_EXPORTS
#define DLLEXPORT_API __declspec(dllimport)
#endif // DLLEXPORT_EXPORTS

//#endif // __cpluscplus

#else // _WIN32 || WIN32 

#define DLLEXPORT_API

#endif // _WIN32 || WIN32

//#ifdef __cpluscplus
extern "C"
{
//#endif // __cpluscplus

	/**
	* @brief 创建机器人控制句柄
	* @param ip  控制器ip地址
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t create_handler(const char *ip, JKHD *handle);

	/**
	* @brief 断开控制器连接
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t destory_handler(const JKHD *handle);

	/**
	* @brief 打开机器人电源
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t power_on(const JKHD *handle);

	/**
	* @brief 关闭机器人电源
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t power_off(const JKHD *handle);

	/**
	* @brief 机器人控制柜关机
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t shut_down(const JKHD *handle);

	/**
	* @brief 控制机器人上使能
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t enable_robot(const JKHD *handle);

	/**
	* @brief 控制机器人下使能
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t disable_robot(const JKHD *handle);

	/**
	* @brief 控制机器人手动模式下运动
	* @param handle  机器人控制句柄
	* @param aj_num 1_based标识值，在关节空间下代表关节号，笛卡尔下依次为x，y，z，rx，ry，rz
	* @param move_mode 机器人运动模式，增量运动、绝对运动或者连续运动
	* @param coord_type 机器人运动坐标系，工具坐标系，基坐标系（当前的世界/用户坐标系）或关节空间
	* @param vel_cmd 指令速度，旋转轴或关节运动单位为rad/s，移动轴单位为mm/s
	* @param pos_cmd 指令位置，旋转轴或关节运动单位为rad，移动轴单位为mm
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t jog(const JKHD *handle, int aj_num, MoveMode move_mode, CoordType coord_type, double vel_cmd, double pos_cmd);

	/**
	* @brief 控制机器人手动模式下运动停止
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t jog_stop(const JKHD *handle, int num);

	/**
	* @brief 机器人关节运动
	* @param handle  机器人控制句柄
	* @param joint_pos 机器人关节运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE为阻塞接口 FALSE为非阻塞接口
	* @param speed 机器人关节运动速度，单位：rad/s
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t joint_move(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed);

	/**
	* @brief 机器人关节运动
	* @param joint_pos 机器人关节运动目标位置
	* @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE为阻塞接口 FALSE为非阻塞接口
	* @param speed 机器人关节运动速度，单位：rad/s
	* @param acc 机器人关节运动角加速度,单位：rad/s^2
	* @param tol 机器人关节运动终点误差,单位：mm
	* @param option_cond 机器人关节可选参数，如果不需要，该值可不赋值,填入空指针就可
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t joint_move_extend(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode, BOOL is_block, double speed, double acc, double tol, const OptionalCond *option_cond);

	/**
	* @brief 机器人末端直线运动
	* @param handle  机器人控制句柄
	* @param end_pos 机器人末端运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE 为阻塞接口 FALSE 为非阻塞接口
	* @param speed 机器人直线运动速度，单位：mm/s
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t linear_move(const JKHD *handle, const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed);

	/**
	* @brief 机器人末端直线运动
	* @param end_pos 机器人末端运动目标位置
	* @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE 为阻塞接口 FALSE 为非阻塞接口
	* @param speed 机器人直线运动速度，单位：mm/s
	* @param acc 机器人直线运动加速度,单位mm/s^2
	* @param tol 机器人直线运动终点误差, 单位mm
	* @param option_cond 机器人关节可选参数，如果不需要，该值可不赋值,填入空指针就可
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t linear_move_extend(const JKHD *handle, const CartesianPose *end_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond);

	/**
	* @brief 机器人末端圆弧运动
	* @param end_pos 机器人末端运动目标位置
	* @param mid_pos 机器人末端运中间点
	* @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
	* @param is_block 设置接口是否为阻塞接口，TRUE 为阻塞接口 FALSE 为非阻塞接口
	* @param speed 机器人圆弧速度，单位：rad/s
	* @param acc 机器人圆弧运动加速度，单位：rad/s^2
	* @param tol 机器人关节运动终点误差, 单位mm
	* @param option_cond 机器人关节可选参数，如果不需要，该值可不赋值,填入空指针就可
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t circular_move(const JKHD *handle, const CartesianPose *end_pos, const CartesianPose *mid_pos, MoveMode move_mode, BOOL is_block, double speed, double accel, double tol, const OptionalCond *option_cond);

	/**
	* @brief 机器人SERVO MOVE模式使能
	* @param handle  机器人控制句柄
	* @param enable  TRUE为进入SERVO MOVE模式，FALSE表示退出该模式
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_move_enable(const JKHD *handle, BOOL enable);

	/**
	* @brief 机器人关节空间位置控制模式
	* @param handle  机器人控制句柄
	* @param joint_pos 机器人关节运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @return ERR_SUCC成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_j(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode);

	/**
	* @brief 机器人关节空间位置控制模式
	* @param handle  机器人控制句柄
	* @param joint_pos 机器人关节运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @step_num  倍分周期，servo_j运动周期为step_num*8ms，其中step_num>=1
	* @return ERR_SUCC成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_j_extend(const JKHD *handle, const JointValue *joint_pos, MoveMode move_mode, unsigned int step_num);

	/**
	* @brief 机器人笛卡尔空间位置控制模式
	* @param handle  机器人控制句柄
	* @param cartesian_pose 机器人笛卡尔空间运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_p(const JKHD *handle, const CartesianPose *cartesian_pose, MoveMode move_mode);

	/**
	* @brief 机器人笛卡尔空间位置控制模式
	* @param handle  机器人控制句柄
	* @param cartesian_pose 机器人笛卡尔空间运动目标位置
	* @move_mode 指定运动模式：增量运动或绝对运动
	* @step_num  倍分周期，servo_p运动周期为step_num*8ms，其中step_num>=1
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_p_extend(const JKHD *handle, const CartesianPose *cartesian_pose, MoveMode move_mode, unsigned int step_num);

	/**
	* @brief 设置数字输出变量(DO)的值
	* @param handle 机器人控制句柄
	* @param type DO类型
	* @param index DO索引
	* @param value DO设置值
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_digital_output(const JKHD *handle, IOType type, int index, BOOL value);

	/**
	* @brief 设置模拟输出变量的值(AO)的值
	* @param handle 机器人控制句柄
	* @param type AO类型
	* @param index AO索引
	* @param value AO设置值
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_analog_output(const JKHD *handle, IOType type, int index, float value);

	/**
	* @brief 查询数字输入(DI)状态
	* @param handle 机器人控制句柄
	* @param type DI类型
	* @param index DI索引
	* @param result DI状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_digital_input(const JKHD *handle, IOType type, int index, BOOL *result);

	/**
	* @brief 查询数字输出(DO)状态
	* @param handle 机器人控制句柄
	* @param type DO类型
	* @param index DO索引
	* @param result DO状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_digital_output(const JKHD *handle, IOType type, int index, BOOL *result);

	/**
	* @brief 获取模拟量输入变量(AI)的值
	* @param handle 机器人控制句柄
	* @param type AI的类型
	* @param index AI索引
	* @param result 指定AI状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_analog_input(const JKHD *handle, IOType type, int index, float *result);

	/**
	* @brief 获取模拟量输出变量(AO)的值
	* @param handle 机器人控制句柄
	* @param type AO的类型
	* @param index AO索引
	* @param result 指定AO状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_analog_output(const JKHD *handle, IOType type, int index, float *result);

	/**
	* @brief 查询扩展IO模块是否运行
	* @param handle 机器人控制句柄
	* @param is_running 扩展IO模块运行状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t is_extio_running(const JKHD *handle, BOOL *is_running);

	/**
	* @brief 运行当前加载的作业程序
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t program_run(const JKHD *handle);

	/**
	* @brief 暂停当前运行的作业程序
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t program_pause(const JKHD *handle);

	/**
	* @brief 继续运行当前暂停的作业程序
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t program_resume(const JKHD *handle);

	/**
	* @brief 终止当前执行的作业程序
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t program_abort(const JKHD *handle);

	/**
	* @brief 加载指定的作业程序
	* @param handle 机器人控制句柄
	* @param file 程序文件路径
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t program_load(const JKHD *handle, const char *file);

	/**
	* @brief 获取已加载的作业程序路径
	* @param handle 机器人控制句柄
	* @param file 程序文件路径
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_loaded_program(const JKHD *handle, char *file);

	/**
	* @brief 获取当前机器人作业程序的执行行号
	* @param handle  机器人控制句柄
	* @param curr_line 当前行号查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_current_line(const JKHD *handle, int *curr_line);

	/**
	* @brief 获取机器人作业程序执行状态
	* @param handle  机器人控制句柄
	* @param status 作业程序执行状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_program_state(const JKHD *handle, ProgramState *status);

	/**
	* @brief 设置机器人运行倍率
	* @param handle  机器人控制句柄
	* @param rapid_rate 是程序运行倍率，设置范围为[0,1]
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_rapidrate(const JKHD *handle, double rapid_rate);

	/**
	* @brief 获取机器人运行倍率
	* @param handle 机器人控制句柄
	* @param rate 当前控制系统倍率
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_rapidrate(const JKHD *handle, double *rapid_rate);

	/**
	* @brief 设置指定编号的工具信息
	* @param handle 机器人控制句柄
	* @param id 工具编号
	* @param tcp 工具坐标系相对法兰坐标系偏置
	* @param name 指定工具的别名
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_tool_data(const JKHD *handle, int id, const CartesianPose *tcp, const char *name);

	/**
	* @brief 设置当前使用的工具ID
	* @param handle  机器人控制句柄
	* @param id 工具坐标系ID
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_tool_id(const JKHD *handle, const int id);

	/**
	* @brief 查询当前使用的工具ID
	* @param handle  机器人控制句柄
	* @param id 工具ID查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_tool_id(const JKHD *handle, int *id);

	/**
	* @brief 查询使用的工具信息
	* @param id 工具ID查询结果
	* @param tcp 工具坐标系相对法兰坐标系偏置
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_tool_data(const JKHD *handle, int id, CartesianPose *tcp);

	/**
	* @brief 设置指定编号的用户坐标系信息
	* @param handle 机器人控制句柄
	* @param id 用户坐标系编号
	* @param user_frame 用户坐标系偏置值
	* @param name 用户坐标系别名
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_user_frame_data(const JKHD *handle, int id, const CartesianPose *user_frame, const char *name);

	/**
	* @brief 设置当前使用的用户坐标系ID
	* @param handle  机器人控制句柄
	* @param id 用户坐标系ID
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_user_frame_id(const JKHD *handle, const int id);

	/**
	* @brief 查询当前使用的用户坐标系ID
	* @param handle  机器人控制句柄
	* @param id 获取的结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_user_frame_id(const JKHD *handle, int *id);

	/**
	* @brief 查询使用的用户坐标系信息
	* @param id 用户坐标系ID查询结果
	* @param tcp 用户坐标系偏置值
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_user_frame_data(const JKHD *handle, int id, CartesianPose *tcp);

	/**
	* @brief 控制器机器人进入或退出拖拽模式
	* @param handle  机器人控制句柄
	* @param enable  TRUE为进入拖拽模式，FALSE为退出拖拽模式
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t drag_mode_enable(const JKHD *handle, BOOL enable);

	/**
	* @brief 查询机器人是否处于拖拽模式
	* @param handle  机器人控制句柄
	* @param in_drag 查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t is_in_drag_mode(const JKHD *handle, BOOL *in_drag);

	/**
	* @brief 获取机器人状态
	* @param handle  机器人控制句柄
	* @param state 机器人状态查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_robot_state(const JKHD *handle, RobotState *state);

	/**
	* @brief 获取当前设置下工具末端的位姿
	* @param handle  机器人控制句柄
	* @param tcp_position 工具末端位置查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_tcp_position(const JKHD *handle, CartesianPose *tcp_position);

	/**
	* @brief 获取当前设置下工具末端的位姿
	* @param handle  机器人控制句柄
	* @param tcp_position 工具末端位置查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_joint_position(const JKHD *handle, JointValue *joint_position);

	/**
	* @brief 查询机器人是否处于碰撞保护模式
	* @param handle  机器人控制句柄
	* @param in_collision 查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t is_in_collision(const JKHD *handle, BOOL *in_collision);

	/**
	* @brief 查询机器人是否超出限位
	* @param handle  机器人控制句柄
	* @param on_limit 查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t is_on_limit(const JKHD *handle, BOOL *on_limit);

	/**
	* @brief 查询机器人运动是否停止
	* @param handle  机器人控制句柄
	* @param in_pos 查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t is_in_pos(const JKHD *handle, BOOL *in_pos);

	/**
	* @brief 碰撞之后从碰撞保护模式恢复
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t collision_recover(const JKHD *handle);

	/**
	* @brief  错误状态清除
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t clear_error(const JKHD *handle);

	/**
	* @brief 设置机器人碰撞等级
	* @param handle  机器人控制句柄
	* @param level  碰撞等级，等级0-5 ，0为关闭碰撞，1为碰撞阈值25N，2为碰撞阈值50N，3为碰撞阈值75N，4为碰撞阈值100N，5为碰撞阈值125N
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_collision_level(const JKHD *handle, const int level);

	/**
	* @brief 碰撞之后从碰撞保护模式恢复
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_collision_level(const JKHD *handle, int *level);

	/**
	* @brief 计算指定位姿在当前工具、当前安装角度以及当前用户坐标系设置下的逆解
	* @param handle  机器人控制句柄
	* @param ref_pos 逆解计算用的参考关节空间位置
	* @param cartesian_pose 笛卡尔空间位姿值
	* @param joint_pos 计算成功时关节空间位置计算结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t kine_inverse(const JKHD *handle, const JointValue *ref_pos, const CartesianPose *cartesian_pose, JointValue *joint_pos);

	/**
	* @brief 计算指定关节位置在当前工具、当前安装角度以及当前用户坐标系设置下的位姿值
	* @param handle  机器人控制句柄
	* @param joint_pos 关节空间位置
	* @param cartesian_pose 笛卡尔空间位姿计算结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t kine_forward(const JKHD *handle, const JointValue *joint_pos, CartesianPose *cartesian_pose);

	/**
	* @brief 欧拉角到旋转矩阵的转换
	* @param handle  机器人控制句柄
	* @param rpy 待转换的欧拉角数据
	* @param rot_matrix 转换后的旋转矩阵
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t rpy_to_rot_matrix(const JKHD *handle, const Rpy *rpy, RotMatrix *rot_matrix);

	/**
	* @brief 旋转矩阵到欧拉角的转换
	* @param handle  机器人控制句柄
	* @param rot_matrix 待转换的旋转矩阵数据
	* @param rpy 转换后的RPY欧拉角结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t rot_matrix_to_rpy(const JKHD *handle, const RotMatrix *rot_matrix, Rpy *rpy);

	/**
	* @brief 四元数到旋转矩阵的转换
	* @param handle  机器人控制句柄
	* @param quaternion 待转换的四元数数据
	* @param rot_matrix 转换后的旋转矩阵结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t quaternion_to_rot_matrix(const JKHD *handle, const Quaternion *quaternion, RotMatrix *rot_matrix);

	/**
	* @brief 旋转矩阵到四元数的转换
	* @param handle  机器人控制句柄
	* @param rot_matrix 待转换的旋转矩阵
	* @param quaternion 转换后的四元数结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t rot_matrix_to_quaternion(const JKHD *handle, const RotMatrix *rot_matrix, Quaternion *quaternion);

	/**
	* @brief 注册机器人出现错误时的回调函数
	* @param handle  机器人控制句柄
	* @param func 指向用户定义的函数的函数指针
	* @param error_code 机器人的错误码
	*/
	DLLEXPORT_API errno_t set_error_handler(const JKHD *handle, CallBackFuncType func);

	/**
	* @brief 机器人负载设置
	* @param handle  机器人控制句柄
	* @param payload 负载质心、质量数据
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_payload(const JKHD *handle, const PayLoad *payload);

	/**
	* @brief 获取机器人负载数据
	* @param handle  机器人控制句柄
	* @param payload 负载查询结果
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_payload(const JKHD *handle, PayLoad *payload);

	/**
	* @brief 获取SDK版本号
	* @param handle  机器人控制句柄
	* @param version SDK版本号
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_sdk_version(const JKHD *handle, char *version);

	/**
	* @brief 获取控制器IP
	* @param controller_name 控制器名字
	* @param ip_list 控制器ip列表，控制器名字为具体值时返回该名字所对应的控制器IP地址，控制器名字为空时，返回网段类内的所有控制器IP地址
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_controller_ip(char *controller_name, char *ip_list);

	/**
	* @brief 获取机器人状态数据
	* @param handle  机器人控制句柄
	* @param status 机器人状态
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_robot_status(const JKHD *handle, RobotStatus *status);

	/**
	* @brief 终止当前机械臂运动
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t motion_abort(const JKHD *handle);

	/**
	* @brief 设置错误码文件路径，需要使用get_last_error接口时需要设置错误码文件路径，如果不使用get_last_error接口，则不需要设置该接口（使用英文路径）
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_errorcode_file_path(const JKHD *handle, char *path);

	/**
	* @brief 获取机器人运行过程中最后一个错误码,当调用clear_error时，最后一个错误码会清零
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_last_error(const JKHD *handle, ErrorCode *code);

	/**
	* @brief 设置是否开启调试模式，选择TRUE时，开始调试模式，此时会在标准输出流中输出调试信息，选择FALSE时，不输出调试信息
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_debug_mode(const JKHD *handle, BOOL mode);

	/**
	* @brief 设置轨迹复现配置参数
	* @param handle  机器人控制句柄
	* @param para 轨迹复现配置参数
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_traj_config(const JKHD *handle, const TrajTrackPara *para);

	/**
	* @brief 获取轨迹复现配置参数
	* @param handle  机器人控制句柄
	* @param para 轨迹复现配置参数
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_traj_config(const JKHD *handle, TrajTrackPara *para);

	/**
	* @brief 采集轨迹复现数据控制开关
	* @param handle  机器人控制句柄
	* @param mode 选择TRUE时，开始数据采集，选择FALSE时，结束数据采集
	* @param filename 采集数据的存储文件名，当filename为空指针时，存储文件以当前日期命名
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_traj_sample_mode(const JKHD *handle, const BOOL mode, char *filename);

	/**
	* @brief 采集轨迹复现数据状态查询
	* @param handle  机器人控制句柄
	* @param mode 为TRUE时，数据正在采集，为FALSE时，数据采集结束，在数据采集状态时不允许再次开启数据采集开关
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_traj_sample_status(const JKHD *handle, BOOL *sample_status);

	/**
	* @brief 查询控制器中已经存在的轨迹复现数据的文件名
	* @param handle  机器人控制句柄
	* @param filename 控制器中已经存在的轨迹复现数据的文件名
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_exist_traj_file_name(const JKHD *handle, MultStrStorType *filename);

	/**
	* @brief 重命名轨迹复现数据的文件名
	* @param handle  机器人控制句柄
	* @param src 原文件名
	* @param dest 目标文件名，文件名长度不能超过100个字符，文件名不能为空，目标文件名不支持中文
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t rename_traj_file_name(const JKHD *handle, const char *src, const char *dest);

	/**
	* @brief 删除控制器中轨迹复现数据文件
	* @param handle  机器人控制句柄
	* @param filename 要删除的文件的文件名，文件名为数据文件名字
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t remove_traj_file(const JKHD *handle, const char *filename);

	/**
	* @brief 控制器中轨迹复现数据文件生成控制器执行脚本
	* @param handle  机器人控制句柄
	* @param filename 数据文件的文件名，文件名为数据文件名字，不带后缀
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t generate_traj_exe_file(const JKHD *handle, const char *filename);

	/**
	* @brief SERVO模式下不使用滤波器,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_move_use_none_filter(const JKHD *handle);

	/**
	* @brief SERVO模式下关节空间一阶低通滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param handle  机器人控制句柄
	* @param cutoffFreq 一阶低通滤波器截止频率
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_move_use_joint_LPF(const JKHD *handle, double cutoffFreq);

	/**
	* @brief SERVO模式下关节空间非线性滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param handle  机器人控制句柄
	* @param max_vr 笛卡尔空间姿态变化速度的速度上限值（绝对值）°/s
	* @param max_ar 笛卡尔空间姿态变化速度的加速度上限值（绝对值）°/s^2
	* @param max_jr 笛卡尔空间姿态变化速度的加加速度上限值（绝对值）°/s^3
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_move_use_joint_NLF(const JKHD *handle, double max_vr, double max_ar, double max_jr);

	/**
	* @brief SERVO模式下笛卡尔空间非线性滤波,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param handle  机器人控制句柄
	* @param max_vp 笛卡尔空间下移动指令速度的上限值（绝对值）。单位：mm/s
	* @param max_ap 笛卡尔空间下移动指令加速度的上限值（绝对值）。单位：mm/s^2
	* @param max_jp 笛卡尔空间下移动指令加加速度的上限值（绝对值）单位：mm/s^3
	* @param max_vr 笛卡尔空间姿态变化速度的速度上限值（绝对值）°/s
	* @param max_ar 笛卡尔空间姿态变化速度的加速度上限值（绝对值）°/s^2
	* @param max_jr 笛卡尔空间姿态变化速度的加加速度上限值（绝对值）°/s^3
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_move_use_carte_NLF(const JKHD *handle, double max_vp, double max_ap, double max_jp, double max_vr, double max_ar, double max_jr);

	/**
	* @brief SERVO模式下关节空间多阶均值滤波器,该指令在SERVO模式下不可设置，退出SERVO后可设置
	* @param handle  机器人控制句柄
	* @param max_buf 均值滤波器缓冲区的大小
	* @param kp 加速度滤波系数
	* @param kv 速度滤波系数
	* @param ka 位置滤波系数
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_move_use_joint_MMF(const JKHD *handle, int max_buf, double kp, double kv, double ka);

	/**
	* @brief SERVO模式下速度前瞻参数设置
	* @param handle  机器人控制句柄
	* @param max_buf 缓冲区的大小
	* @param kp 加速度滤波系数
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t servo_speed_foresight(const JKHD *handle, int max_buf, double kp);

	/**
	* @brief 设置SDK日志路径
	* @param handle  机器人控制句柄
	* @param filepath SDK日志路径
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_SDK_filepath(const JKHD *handle, char *filepath);

	/**
	* @brief 设置传感器品牌
	* @param handle  机器人控制句柄
	* @param sensor_brand 传感器品牌，可选值为1,2,3 分别代表不同品牌力矩传感器
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_torsenosr_brand(const JKHD *handle, int sensor_brand);

	/**
	* @brief 获取传感器品牌
	* @param handle  机器人控制句柄
	* @param sensor_brand 传感器品牌，可选值为1,2,3 分别代表不同品牌力矩传感器
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_torsenosr_brand(const JKHD *handle, int *sensor_brand);

	/**
	* @brief 开启或关闭力矩传感器
	* @param handle  机器人控制句柄
	* @param sensor_mode 0代表关闭传感器，1代表开启力矩传感器
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_torque_sensor_mode(const JKHD *handle, int sensor_mode);

	/**
	* @brief 设置柔顺控制参数
	* @param handle  机器人控制句柄
	* @param axis 代表配置哪一轴，可选值为0~5
	* @param opt 柔顺方向，可选值为 1 2 3 4 5 6分别对应 fx fy fz mx my mz 0代表没有勾选
	* @param ftUser 阻尼力，表示用户用多大的力才能让机器人的沿着某个方向以最大速度进行运动
	* @param ftReboundFK 回弹力，表示机器人回到初始状态的能力
	* @param ftConstant 代表恒力，手动操作时全部设置为0
	* @param ftNnormalTrack 法向跟踪，手动操作时全部设置为0,
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_admit_ctrl_config(const JKHD *handle, const int axis, const int opt, const double ftUser, const double ftConstant, const int ftNnormalTrack, const double ftReboundFK);

	/**
	* @brief 开始辨识工具末端负载
	* @param handle  机器人控制句柄
	* @param joint_pos 使用力矩传感器进行自动负载辨识时的结束位置
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t start_torq_sensor_payload_identify(const JKHD *handle, const JointValue *joint_pos);

	/**
	* @brief 获取末端负载辨识状态
	* @param handle  机器人控制句柄
	* @param identify_status 0代表辨识完成，1代表未完成，2代表辨识失败
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_torq_sensor_identify_staus(const JKHD *handle, int *identify_status);

	/**
	* @brief 获取末端负载辨识结果
	* @param handle  机器人控制句柄
	* @param payload 末端负载
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_torq_sensor_payload_identify_result(const JKHD *handle, PayLoad *payload);

	/**
	* @brief 设置传感器末端负载
	* @param handle  机器人控制句柄
	* @param payload 末端负载
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_torq_sensor_tool_payload(const JKHD *handle, const PayLoad *payload);

	/**
	* @brief 获取传感器末端负载
	* @param handle  机器人控制句柄
	* @param payload 末端负载
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_torq_sensor_tool_payload(const JKHD *handle, PayLoad *payload);

	/**
	* @brief 力控拖拽使能
	* @param handle  机器人控制句柄
	* @param enable_flag 0为关闭力控拖拽使能，1为开启
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t enable_admittance_ctrl(const JKHD *handle, const int enable_flag);

	/**
	* @brief 设置力控类型和传感器初始化状态
	* @param handle  机器人控制句柄
	* @param sensor_compensation 是否开启传感器补偿,1代表开启即初始化,0代表不初始化
	* @param compliance_type 0 代表不使用任何一种柔顺控制方法 1 代表恒力柔顺控制,2 代表速度柔顺控制
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_compliant_type(const JKHD *handle, int sensor_compensation, int compliance_type);

	/**
	* @brief 获取力控类型和传感器初始化状态
	* @param handle  机器人控制句柄
	* @param sensor_compensation 是否开启传感器补偿,1代表开启即初始化,0代表不初始化
	* @param compliance_type 0 代表不使用任何一种柔顺控制方法 1 代表恒力柔顺控制,2 代表速度柔顺控制
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_compliant_type(const JKHD *handle, int *sensor_compensation, int *compliance_type);

	/**
	* @brief 获取力控柔顺控制参数
	* @param handle  机器人控制句柄
	* @param admit_ctrl_cfg 机器人力控柔顺控制参数存储地址
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_admit_ctrl_config(const JKHD *handle, RobotAdmitCtrl *admit_ctrl_cfg);

	/**
	* @brief 设置力控传感器通信参数
	* @param handle  机器人控制句柄
	* @param type 0为使用tcp/ip协议，1为使用RS485协议
	* @param ip_addr为力控传感器地址
	* @param port为使用tcp/ip协议时力控传感器端口号
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_torque_sensor_comm(const JKHD *handle, const int type, const char *ip_addr, const int port);

	/**
	* @brief 获取力控传感器通信参数
	* @param handle  机器人控制句柄
	* @param ip_addr为力控传感器地址
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_torque_sensor_comm(const JKHD *handle, int *type, char *ip_addr, int *port);

	/**
	* @brief 设置力控的低通滤波器的值
	* @param torque_sensor_filter 低通滤波器的值,单位：Hz
	*/
	DLLEXPORT_API errno_t set_torque_sensor_filter(const JKHD *handle, const float torque_sensor_filter);

	/**
	* @brief 获取力控的低通滤波器的值
	* @param torque_sensor_filter 低通滤波器的值,单位：Hz
	*/
	DLLEXPORT_API errno_t get_torque_sensor_filter(const JKHD *handle, float *torque_sensor_filter);

	/**
	* @brief 设置力传感器的传感器限位参数配置
	* @param torque_sensor_soft_limit 力传感器的传感器限位参数
	*        力限位 fx, fy, fz 单位：N
	*        力矩限位 tx, ty, tz 单位：N*m
	*/
	DLLEXPORT_API errno_t set_torque_sensor_soft_limit(const JKHD *handle, const FTxyz torque_sensor_soft_limit);

	/**
	* @brief 获取力传感器的传感器限位参数配置
	* @param torque_sensor_soft_limit 力传感器的传感器限位参数
	*        力限位 fx, fy, fz 单位：N
	*        力矩限位 tx, ty, tz 单位：N*m
	*/
	DLLEXPORT_API errno_t get_torque_sensor_soft_limit(const JKHD *handle, FTxyz *torque_sensor_soft_limit);

	/**
	* @brief 关闭力控
	* @param handle  机器人控制句柄
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t disable_force_control(const JKHD *handle);

	/**
	* @brief 设置速度柔顺控制参数
	* @param handle  机器人控制句柄
	* @param vel_cfg为速度柔顺控制参数
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_vel_compliant_ctrl(const JKHD *handle, const VelCom *vel);

	/**
	* @brief 设置柔顺控制力矩条件
	* @param handle  机器人控制句柄
	* @param ft为柔顺控制力矩条件
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_compliance_condition(const JKHD *handle, const FTxyz *ft);

	/**
	* @brief 设置网络异常，SDK与机器人控制器失去连接后多长时间机器人控制器终止机械臂当前运动
	* @param handle  机器人控制句柄
	* @param millisecond 时间参数，单位毫秒
	* @param mnt 网络异常时机器人需要进行的动作类型
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_network_exception_handle(const JKHD *handle, float millisecond, ProcessType mnt);

	/**
	* @brief 设置机器人状态数据自动更新时间间隔
	* @param handle  机器人控制句柄
	* @param millisecond 时间参数，单位毫秒
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_status_data_update_time_interval(const JKHD *handle, float millisecond);

	/**
	* @brief 设置机器人阻塞等待超时时间
	* @param seconds 时间参数，单位秒
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_block_wait_timeout(const JKHD* handle, float seconds);

	/**
	* @brief 设置导纳控制运动坐标系
	* @param ftFrame 0工具 1世界
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_ft_ctrl_frame(const JKHD *handle, const int ftFrame);

	/**
	* @brief 获取导纳控制运动坐标系
	* @param ftFrame 0工具 1世界
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_ft_ctrl_frame(const JKHD *handle, int *ftFrame);

	/**
	* @brief 获取机器人DH参数
	* @param dhParam DH参数
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_dh_param(const JKHD *handle, DHParam *dhParam);

	/**
	* @brief 设置tioV3电压参数
	* @param vout_enable 电压使能，0:关，1开
	* @param vout_vol 电压大小 0:24v 1:12v
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t set_tio_vout_param(const JKHD* handle, int vout_enable, int vout_vol);

	/**
	* @brief 获取tioV3电压参数
	* @param vout_enable 电压使能，0:关，1开
	* @param vout_vol 电压大小 0:24v 1:12v
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_tio_vout_param(const JKHD* handle, int* vout_enable, int* vout_vol);
	/**
	* @brief 与控制器建立ftp链接
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t init_ftp_client(const JKHD *handle);

	/**
	* @brief 与控制器建立加密ftp链接(需要app登录且控制器版本支持)
	* @param password 机器人登陆密码
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t init_ftp_client_with_ssl(const JKHD* handle, char* password);


	/**
	* @brief 断开与控制器ftp链接
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t close_ftp_client(const JKHD *handle);
	/**
	* @brief 从控制器下载指定类型和名称的文件到本地
	* @param remote 控制器内部文件名绝对路径
	* @param local 下载到本地文件名绝对路径
	* @param opt 1单个文件 2文件夹
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t download_file(const JKHD *handle, char *local, char *remote, int opt);

	/**
	* @brief 从控制器上传指定类型和名称的文件到本地
	* @param remote 控制器内部文件名绝对路径
	* @param local 下载到本地文件名绝对路径
	* @param opt 1单个文件 2文件夹
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t upload_file(const JKHD *handle, char *local, char *remote, int opt);

	/**
	* @brief 从控制器删除指定类型和名称的文件
	* @param remote 控制器内部文件名
	* @param opt 1单个文件 2文件夹
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t del_ftp_file(const JKHD *handle, char *remote, int opt);

	/**
	* @brief 重命名控制器指定类型和名称的文件
	* @param remote 控制器内部文件名原名称
	* @param des 重命名的目标名
	* @param opt 1单个文件 2文件夹
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t rename_ftp_file(const JKHD *handle, char *remote, char *des, int opt);

	/**
	* @brief 获取ftp文件列表
	* @param remotedir 查询列表 
	* @param type 1单个文件 2文件夹
	* @param ret 返回字符串
	* @return ERR_SUCC 成功 其他失败
	*/
	DLLEXPORT_API errno_t get_ftp_dir(const JKHD *handle, const char *remotedir, int type, char *ret);
//#ifdef __cpluscplus
}
//#endif

#undef DLLEXPORT_API
#endif

