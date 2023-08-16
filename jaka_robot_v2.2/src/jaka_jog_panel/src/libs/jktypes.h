	/**
	* @last update Nov 30 2021 
	* @sdkversion 2.1.2.3_dev
	* @Maintenance star@jaka
	*/
#ifndef _JHTYPES_H_
#define _JHTYPES_H_

#define TRUE 1
#define FALSE 0
#include <stdio.h>
#include <stdint.h>

typedef int BOOL;	 //布尔类型
typedef int JKHD;	 //机械臂控制句柄类型
typedef int errno_t; //接口返回值类型

/**
* @brief 笛卡尔空间位置数据类型
*/
typedef struct
{
	double x; ///< x轴坐标，单位mm
	double y; ///< y轴坐标，单位mm
	double z; ///< z轴坐标，单位mm
} CartesianTran;

/**
* @brief 欧拉角姿态数据类型
*/
typedef struct
{
	double rx; ///< 绕固定轴X旋转角度，单位：rad
	double ry; ///< 绕固定轴Y旋转角度，单位：rad
	double rz; ///< 绕固定轴Z旋转角度，单位：rad
} Rpy;

/**
* @brief 四元数姿态数据类型
*/
typedef struct
{
	double s;
	double x;
	double y;
	double z;
} Quaternion;

/**
 *@brief 笛卡尔空间位姿类型
 */
typedef struct
{
	CartesianTran tran; ///< 笛卡尔空间位置
	Rpy rpy;			///< 笛卡尔空间姿态
} CartesianPose;

/**
* @brief 旋转矩阵数据类型
*/
typedef struct
{
	CartesianTran x; ///< x轴列分量
	CartesianTran y; ///< y轴列分量
	CartesianTran z; ///< z轴列分量
} RotMatrix;

/**
* @brief 程序运行状态枚举类型
*/
typedef enum
{
	PROGRAM_IDLE,	 ///< 机器人停止运行
	PROGRAM_RUNNING, ///< 机器人正在运行
	PROGRAM_PAUSED	 ///< 机器人暂停
} ProgramState;

/**
* @brief 坐标系选择枚举类型
*/
typedef enum
{
	COORD_BASE,	 ///< 基坐标系
	COORD_JOINT, ///< 关节空间
	COORD_TOOL	 ///< 工具坐标系
} CoordType;

/**
* @brief jog运动模式枚举 
*/
typedef enum
{
	ABS = 0, ///< 绝对运动
	INCR,	 ///< 增量运动
	CONTINUE ///< 连续运动
} MoveMode;

/**
* @brief 系统监测数据类型
*/
typedef struct
{
	int scbMajorVersion;		///<scb主版本号
	int scbMinorVersion;		///<scb次版本号
	int cabTemperature;			///<控制柜温度
	double robotAveragePower;	///<控制柜总线平均功率
	double robotAverageCurrent; ///<控制柜总线平均电流
	double instCurrent[6];		///<机器人6个轴的瞬时电流
	double instVoltage[6];		///<机器人6个轴的瞬时电压
	double instTemperature[6];	///<机器人6个轴的瞬时温度
} SystemMonitorData;

/**
* @brief 负载数据类型
*/
typedef struct
{
	double mass;			///<负载质量，单位：kg
	CartesianTran centroid; ///<负载质心, 单位：mm
} PayLoad;

/**
* @brief 关节位置数据类型
*/
typedef struct
{
	double jVal[6]; ///< 6关节位置值，单位：rad
} JointValue;

/**
* @brief IO类型枚举
*/
typedef enum
{
	IO_CABINET, ///< 控制柜面板IO
	IO_TOOL,	///< 工具IO
	IO_EXTEND,	///< 扩展IO
	IO_REALY,   ///< 继电器IO，目前仅CAB V3支持DO
	IO_MODBUS_SLAVE, ///< Modbus从站IO,从0索引
	IO_PROFINET_SLAVE, ///< Profinet从站IO,从0索引
	IO_EIP_SLAVE      ///< ETHRENET/IP从站IO,从0索引
} IOType;

/**
* @brief 机器人状态数据
*/
typedef struct
{
	BOOL estoped;	   ///< 是否急停
	BOOL poweredOn;	   ///< 是否打开电源
	BOOL servoEnabled; ///< 是否使能
} RobotState;

/**
* @brief 机器人回调函数指针
*/
typedef void (*CallBackFuncType)(int);

/**
* @brief 机器人力矩前馈数据
*/

/**
* @brief 机器人关节监测数据
*/
typedef struct
{
	double instCurrent;		///< 瞬时电流
	double instVoltage;		///< 瞬时电压
	double instTemperature; ///< 瞬时温度
	double instVel;			///< 瞬时速度 控制器1.7.0.20及以上
	double instTorq;		///< 瞬时力矩
} JointMonitorData;

/**
* @brief EXtio数据
*/
typedef struct
{
	int din[256];				  ///< 数字输入din[0]为有效信号的个数
	int dout[256];				  ///< 数字输出dout[0]为有效信号的个数
	float ain[256];				  ///< 模拟输入ain[0]为有效信号的个数
	float aout[256];			      ///< 模拟输出aout[0]为有效信号的个数
} Io_group;

/**
* @brief 机器人监测数据
*/
typedef struct
{
	double scbMajorVersion;				  ///< scb主版本号
	double scbMinorVersion;				  ///< scb小版本号
	double cabTemperature;				  ///< 控制器温度
	double robotAveragePower;			  ///< 机器人平均电压
	double robotAverageCurrent;			  ///< 机器人平均电流
	JointMonitorData jointMonitorData[6]; ///< 机器人6个关节的监测数据
} RobotMonitorData;

/**
* @brief 力矩传感器监测数据
*/
typedef struct
{
	char ip[20];		 ///< 力矩传感器ip地址
	int port;			 ///< 力矩传感器端口号
	PayLoad payLoad;	 ///< 工具负载
	int status;			 ///< 力矩传感器状态
	int errcode;		 ///< 力矩传感器异常错误码
	double actTorque[6]; ///< 力矩传感器实际接触力值（勾选初始化时）或原始读数值（勾选不初始化时）
	double torque[6];	 ///< 力矩传感器原始读数值
	double realTorque[6];///< 力矩传感器实际接触力值（不随初始化选项变化）
} TorqSensorMonitorData;

/**
* @brief 机器人状态监测数据,使用get_robot_status函数更新机器人状态数据
*/
typedef struct
{
	int errcode;									///< 机器人运行出错时错误编号，0为运行正常，其它为运行异常
	int inpos;										///< 机器人运动是否到位标志，0为没有到位，1为运动到位
	int powered_on;									///< 机器人是否上电标志，0为没有上电，1为上电
	int enabled;									///< 机器人是否使能标志，0为没有使能，1为使能
	double rapidrate;								///< 机器人运动倍率
	int protective_stop;							///< 机器人是否检测到碰撞，0为没有检测到碰撞，1为检测到碰撞
	int emergency_stop;								///< 机器人是否急停，0为没有急停，1为急停
	int dout[256];									///< 机器人控制柜数字输出信号,dout[0]为信号的个数
	int din[256];									///< 机器人控制柜数字输入信号,din[0]为信号的个数	
	double ain[256];								///< 机器人控制柜模拟输入信号,ain[0]为信号的个数
	double aout[256];								///< 机器人控制柜模拟输出信号,aout[0]为信号的个数
	int tio_dout[16];								///< 机器人末端工具数字输出信号,tio_dout[0]为信号的个数
	int tio_din[16];								///< 机器人末端工具数字输入信号,tio_din[0]为信号的个数
	double tio_ain[16];								///< 机器人末端工具模拟输入信号,tio_ain[0]为信号的个数
	int tio_key[3];                                 ///< 机器人末端工具按钮 [0]free;[1]point;[2]pause_resume;
	Io_group extio;								    ///< 机器人外部应用IO
	Io_group modbus_slave;							///< 机器人Modbus从站
	Io_group profinet_slave;						///< 机器人Profinet从站
	Io_group eip_slave;								///< 机器人Ethernet/IP从站
	unsigned int current_tool_id;					///< 机器人目前使用的工具坐标系id
	double cartesiantran_position[6];				///< 机器人末端所在的笛卡尔空间位置
	double joint_position[6];						///< 机器人关节空间位置
	unsigned int on_soft_limit;						///< 机器人是否处于限位，0为没有触发限位保护，1为触发限位保护
	unsigned int current_user_id;					///< 机器人目前使用的用户坐标系id
	int drag_status;								///< 机器人是否处于拖拽状态，0为没有处于拖拽状态，1为处于拖拽状态
	RobotMonitorData robot_monitor_data;			///< 机器人状态监测数据
	TorqSensorMonitorData torq_sensor_monitor_data; ///< 机器人力矩传感器状态监测数据
	int is_socket_connect;							///< sdk与控制器连接通道是否正常，0为连接通道异常，1为连接通道正常
} RobotStatus;

/**
* @brief 机器人错误码数据类型
*/
typedef struct
{
	long code;		   ///< 错误码编号
	char message[120]; ///< 错误码对应提示信息
} ErrorCode;

/**
* @brief 轨迹复现配置参数存储数据类型
*/
typedef struct
{
	double xyz_interval; ///< 空间位置采集精度
	double rpy_interval; ///< 姿态采集精度
	double vel;			 ///< 执行脚本运行速度
	double acc;			 ///< 执行脚本运行加速度
} TrajTrackPara;

/**
* @brief 多个字符串存储数据类型
*/
typedef struct
{
	int len;			 ///< 字符串个数
	char name[128][128]; ///< 数据存储二维数组
} MultStrStorType;

/**
* @brief 运动参数可选项
*/
typedef struct
{
	int executingLineId; ///< 控制命令id编号
} OptionalCond;

/**
* @brief 网络异常机器人运动自动终止类型枚举
*/
typedef enum
{
	MOT_KEEP,  ///< 网络异常时机器人继续保持原来的运动
	MOT_PAUSE, ///< 网络异常时机器人暂停运动
	MOT_ABORT  ///< 网络异常时机器人终止运动
} ProcessType;

/**
* @brief 柔顺控制参数类型
*/
typedef struct
{
	int opt;			 ///< 柔顺方向，可选值为 1 2 3 4 5 6分别对应 fx fy fz mx my mz,0代表没有勾选
	double ft_user;		 ///< 用户用多大的力才能让机器人的沿着某个方向以最大速度进行运动
	double ft_rebound;	 ///< 回弹力:机器人回到初始状态的能力
	double ft_constant;	 ///< 恒力
	int ft_normal_track; ///< 法向跟踪是否开启，0为没有开启，1为开启
} AdmitCtrlType;

/**
* @brief 机器人柔顺控制参数类型
*/
typedef struct
{
	AdmitCtrlType admit_ctrl[6];
} RobotAdmitCtrl;

/**
* @brief 速度柔顺控制等级和比率等级设置
* 速度柔顺控制分三个等级，并且  1>rate1>rate2>rate3>rate4>0
* 等级为1时，只能设置rate1,rate2两个等级。rate3,rate4的值为0
* 等级为2时，只能设置rate1,rate2，rate3 三个等级。rate4的值为0
* 等级为3时，能设置 rate1,rate2，rate3,rate4 4个等级
*/
typedef struct
{
	int vc_level; //速度柔顺控制等级
	double rate1; //比率1等级
	double rate2; //比率2等级
	double rate3; //比率3等级
	double rate4; //比率4等级
} VelCom;

/**
* @brief 力传感器的受力分量和力矩分量
*/
typedef struct
{
	double fx; // 沿x轴受力分量
	double fy; // 沿y轴受力分量
	double fz; // 沿z轴受力分量
	double tx; // 绕x轴力矩分量
	double ty; // 绕y轴力矩分量
	double tz; // 绕z轴力矩分量
} FTxyz;

/**
* @brief ftp文件数据类型
*/
struct FtpFile
{
	const char *filename;
	FILE *stream;
};

/**
 *  @brief DH参数
 */
typedef struct
{
	double alpha[6];
	double a[6];
	double d[6];
	double joint_homeoff[6];
} DHParam;

/**
 *  @brief rs485信号量参数
 */
typedef struct
{
	char sig_name[20];//标识名
	int chn_id;		//RS485通道ID
	int sig_type;	//信号量类型
	int sig_addr;	//寄存器地址
	int value;		//值  设置时无效
	int frequency;	//信号量在控制器内部刷新频率不大于10
}SignInfo;

/**
 *  @brief rs485RTU配置参数
 */
typedef struct
{
	int chn_id;		//RS485通道ID  查询时chn_id作为输入参数
	int slaveId;	//当通道模式设置为Modbus RTU时，需额外指定Modbus从站节点ID，其余模式可忽略
	int baudrate;	//波特率4800,9600,14400,19200,38400,57600,115200,230400
	int databit;	//数据位7，8
	int stopbit;	//停止位1，2
	int parity;		//校验位78-> 无校验 79->奇校验 69->偶校验
}ModRtuComm;

#endif
