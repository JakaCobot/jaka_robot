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
 * File: jktypes.h
 * @author star@jaka
 * @date Nov-30-2021 
 *  
**************************************************************************/


#ifndef _JHTYPES_H_
#define _JHTYPES_H_

#define TRUE 1
#define FALSE 0
#include <stdio.h>
#include <stdint.h>

typedef int BOOL;	 //SDK BOOL
typedef int JKHD;	 //robot control handler
typedef int errno_t; //SDK error code

/**
 *@brief 
 */
typedef struct{
	BOOL estoped;     // estop
	BOOL poweredOn;		// power on
	BOOL servoEnabled;	// enable robot or not
}RobotState;

/**
* @brief cartesian trans, unit: mm
*/
typedef struct
{
	double x;
	double y;
	double z;
} CartesianTran;

/**
* @brief cartesian ori, unit rad
*/
typedef struct
{
	double rx;
	double ry;
	double rz;
} Rpy;

/**
* @brief Quaternion
*/
typedef struct
{
	double s;
	double x;
	double y;
	double z;
} Quaternion;

/**
 *@brief cartesian pos including trans and ori
 */
typedef struct
{
	CartesianTran tran;
	Rpy rpy;
} CartesianPose;

/**
* @brief rot matrix
*/
typedef struct
{
	CartesianTran x; ///< x
	CartesianTran y; ///< y
	CartesianTran z; ///< z
} RotMatrix;

/**
* @brief program state enum
*/
typedef enum
{
	PROGRAM_IDLE,	 ///< idle
	PROGRAM_RUNNING, ///< running
	PROGRAM_PAUSED	 ///< paused
} ProgramState;

/**
* @brief coordinate type enum
*/
typedef enum
{
	COORD_BASE,
	COORD_JOINT,
	COORD_TOOL
} CoordType;

/**
* @brief jog mode enum
*/
typedef enum
{
	ABS = 0,
	INCR,
	CONTINUE
} MoveMode;


/**
* @brief payload
*/
typedef struct
{
	double mass;			///< unit ：kg
	CartesianTran centroid; ///< unit ：mm
} PayLoad;

/**
* @brief joint position, unit: rad
*/
typedef struct
{
	double jVal[6];
} JointValue;

/**
* @brief collision option
*/
typedef struct{
	int collisionMethod;
	int reactionType;
	float reboundAngle;
}CollisionOption;

/**
* @brief collision option setting type
*/
typedef enum {
	CollitionOption_ALL,
	CollitionOption_Method,
	CollitionOption_ReactionType,
	CollitionOption_ReboundAngle
}CollisionOptionSettingType;

/**
* @brief IO type enum
*/
typedef enum
{
	IO_CABINET, 		///< cabinet panel IO
	IO_TOOL,			///< tool IO
	IO_EXTEND,			///< extend IO(work as modbus master)
	IO_RELAY,   		///< relay IO，only available in CAB V3 (DO)
	IO_MODBUS_SLAVE, 	///< Modbus slave IO, index from 0
	IO_PROFINET_SLAVE, 	///< Profinet slave IO, index from 0
	IO_EIP_SLAVE      	///< ETHRENET/IP slave IO, index from 0
} IOType;


/**
* @brief callback
* @param info remember to assign char array, no less than 1024 bytes
				the feedback info contains at least 3 data part: "section", "key", "data"
				which is packed as json. The "data" part represent the current(new) value
*/
typedef void (*CallBackFuncType)(char* info);


/**
* @brief basic robot stat
*/
typedef struct
{
	int errcode;	///< 0: normal, others: errorcode
	char errmsg[200]; ///< controller errmsg
	
	int powered_on;	///< 0: power off，1: power on
	int enabled;	///< 0: disabled，1: enabled
} RobotStatus_simple;

/**
* @brief EXtio Data
*/
typedef struct
{
	int din[256];				  ///< Digital input din[0] is the number of valid signals
	int dout[256];				  ///< Digital output dout[0] is the number of valid signals
	float ain[256];				  ///< Analog input din[0] is the number of valid signals
	float aout[256];			  ///< Analog output dout[0] is the number of valid signals
} Io_group;

/**
* @brief Robot joint monitoring data
*/
typedef struct
{
	double instCurrent;		///< Instantaneous current
	double instVoltage;		///< Instantaneous voltage
	double instTemperature; ///< Instantaneous temperature
	double instVel;			///< Instantaneous speed controller 1.7.0.20 and above
	double instTorq;		///< Instantaneous torque
} JointMonitorData;

/**
* @brief Robot monitoring data
*/
typedef struct
{
	double scbMajorVersion;				  ///< scb major version number
	double scbMinorVersion;				  ///< scb minor version number
	double cabTemperature;				  ///< Controller temperature
	double robotAveragePower;			  ///< Robot average voltage
	double robotAverageCurrent;			  ///< Robot average current
	JointMonitorData jointMonitorData[6]; ///< Monitoring data of the robot's six joints
} RobotMonitorData;

/**
* @brief Torque sensor monitoring data
*/
typedef struct
{
	char ip[20];		 ///< Torque sensor IP address
	int port;			 ///< Torque sensor port number
	PayLoad payLoad;	 ///< Tool load
	int status;			 ///< Torque sensor status
	int errcode;		 ///< Torque sensor abnormal error code
	double actTorque[6]; ///< The actual contact force value of the torque sensor (when Initialize is checked) or the raw reading value (when Do Not Initialize is checked)
	double torque[6];	 ///< Torque sensor raw reading value
	double realTorque[6];///< The actual contact force value of the torque sensor (does not change with the initialization options)
} TorqSensorMonitorData;

/**
* @brief Robot status monitoring data, use the get_robot_status function to update the robot status data
*/
typedef struct
{
	int errcode;									///< Error number when the robot runs into an error. 0 means normal operation, and others mean abnormal operation.
	int inpos;										///< The robot movement is in place, 0 means not in place, 1 means in place
	int powered_on;									///< Whether the robot is powered on, 0 means no power, 1 means power on
	int enabled;									///< Flag indicating whether the robot is enabled, 0 means not enabled, 1 means enabled
	double rapidrate;								///< Robot movement ratio
	int protective_stop;							///< Whether the robot detects a collision, 0 means no collision is detected, 1 means a collision is detected
	int emergency_stop;								///< Whether the robot stops suddenly, 0 means no sudden stop, 1 means sudden stop
	int dout[256];									///< Digital output signal of the robot control cabinet, dout[0] is the number of signals
	int din[256];									///< Digital input signal of robot control cabinet, din[0] is the number of signals	
	double ain[256];								///< Robot control cabinet analog input signal, ain[0] is the number of signals
	double aout[256];								///< Robot control cabinet analog output signal, aout[0] is the number of signals
	int tio_dout[16];								///< The digital output signal of the robot end tool, tio_dout[0] is the number of signals
	int tio_din[16];								///< The digital input signal of the robot end tool, tio_din[0] is the number of signals
	double tio_ain[16];								///< Robot end tool analog input signal, tio_ain[0] is the number of signals
	int tio_key[3];                                 ///< Robot end tool buttons [0]free;[1]point;[2]pause_resume;
	Io_group extio;								    ///< Robot external application IO
	Io_group modbus_slave;							///< Robot Modbus Slave
	Io_group profinet_slave;						///< Robot Profinet Slave
	Io_group eip_slave;								///< Robot Ethernet/IP Slave
	unsigned int current_tool_id;					///< The tool coordinate system id currently used by the robot
	double cartesiantran_position[6];				///< The Cartesian space position of the robot end
	double joint_position[6];						///< Robot joint space position
	unsigned int on_soft_limit;						///< Whether the robot is in limit position, 0 means no limit protection is triggered, 1 means limit protection is triggered
	unsigned int current_user_id;					///< The user coordinate system id currently used by the robot
	int drag_status;								///< Whether the robot is in the dragging state, 0 means not in the dragging state, 1 means in the dragging state
	RobotMonitorData robot_monitor_data;			///< Robot status monitoring data
	TorqSensorMonitorData torq_sensor_monitor_data; ///< Robot torque sensor status monitoring data
	int is_socket_connect;							///< Whether the connection channel between SDK and controller is normal, 0 means the connection channel is abnormal, 1 means the connection channel is normal
} RobotStatus;

/**
 *@brief motion status
 */
typedef struct
{
	int motion_line; 		///< the executing motion cmd id
	int motion_line_sdk; 	///< reserved
	BOOL inpos; 			///< previous motion cmd is done, should alway check queue info together
	BOOL err_add_line; 		///< previous motion cmd is dropped by controller, like target is already reached
	int queue; 				///< motion cmd number in buffer
	int active_queue; 		///< motion cmd number in blending buffer
	BOOL queue_full;		///< motion buffer is full and motion cmds reveived at this moment will be dropped
	BOOL paused;			///< motion cmd is paused and able to resume
} MotionStatus;

/**
* @brief error information
*/
typedef struct
{
	long code;		   ///< error code
	char message[120]; ///< error message
} ErrorCode;

/**
* @brief traj config
*/
typedef struct
{
	double xyz_interval; ///< catesian trans sampling accuracy
	double rpy_interval; ///< catesian ori sampling accuracy
	double vel;			 ///< 
	double acc;			 ///< 
} TrajTrackPara;

#define MaxLength  256
/**
* @brief 
*/
typedef struct
{
	int len;			             ///< length
	char name[MaxLength][MaxLength]; ///< 
} MultStrStorType;

/**
* @brief not used
*/
typedef struct
{
	int executingLineId; ///< cmd id
} OptionalCond;

/**
* @brief operations when lost communication
*/
typedef enum
{
	MOT_KEEP,  ///< no change
	MOT_PAUSE, ///< pause
	MOT_ABORT  ///< abort
} ProcessType;

/**
* @brief admittance config
*/
typedef struct
{
	int opt;			 ///< 0: disable, 1: enable
	int axis;
	double ft_user;		 ///< force to let robot move with max speed
	double ft_rebound;	 ///< rebound ability
	double ft_constant;	 ///< const force
	int ft_normal_track; ///< 0: disable，1: enable
} AdmitCtrlType;

/**
* @brief tool drive config
*/
typedef struct
{
	int opt;			///< 0: disable, 1: enable
	int axis;			///< axis index, [0,5]
	double rebound;	 	///< rebound ability
	double rigidity;	///< 
} ToolDriveConfig;

/**
* @brief used for get/set_cst_force_ctrl_config
*/
typedef struct
{
	int opt;			///< 0: disable, 1: enable
	int axis;			///< axis index, [0,5]
	double rebound;	 	///< rebound ability
	double constant;	///< const force
	double rigidity; 	///< ftDamping
} ConstForceConfig;



/**
* @brief admittance config group
*/
typedef struct
{
	AdmitCtrlType admit_ctrl[6];
} RobotAdmitCtrl;

/**
* @brief 
*/
typedef struct
{
	ToolDriveConfig config[6];
} RobotToolDriveCtrl;

/**
* @brief 
*/
typedef struct
{
	ConstForceConfig config[6];
} RobotConstForceCtrl;

/**
 @brief 
 */
typedef enum{
	FTFrame_Tool = 0,
	FTFrame_World = 1
}FTFrameType;

/**
* @brief vel control level config
* 1>rate1>rate2>rate3>rate4>0
* level 1: only able to set rate1,rate2。rate3,rate4 are 0
* level 2，only able to set rate1,rate2，rate3。rate4 is 0
* level 3，able to set rate1,rate2，rate3,rate4
*/
typedef struct
{
	int vc_level; // control level
	double rate1; //
	double rate2; //
	double rate3; //
	double rate4; //
} VelCom;

/**
* @brief 
*/
typedef struct
{
	double fx;
	double fy;
	double fz;
	double tx;
	double ty;
	double tz;
} FTxyz;

/**
* @brief torque sensor data
*/
typedef struct
{
	int status;
	int errorCode;
	FTxyz data;
} TorqSensorData;

/**
* @brief ftp file/folder info
*/
struct FtpFile
{
	const char *filename;
	FILE *stream;
};

/**
* @brief torque sensor value type
*/
typedef enum
{
	Actual,	 ///< actual feedback
	General, ///< used by controller
	Real	 ///< real feedback without gravity and bias
} TorqSensorDataType;

/**
 *  @brief DH parameters
 */
typedef struct
{
	double alpha[6];
	double a[6];
	double d[6];
	double joint_homeoff[6];
} DHParam;

/**
 *  @brief rs485 signal info
 */
typedef struct
{
	char sig_name[20];
	int chn_id;		
	int sig_type;	
	int sig_addr;	
	int value;		
	int frequency;	//no more than 10
}SignInfo;

/**
 *  @brief rs485RTU config
 */
typedef struct
{
	int chn_id;		//RS485 channel ID
	int slaveId;	//must set Modbus slave ID when modbus RTU
	int baudrate;	//4800,9600,14400,19200,38400,57600,115200,230400
	int databit;	//7，8
	int stopbit;	//1，2
	int parity;		//78: no check,  79: odds parity,  69: even parity
}ModRtuComm;

/**
 *@brief joint move param
 */
typedef struct{
	int id;				///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
	BOOL is_block;		///< block until this cmd is done
	JointValue joints;	///< targe joint value
	MoveMode mode;		///< motion mode
	double vel;			///< velocity
	double acc;			///< acceleration, set to 90 if you have no idea
	double tol;			///< tolerance, used for blending. set to 0 if you want to reach a fine point
} MoveJParam;

typedef struct{
	int id;					///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
	BOOL is_block;			///< block until this cmd is done
	CartesianPose end_pos;	///< taget position
	MoveMode move_mode;		///< motion mode
	double vel;				///< velocity
	double acc;				///< acceleration, set to 500 if you have no idea
	double tol;				///< tolerance, used for blending. set to 0 if you want to reach a fine point
	double ori_vel;			///< set to 3.14 if you have no idea
	double ori_acc;			///< set to 12.56 if you have no idea
} MoveLParam;

typedef struct{
	int id;					///< motion cmd id, range limit: [0, 5000], set to -1 if you want controller to set automatically
	BOOL is_block;			///< block until this cmd is done
	CartesianPose mid_pos;	///< mid position
	CartesianPose end_pos;	///< end position
	MoveMode move_mode;		///< motion mode
	double vel;				///< velocity
	double acc;				///< acceleration, set to 500 if you have no idea
	double tol;				///< tolerance, used for blending. set to 0 if you want to reach a fine point
	
	double circle_cnt;		///< circule count
	int circle_mode;		///< clock wise or counter clock wise
} MoveCParam;

/**
 *@brief Controller User Variable Struct
 *@param id controller inner usage
 *@param value value type always double
 *@param alias variable alias which is less than 100 bytes
 */
typedef struct {
	int id;
	double value;
	char alias[100];
} UserVariable;

/**
 * @brief number of UserVariable is fixed to 100
 */
typedef struct{
	UserVariable v[100];
} UserVariableList;


#endif
