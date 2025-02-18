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
 * File: jkerr.h
 * @author star@jaka
 * @date Nov-30-2021 
 *  
**************************************************************************/


#ifndef _JHERR_H_
#define _JHERR_H_

#define ERR_SUCC                 0              //success
#define ERR_FUCTION_CALL_ERROR   2              //interface error or controller not support
#define ERR_INVALID_HANDLER		 -1             //invalid handler
#define	ERR_INVALID_PARAMETER    -2             //invalid parameter
#define ERR_COMMUNICATION_ERR	 -3             //fail to connect
#define ERR_KINE_INVERSE_ERR     -4             //kine_inverse error
#define ERR_EMERGENCY_PRESSED    -5             //e-stop
#define ERR_NOT_POWERED          -6             //not power on
#define ERR_NOT_ENABLED          -7             //not enable
#define ERR_DISABLE_SERVOMODE    -8             //not in servo mode
#define ERR_NOT_OFF_ENABLE       -9             //must turn off enable before power off
#define ERR_PROGRAM_IS_RUNNING   -10            //cannot operate, program is running
#define ERR_CANNOT_OPEN_FILE     -11            //cannot open file, or file doesn't exist
#define ERR_MOTION_ABNORMAL      -12            //motion abnormal
#define ERR_FTP_PERFORM			 -14            //ftp error
#define ERR_VALUE_OVERSIZE       -15            // socket msg or value oversize
#define ERR_KINE_FORWARD         -16             //kine_forward error
#define ERR_EMPTY_FOLDER         -17             //not support empty folder


#define ERR_PROTECTIVE_STOP     -20            // protective stop
#define ERR_EMERGENCY_STOP      -21            // protective stop
#define ERR_SOFT_LIMIT          -22             // on soft limit

#define ERR_CMD_ENCODE          -30             // fail to encode cmd string
#define ERR_CMD_DECODE          -31             // fail to decode cmd string    
#define ERR_UNCOMPRESS          -32             // fail to uncompress port 10004 string     

#define ERR_MOVEL               -40             // move linear error
#define ERR_MOVEJ               -41             // move joint error
#define ERR_MOVEC               -42             // move circular error

#define ERR_MOTION_TIMEOUT          -50             // block_wait timeout
#define ERR_POWERON_TIMEOUT         -51             // power on timeout
#define ERR_POWEROFF_TIMEOUT        -52             // power off timeout
#define ERR_ENABLE_TIMEOUT          -53             // enable timeout
#define ERR_DISABLE_TIMEOUT         -54             // disable robot timeout
#define ERR_USERFRAME_SET_TIMEOUT   -55             // fail to set userframe
#define ERR_TOOL_SET_TIMEOUT        -56             // fail to set tool

#define ERR_IO_SET_TIMEOUT      -60             // fail to set IO
#define ERR_OPERATION_TIMEOUT      -61             // operation timeout

#define ERR_DEPRECATE -9998     // deprecated interface, will be removed in the future, function is still available
#define ERR_OBSOLETE -9999      // obsolete interface, will be removed in the future, function is unavailable


#endif
