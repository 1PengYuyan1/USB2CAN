// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#ifndef __Tangair_usb2can__
#define __Tangair_usb2can__

#include <assert.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <sched.h>
#include <unistd.h>
#include "usb_can.h"
#include <memory>

// 辅助函数
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);

//达妙其他电机参数    {P, V, T}
				// {12.5, 30, 10 }, // DM4310
				// {12.5, 50, 10 }, // DM4310_48V
				// {12.5, 8, 28 },  // DM4340
				// {12.5, 10, 28 }, // DM4340_48V
				// {12.5, 45, 20 }, // DM6006
				// {12.5, 45, 40 }, // DM8006
				// {12.5, 45, 54 }, // DM8009
				// {12.5,25,  200}, // DM10010L
				// {12.5,20, 200},  // DM10010
				// {12.5,280,1},    // DMH3510
				// {12.5,45,10},    // DMH6215
				// {12.5,45,10}     // DMG6220

// 达妙电机,此处为DM10010L参数
// #define P_MIN -12.5f
// #define P_MAX 12.5f
// #define V_MIN -8.0f
// #define V_MAX 8.0f
// #define T_MIN -28.0f
// #define T_MAX 28.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f

#define MOTOR_DM4310 1
#define MOTOR_DM4340 2
#define MOTOR_DM8009 3
#define MOTOR_DM3519 4

#define PI (3.1415926f)

typedef struct
{
float P_MIN ;
float P_MAX ;
float V_MIN ;
float V_MAX ;
float T_MIN ;
float T_MAX ;
float KP_MIN ;
float KP_MAX ;
float KD_MIN ;
float KD_MAX ;

} MotorType_NumLink;

typedef struct
{
	uint16_t id;

	float position;
	float speed;
	float kp;
	float kd;
	float torque;

	float Angle;

	//位置限制
	float max_position;
	float min_position;

	MotorType_NumLink motor_type;

} Motor_CAN_Send_Struct;

typedef struct
{
	uint16_t ERR;

	uint16_t current_position; 		//
	uint16_t current_speed;	   		//
	uint16_t current_torque;   		//
	uint16_t current_temp_MOS;	    //
	uint16_t current_temp_Rotor;	//


	float current_position_f;		//
	float current_speed_f;			//
	float current_torque_f;			//
	float current_angle;			//

	MotorType_NumLink motor_type;

} Motor_CAN_Recieve_Struct;

typedef struct
{
	Motor_CAN_Send_Struct ID_1_motor_send, ID_2_motor_send, ID_3_motor_send,ID_4_motor_send, ID_5_motor_send, ID_6_motor_send;
	Motor_CAN_Recieve_Struct ID_1_motor_recieve, ID_2_motor_recieve, ID_3_motor_recieve,ID_4_motor_recieve, ID_5_motor_recieve, ID_6_motor_recieve;

} USB2CAN_CAN_Bus_Struct;



class Tangair_usb2can
{
public:
	
	bool all_thread_done_;
	bool running_;

	void Spin();

	Tangair_usb2can();

	~Tangair_usb2can();

  
	// CAN设备0
	int USB2CAN0_;
	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();

    int can_dev0_rx_count;
	int can_dev0_rx_count_thread;

	//can发送测试线程
	std::thread _CAN_TX_test_thread;
	//can发送测试线程函数
	void CAN_TX_test_thread();

    int speed_input;
    //can键盘输入线程
	std::thread _keyborad_input;
	//can发送测试线程函数
	void keyborad_input();

	/*****************************************************************************************************/
	/*********************************       ***电机相关***      ***********************************************/
	/*****************************************************************************************************/
	// 电机基本操作变量
	FrameInfo txMsg_CAN = {
		.canID = 0,
		.frameType = STANDARD,
		.dataLength = 8,
	};

	uint8_t Data_CAN[8];
	Motor_CAN_Send_Struct Motor_Data_Single;

	//接收暂存
	Motor_CAN_Recieve_Struct CAN_DEV0_RX;
	Motor_CAN_Recieve_Struct CAN_DEV1_RX;

	// 腿部数据
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_1; // 模块0，can1
	USB2CAN_CAN_Bus_Struct USB2CAN0_CAN_Bus_2;  // 模块0，can2

	//初始化
	void USB2CAN_CAN_Bus_inti_set(USB2CAN_CAN_Bus_Struct* Leg_Data);
	void USB2CAN_CAN_Bus_Init();

	void initMotorParams_Init_set(Motor_CAN_Send_Struct* motor, int motorType) ;
	void initMotorParams_Init();

	void Motor_Enable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Disable(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void Motor_Zore(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void CAN_Send_Control(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data); // 运控�??,CAN1=CAN_TX_MAILBOX0,CAN2=CAN_TX_MAILBOX1

	void Motor_Passive_SET(int32_t dev, uint8_t channel, Motor_CAN_Send_Struct *Motor_Data);

	void ENABLE_ALL_MOTOR(int delay_us);

	void DISABLE_ALL_MOTOR(int delay_us);

	void ZERO_ALL_MOTOR(int delay_us);

	void PASSIVE_ALL_MOTOR(int delay_us);

	void CAN_TX_ALL_MOTOR(int delay_us);

	float ShineNum(float x,float in_max,float in_min,float out_max,float out_min);

	void Shooting_tset(float x, float y, float z);

	void Pre_shoot();

	void Shoot_move();

	void back_move();

private:


};

#endif

// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
