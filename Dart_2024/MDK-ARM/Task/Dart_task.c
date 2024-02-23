#include "stm32f4xx.h"                  // Device header
#include "Dart_task.h" 
#include "cmsis_os.h"
#include "gpio.h"
#include "remote_control.h"
#include "motor.h"
#include "can_user.h"
#include "switch.h"
#include <stdlib.h> 
#include <math.h>
//变量定义==============================================
//PID
fp32 motor_speed_pid[3]={1,0,0};
fp32 motor_pos_pid[3] = {1,0,1};
uint8_t dart_adjust_falg[3] = {0};

uint8_t flag[4] = {0};
uint8_t speed1;
uint8_t speed2;
//函数声明==============================================
static void dart_init();
static void mode_choose();
static void dart_adjust();
static void dart_mode_reset();   //飞镖电机编码值置0
static void dart_control_mode(); //飞镖遥控器控制
static void dart_pos_mode();     //飞镖位置控制
static void calc_and_send();


//主函数==============================================
void Dart_task(void const * argument)
{ 
  dart_init();
  motor_init();
  for(;;)
  {
		dart_adjust();
		motor_calc();
	  //mode_choose();
		dart_control_mode();
    calc_and_send();		
    osDelay(1);
		for(uint8_t i=0;i<4;i++)
		{
			flag[i] = get_switch(i);
		}
  }
}

//初始化==============================================
void dart_init()
{
  for (uint8_t i = 4; i < 8; i++)
	{
        pid_init(&dart.motor_speed_pid[i], motor_speed_pid, 16000, 16000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384			
	      //pid_init(&dart.motor_pos_pid[i], motor_pos_pid, 10000, 10000);
	}   
}

//模式选择=======================================+-*=======
//遥控器 左S s[1] 132   右S s[0] 132
// 左右ch[2] 上下ch[3]  左右ch[0] 上下ch[1]
void mode_choose()
{
  
//	if(rc_ctrl.rc.s[0] == 2)
//	{
//		 dart_adjust();
//	}
//	else
//	{
//		dart_adjust_falg[1] = 0;
//	}
	
	if(rc_ctrl.rc.s[0] == 3)
	{
		  dart_control_mode();
	}
  
}


//遥控器控制飞镖
void dart_control_mode()
{
	//YAW轴
	if(rc_ctrl.rc.ch[0]<-110 && abs(dart.motor_info[7].torque_current) < 4000 && get_switch(1) == 0)
	{
		dart.motor_info[7].target_speed = -rc_ctrl.rc.ch[0]*20;
	}
	else if(rc_ctrl.rc.ch[0] > 110 && abs(dart.motor_info[2].torque_current) < 4000 && dart.motor_info[7].absolute_angle < 2900)
	{
		dart.motor_info[7].target_speed = -rc_ctrl.rc.ch[0]*20;
	}
	else
	{
		dart.motor_info[7].target_speed = 0;
	}
	
	//PITCH轴
	if(rc_ctrl.rc.ch[1] < -110 && abs(dart.motor_info[2].torque_current) < 4000 && get_switch(0) == 0)
	{
		dart.motor_info[6].target_speed = rc_ctrl.rc.ch[1]*20;
		dart.motor_info[5].target_speed = rc_ctrl.rc.ch[1]*20;
	}
	else if(rc_ctrl.rc.ch[1] > 110 && abs(dart.motor_info[2].torque_current) < 4000 && dart.motor_info[6].absolute_angle < 3000)
	{
		dart.motor_info[6].target_speed = rc_ctrl.rc.ch[1]*20;
		dart.motor_info[5].target_speed = rc_ctrl.rc.ch[1]*20;
	}
	else
	{
		dart.motor_info[6].target_speed = 0;
		dart.motor_info[5].target_speed = 0;
	}	
}

//编码器校准=========================================
void dart_adjust()
{
		//pitch轴编码器置0
	  if(get_switch(0) == 1)
	  {
		  dart.motor_info[6].absolute_angle = 0;
		  dart.motor_info[5].absolute_angle = 0;
	  }
//		if(get_switch(0)==0 && dart_adjust_falg[0] == 0)
//		{
//			dart.motor_info[6].target_speed = 200;
//			dart.motor_info[5].target_speed = 200; 
//		}
//		else
//		{
//			dart.motor_info[6].target_speed   = 0;
//			dart.motor_info[5].target_speed   = 0;
//			dart.motor_info[6].absolute_angle = 0;
//			dart.motor_info[5].absolute_angle = 0;
//			dart_adjust_falg[0] = 1;
//		}
		
		//yaw轴编码置0
	   if(get_switch(1) == 1)
	   {
		    dart.motor_info[7].absolute_angle = 0;
	   }
//		if(get_switch(1)==0 && dart_adjust_falg[1] == 0)
//		{
//			dart.motor_info[7].target_speed = 200;
//		}
//		else
//		{
//			dart.motor_info[7].target_speed = 0;
//			dart.motor_info[7].absolute_angle = 0;
//			dart_adjust_falg[1] = 1;
//		}

}
//PID计算==============================================
void calc_and_send()
{
	for(uint16_t index = 0;	index<8;index++)
	{
			dart.motor_info[index].set_voltage = pid_calc(&dart.motor_speed_pid[index],
		                                                 dart.motor_info[index].target_speed,
		                                                 dart.motor_info[index].rotor_speed);
	}
	
	can2_cmd_motor(1,dart.motor_info[4].set_voltage,
	                 dart.motor_info[5].set_voltage,
	                 dart.motor_info[6].set_voltage,
	                 dart.motor_info[7].set_voltage);

}



//暂时没用
//void dart_pos_mode()
//{
//	  dart.motor_info[0].target_pos   = 8191/2 + 3*rc_ctrl.rc.ch[0];
//		dart.motor_info[0].target_speed = pos_pid_calc(&dart.motor_pos_pid[0],
//	                                          dart.motor_info[0].target_pos,
//	                                          dart.motor_info[0].absolute_angle);	   
//}