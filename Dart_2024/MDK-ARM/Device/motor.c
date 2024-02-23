#include "motor.h"
#include "struct_typedef.h"
#include "can_user.h"
//3508��ʶ�� 0x200 0x1FF   ������0x200+ID
//2006��ʶ�� 0x200 0x1FF   ������0x200+ID
//6020��ʶ�� 0x1FF 0x2FF   ������0x204+ID

/*
can1 6020*3 
can2 2006*2 3508*2
*/

dart_control  dart;

void motor_2006_push_calc(uint16_t i);
void motor_3508_calc(uint16_t i);
void motor_2006_calc(uint16_t i);
void motor_pitch_calc(uint16_t i);
void motor_yaw_calc(uint16_t i);

//�����ʼ������
void motor_init()
{
	for(uint8_t index = 0;index<7;index++)
	{
	   dart.motor_info[index].last_angle = dart.motor_info[index].rotor_angle;
	}
}

//������㺯��
void motor_calc()
{
	//�Ƹ�
	motor_2006_push_calc(2);
	//pitch
	motor_pitch_calc(5);
	motor_pitch_calc(6);
	//yaw
	motor_yaw_calc(7);
	//����
	motor_2006_calc(3);
}


//2006�����Ƹ�
void motor_2006_push_calc(uint16_t i)
{

	dart.motor_info[i].angle_difl = dart.motor_info[i].rotor_angle - dart.motor_info[i].last_angle;  //3508ת���ĽǶȲ�
	dart.motor_info[i].last_angle = dart.motor_info[i].rotor_angle;       
	
	if(dart.motor_info[i].angle_difl>8191/2)
	{
		dart.motor_info[i].angle_difl -= 8191;
	}
	else if(dart.motor_info[i].angle_difl <- 8191/2)
	{
		dart.motor_info[i].angle_difl +=8191;
	}
	
	dart.motor_info[i].angle_difl/=5000;               //���ٱ� 
	
	dart.motor_info[i].absolute_angle -= dart.motor_info[i].angle_difl;  //�ǶȲ��ۼ�,�õ����ԽǶ�
	
	if(dart.motor_info[i].absolute_angle>8191)
	{
		dart.motor_info[i].absolute_angle -= 8191;
	}
	else if(dart.motor_info[i].absolute_angle<0)
	{
		dart.motor_info[i].absolute_angle+=8191;
	}
	
}

//3508����������������ٱ�����
void motor_3508_calc(uint16_t i)
{
	dart.motor_info[i].angle_difl = dart.motor_info[i].rotor_angle - dart.motor_info[i].last_angle;  //3508ת���ĽǶȲ�
	dart.motor_info[i].last_angle = dart.motor_info[i].rotor_angle;       
	
	if(dart.motor_info[i].angle_difl>8191/2)
	{
		dart.motor_info[i].angle_difl -= 8191;
	}
	else if(dart.motor_info[i].angle_difl <- 8191/2)
	{
		dart.motor_info[i].angle_difl +=8191;
	}
	
	dart.motor_info[i].angle_difl/=(3591/187);               //���ٱ� 
	
	dart.motor_info[i].absolute_angle += dart.motor_info[i].angle_difl;  //�ǶȲ��ۼ�,�õ����ԽǶ�
	
	if(dart.motor_info[i].absolute_angle>8191)
	{
		dart.motor_info[i].absolute_angle -= 8191;
	}
	else if(dart.motor_info[i].absolute_angle<0)
	{
		dart.motor_info[i].absolute_angle+=8191;
	}													
}

//2006����������������ٱ�����
void motor_2006_calc(uint16_t i)
{
	dart.motor_info[i].angle_difl = dart.motor_info[i].rotor_angle - dart.motor_info[i].last_angle;  //3508ת���ĽǶȲ�
	dart.motor_info[i].last_angle = dart.motor_info[i].rotor_angle;       
	
	if(dart.motor_info[i].angle_difl>8191/2)
	{
		dart.motor_info[i].angle_difl -= 8191;
	}
	else if(dart.motor_info[i].angle_difl <- 8191/2)
	{
		dart.motor_info[i].angle_difl +=8191;
	}
	
	dart.motor_info[i].angle_difl/=360;               //���ٱ� 
	
	dart.motor_info[i].absolute_angle += dart.motor_info[i].angle_difl;  //�ǶȲ��ۼ�,�õ����ԽǶ�
	
	if(dart.motor_info[i].absolute_angle>8191)
	{
		dart.motor_info[i].absolute_angle -= 8191;
	}
	else if(dart.motor_info[i].absolute_angle<0)
	{
		dart.motor_info[i].absolute_angle+=8191;
	}													
}

//6020���������
void motor_pitch_calc(uint16_t i)
{
	dart.motor_info[i].angle_difl = dart.motor_info[i].rotor_angle - dart.motor_info[i].last_angle;  //3508ת���ĽǶȲ�
	dart.motor_info[i].last_angle = dart.motor_info[i].rotor_angle;       
	
	if(dart.motor_info[i].angle_difl>8191/2)
	{
		dart.motor_info[i].angle_difl -= 8191;
	}
	else if(dart.motor_info[i].angle_difl <- 8191/2)
	{
		dart.motor_info[i].angle_difl +=8191;
	}
	
	dart.motor_info[i].angle_difl/=100;               //���ٱ� 
	
	dart.motor_info[i].absolute_angle += dart.motor_info[i].angle_difl;  //�ǶȲ��ۼ�,�õ����ԽǶ�
	
	if(dart.motor_info[i].absolute_angle>8191)
	{
		dart.motor_info[i].absolute_angle -= 8191;
	}
	else if(dart.motor_info[i].absolute_angle<0)
	{
		dart.motor_info[i].absolute_angle+=8191;
	}													
}

//6020���������
void motor_yaw_calc(uint16_t i)
{
	dart.motor_info[i].angle_difl = dart.motor_info[i].rotor_angle - dart.motor_info[i].last_angle;  //3508ת���ĽǶȲ�
	dart.motor_info[i].last_angle = dart.motor_info[i].rotor_angle;       
	
	if(dart.motor_info[i].angle_difl>8191/2)
	{
		dart.motor_info[i].angle_difl -= 8191;
	}
	else if(dart.motor_info[i].angle_difl <- 8191/2)
	{
		dart.motor_info[i].angle_difl +=8191;
	}
	
	dart.motor_info[i].angle_difl/=100;               //���ٱ� 
	
	dart.motor_info[i].absolute_angle -= dart.motor_info[i].angle_difl;  //�ǶȲ��ۼ�,�õ����ԽǶ�
	
	if(dart.motor_info[i].absolute_angle>8191)
	{
		dart.motor_info[i].absolute_angle -= 8191;
	}
	else if(dart.motor_info[i].absolute_angle<0)
	{
		dart.motor_info[i].absolute_angle+=8191;
	}													
}


