#ifndef MOTOR_H
#define MOTOR_H
#include "struct_typedef.h"
#include "PID.h"

//rm motor data
//����������ݽṹ��
typedef struct
{
	  //ID��
    uint16_t can_id;		
	    
	  //�������
    int16_t  set_voltage;		//������Ϣ
	
	  //�ٶ����
    int16_t  rotor_speed;		//���ڵ�ת��
	  int16_t  target_speed;  //Ŀ���ٶ�
	 
	  //�Ƕ����
    float rotor_angle;	  //���ڵĽǶ�
		float last_angle;		  //�ϴεĽǶ�
	  float angle_difl;     //�ǶȲ�����ۼӣ�Ҳ�������ڳ��ȼ��㣩
	  float absolute_angle; //���ԽǶ�
	
	  //λ�����
	  int16_t target_pos;
	

    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�

} dart_motor_t;

//dart total data
//����������ƽṹ��
typedef struct 
{
    dart_motor_t motor_info[8];     
    pid_struct_t motor_speed_pid[8];    
    pid_struct_t motor_pos_pid[8]; 


}dart_control;

extern dart_control  dart;
void motor_calc();
void motor_init();
	
#endif
